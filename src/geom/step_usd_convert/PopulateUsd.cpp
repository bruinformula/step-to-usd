#include <iostream>
#include <optional>
#include <string>
#include <filesystem>
#include <map>
#include <mutex>
#include <unordered_set>
#include <utility>
#include <vector>
#include <stddef.h>

#include <opencascade/TDF_Label.hxx>
#include <opencascade/TopoDS_Shape.hxx>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/pxr.h>
#include <pxr/usd/usd/common.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/editTarget.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/primFlags.h>
#include <pxr/usd/usd/relationship.h>

#include <pxr/base/work/loops.h>
#include <pxr/base/work/workTBB/loops_impl.h>
#include <pxr/base/tf/error.h>
#include <pxr/base/tf/errorMark.h>
#include <pxr/base/tf/staticData.h>
#include <pxr/base/tf/token.h>

#include <pxr/usd/usd/inherits.h>
#include <pxr/usd/usd/payloads.h>
#include <pxr/usd/usd/variantSets.h>
#include <pxr/usd/usd/editContext.h>
#include <pxr/usd/usd/specializes.h>

#include <pxr/usd/usdGeom/scope.h>
#include <pxr/usd/usdGeom/imageable.h>
#include <pxr/usd/usdGeom/tokens.h>

#include <pxr/usd/sdf/declareHandles.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/payload.h>
#include <pxr/usd/sdf/primSpec.h>
#include <pxr/usd/sdf/proxyTypes.h>
#include <pxr/usd/sdf/reference.h>

#pragma pop_macro("Handle")

#include "stepTessellationAPI.h"
#include "stepFileContainerAPI.h"

#include "UsdStepExporter.h"
#include "StepModel.h"

PXR_NAMESPACE_USING_DIRECTIVE

TessParams getTessParams(
    UsdPrim prim,
    const TessParams& defaultParams
) {
    AutolibStepTessellationAPI api(prim);

    TessParams params = defaultParams;

    updateIfAuthored(api.GetStepMeshLinearDeflectionAttr(), &params.meshLinearDeflection);
    updateIfAuthored(api.GetStepMeshAngularDeflectionAttr(), &params.meshAngularDeflection);
    updateIfAuthored(api.GetStepMeshMinSizeAttr(), &params.meshMinSize);

    updateIfAuthored(api.GetStepWireframeDeflectionAttr(), &params.wireframeDeflection);
    updateIfAuthored(api.GetStepWireframeTypeAttr(), &params.wireframeMode.type);
    updateIfAuthored(api.GetStepWireframeSamplingAttr(), &params.wireframeMode.sampling);

    updateIfAuthored(api.GetStepSketchDeflectionAttr(), &params.sketchDeflection);
    updateIfAuthored(api.GetStepSketchTypeAttr(), &params.sketchMode.type);
    updateIfAuthored(api.GetStepSketchSamplingAttr(), &params.sketchMode.sampling);

    updateIfAuthored(api.GetStepRenderPurposeThresholdAttr(), &params.renderPurposeThreshold);
    updateIfAuthored(api.GetStepSelfIntersectionThresholdAttr(), &params.selfIntersectionThreshold);
    updateIfAuthored(api.GetStepMaxNumberRemeshPassesAttr(), &params.maxNumberRemeshPasses);

    return params;
}

void resolveParamsRecursive(
    const UsdPrim& prim, 
    const TessParams& currentParams, 
    std::map<SdfPath, TessParams>& results
) {
    TessParams primParams = getTessParams(prim, currentParams);
    
    std::vector<std::string> vsetNames;
    prim.GetVariantSets().GetNames(&vsetNames);

    if (vsetNames.empty()) {
        results[prim.GetPath()] = primParams;
        for (const UsdPrim& child : prim.GetChildren()) {
            resolveParamsRecursive(child, primParams, results);
        }
        return;
    }

    // Since a prim can have multiple variant sets, need a resolution 
    // of all variant selections to fully evaluate every permutation 
    // authored on this prim.
    for (const std::string& name : vsetNames) {
        UsdVariantSet vset = prim.GetVariantSet(name);
        std::vector<std::string> variantNames = vset.GetVariantNames();

        bool hasDefault = false;
        for (const std::string& vn : variantNames) {
            if (vn == "default") hasDefault = true;
        }
        if (!hasDefault) {
            variantNames.insert(variantNames.begin(), "default");
        }

        for (const std::string& variantName : variantNames) {
            if (variantName == "default" && !hasDefault) {
                vset.ClearVariantSelection();
            } else {
                vset.SetVariantSelection(variantName);
            }
            
            // Re-evaluate params in case this variant authos new opinions
            TessParams variantParams = getTessParams(prim, primParams);
            SdfPath variantPath = prim.GetPath().AppendVariantSelection(name, variantName);
            
            results[variantPath] = variantParams;
            
            for (const UsdPrim& child : prim.GetChildren()) {
                resolveParamsRecursive(child, variantParams, results);
            }
        }
    }
}

std::map<SdfPath, TessParams> resolveParams(
    const UsdPrim& rootPrim, // prototypes
    const TessParams& defaultParams
) {
    bool initialValue = rootPrim.IsActive();
    rootPrim.SetActive(true);
    std::map<SdfPath, TessParams> results;

    for (const UsdPrim& child : rootPrim.GetChildren()) {
        resolveParamsRecursive(child, defaultParams, results);
    }

    rootPrim.SetActive(initialValue);
    return results;
}

std::optional<SdfReference> UsdStepExporter::getPrototypesDefaultParams(const UsdPrim& rootPrim) {
    AutolibStepFileContainerAPI api(rootPrim);

    SdfPathVector targets;
    api.GetStepDefaultParamsRel().GetForwardedTargets(&targets);

    if (targets.empty()) {
        std::cerr << "Warning: No default params target specified on root prim. Using hardcoded defaults.\n";
        return std::nullopt;
    }

    UsdPrim targetPrim = rootPrim.GetStage()->GetPrimAtPath(targets[0]);

    if (!targetPrim.IsValid()) {
        std::cerr << "Warning: Default params target " << targets[0] << " is invalid. Using hardcoded defaults.\n";
        return std::nullopt;
    }

    SdfPrimSpecHandleVector stack = targetPrim.GetPrimStack();

    if (stack.empty()) {
        std::cerr << "Warning: Default params target " << targets[0] << " has empty prim stack. Using hardcoded defaults.\n";
        return std::nullopt;
    }

    SdfReference reference(stack[0]->GetLayer()->GetIdentifier(), stack[0]->GetPath());

    return reference;
}

struct PrototypeContainer {
    std::string variantSetName;
    std::string variantName;
    fs::path filePath;
    UsdStageRefPtr stage;
};

// Flattened tessellation target
struct TessellationJob {
    const PrototypeContainer* proto;
    int defIndex;
    SdfPath prototypePath;
    TessParams params;
    TessResult result;
};

void UsdStepExporter::populateUsd(
    const StepModel& model, 
    UsdStageRefPtr rootStage,
    UsdPrim& rootPrim,
    const std::map<std::string, std::vector<std::string>>& variantSetNameToVariantNames,
    const std::unordered_set<SdfPath, SdfPath::Hash>& prototypesFilter
) {
    TfErrorMark mark;
    rootStage->Unload();

    fs::path rootFilePath = fs::canonical(rootStage->GetRootLayer()->GetResolvedPath().GetPathString()).remove_filename();
    std::string baseName = model.stepPath.stem().string();

    SdfPath assemblyPath("/Assembly");
    SdfPath prototypesPath("/Prototypes");
    SdfPath rootPrimPath = rootPrim.GetPath();
    SdfPath assemblyInRootPath = rootPrimPath.AppendChild(TfToken("Assembly"));
    SdfPath prototypesInRootPath = rootPrimPath.AppendChild(TfToken("Prototypes"));

    // Clear existing payloads from the root stage beforehand to prevent OpenUSD core crashes
    // and noisy warnings when the prototype layers are modified later on disk.
    {
        SdfChangeBlock block;
        if (variantSetNameToVariantNames.empty()) {
            if (UsdPrim p = rootStage->GetPrimAtPath(prototypesInRootPath))
                p.GetPayloads().ClearPayloads();
        } else {
            for (const auto& entry : variantSetNameToVariantNames) {
                if (UsdVariantSet varSet = rootPrim.GetVariantSet(entry.first)) {
                    for (const auto& variant : entry.second) {
                        varSet.SetVariantSelection(variant);
                        UsdEditContext ctx(varSet.GetVariantEditContext());
                        UsdPrim pPrim = rootStage->OverridePrim(prototypesInRootPath);
                        pPrim.GetPayloads().ClearPayloads();
                    }
                }
            }
        }
    }

    bool makeFreshStage = prototypesFilter.empty();
    std::vector<PrototypeContainer> prototypes;
    std::mutex protoMutex;

    // Setup Prototype Stages for all variants
    if (variantSetNameToVariantNames.empty()) {
        fs::path prototypesStageFilePath = rootFilePath / (baseName + "-prototypes.usda");
        
        UsdStageRefPtr prototypesStage = UsdStepExporter::initUsdStage(prototypesStageFilePath, rootPrimPath, makeFreshStage);
        
        UsdPrim existingPrototypesRoot = prototypesStage->GetPrimAtPath(prototypesPath);
        if (existingPrototypesRoot.IsValid() && !existingPrototypesRoot.IsActive()) existingPrototypesRoot.SetActive(true);

        prototypesStage->SetMetadata(TfToken("metersPerUnit"), model.metersPerUnit);
        UsdGeomScope prototypesScope = UsdGeomScope::Define(prototypesStage, prototypesPath);
        prototypesStage->SetDefaultPrim(prototypesScope.GetPrim());
        prototypesStage->Save();

        PrototypeContainer container{"", "", prototypesStageFilePath, prototypesStage};
        prototypes.push_back(container);
    } else {
        baseName += "-";
        for (const auto& entry : variantSetNameToVariantNames) {
            const std::string& set = entry.first;
            fs::path variantSubPath = rootFilePath / set;
            if (!fs::exists(variantSubPath) && !fs::create_directory(variantSubPath)) continue;

            for (const auto& variant : entry.second) {
                fs::path prototypesStageFilePath = variantSubPath / (baseName + set + "-" + variant + "-prototypes.usda");
                
                UsdStageRefPtr prototypesStage = UsdStepExporter::initUsdStage(prototypesStageFilePath, rootPrimPath, makeFreshStage);

                UsdPrim existingPrototypesRoot = prototypesStage->GetPrimAtPath(prototypesPath);
                if (existingPrototypesRoot.IsValid() && !existingPrototypesRoot.IsActive()) existingPrototypesRoot.SetActive(true);

                prototypesStage->SetMetadata(TfToken("metersPerUnit"), model.metersPerUnit);
                prototypesStage->SetDefaultPrim(UsdGeomScope::Define(prototypesStage, prototypesPath).GetPrim());
                prototypesStage->Save();

                PrototypeContainer container{set, variant, prototypesStageFilePath, prototypesStage};
                prototypes.push_back(container);
            }
        }
    }

    // Assembly Stage
    fs::path assemblyStageFilePath = rootFilePath / (model.stepPath.stem().string() + "-assembly.usdc");
    UsdStageRefPtr assemblyStage = UsdStepExporter::initUsdStage(assemblyStageFilePath, rootPrimPath, makeFreshStage);
    rootPrim = rootStage->GetPrimAtPath(rootPrimPath);
    
    assemblyStage->SetMetadata(TfToken("metersPerUnit"), model.metersPerUnit);
    assemblyStage->SetDefaultPrim(assemblyStage->OverridePrim(rootPrimPath));
    UsdGeomScope::Define(assemblyStage, assemblyInRootPath);

    SdfLayerHandle rootLayer = rootStage->GetRootLayer();
    std::string assemblyRelativeFilePath = fs::relative(assemblyStageFilePath, rootFilePath).string();
    
    // Write Xforms
    std::vector<std::pair<TDF_Label, TopoDS_Shape>> defs(model.definitionShapes.begin(), model.definitionShapes.end());
    LabelMap<SdfPath> prototypePaths;

    for (const auto& proto : prototypes) {
        writeCadPart(proto.stage, rootPrim, SdfPath("/CADPart"));
        writePrototypeXformsInPrototypesStage(proto.stage, rootPrim, defs, prototypesPath, prototypesFilter, prototypePaths, "", makeFreshStage);
        proto.stage->Save();
    }

    writePrototypeOverridesInAssemblyStage(assemblyStage, rootPrim, prototypePaths);
    std::vector<SdfPath> nodePaths = computeNodePaths(model.partNodes, assemblyInRootPath);
    
    if (makeFreshStage) writeAssemblyXforms(assemblyStage, rootPrim.GetPrimPath(), model.partNodes, nodePaths, prototypePaths);
    assemblyStage->Save();

    bool alreadyExists = false;
    for (const auto& path : rootLayer->GetSubLayerPaths()) {
        if (path == assemblyRelativeFilePath) { alreadyExists = true; break; }
    }
    if (!alreadyExists) rootLayer->InsertSubLayerPath(assemblyRelativeFilePath);
    assemblyStage->Save();


    // Payload logic on root stage
    for (const auto& proto : prototypes) {
        std::string payloadPath = fs::relative(proto.filePath, rootFilePath).string();
        
        if (!proto.variantSetName.empty()) {
            UsdVariantSet varSet = rootPrim.GetVariantSet(proto.variantSetName);
            varSet.SetVariantSelection(proto.variantName);
            UsdEditContext ctx(varSet.GetVariantEditContext());
            UsdPrim pPrim = rootStage->OverridePrim(prototypesInRootPath);
            pPrim.GetPayloads().ClearPayloads();
            pPrim.GetPayloads().AddPayload(SdfPayload(payloadPath, prototypesPath));
        } else {
            UsdPrim pPrim = rootStage->OverridePrim(prototypesInRootPath);
            pPrim.GetPayloads().ClearPayloads();
            pPrim.GetPayloads().AddPayload(SdfPayload(payloadPath, prototypesPath));
        }
    }
    
    rootStage->GetRootLayer()->Save();
    rootStage->Reload();
    rootPrim = rootStage->GetPrimAtPath(rootPrimPath);
    rootStage->Unload(rootPrim.GetPath());

    // Flatten Tessellation Jobs
    std::vector<TessellationJob> tessJobs;
    for (const auto& proto : prototypes) {
        if (!proto.variantSetName.empty()) {
            UsdVariantSet varSet = rootPrim.GetVariantSet(proto.variantSetName);
            varSet.SetVariantSelection(proto.variantName);
        }
        
        UsdPrim protoRootPrim = rootStage->GetPrimAtPath(prototypesInRootPath);
        TessParams rootParams = getTessParams(rootPrim);
        TessParams variantLevelParams = getTessParams(protoRootPrim, rootParams);
        std::map<SdfPath, TessParams> paramsBank = resolveParams(protoRootPrim, variantLevelParams);

        for (size_t i = 0; i < defs.size(); ++i) {
            SdfPath protoPath = prototypePaths.at(defs[i].first); // /Prototypes/rod0

            SdfPath paramKeyPath = prototypesInRootPath.AppendChild(protoPath.GetNameToken());
            
            // Re-apply any variant extensions based on variants stored in `proto` 
            // However, resolveParams currently spits out nested paths if variants were found inside it.
            // If wonderful_model.usda has per-prototype opinions like `/Prototypes/rod0{quality=draft}`
            // we must check the paramsBank if there was a variant block nested matching those specific opinions

            TessParams params = variantLevelParams;
            
            bool foundVariantForProto = false;
            for (const auto& kv : paramsBank) {
                if (kv.first.GetPrimPath() == paramKeyPath) {
                    foundVariantForProto = true;
                    params = kv.second;
                    
                    // We need the prototypePath to include the variant selections so writer knows where to author
                    SdfPath jobProtoPath = protoPath;
                    auto variantSelection = kv.first.GetVariantSelection();
                    if (!variantSelection.first.empty()) {
                        jobProtoPath = jobProtoPath.AppendVariantSelection(variantSelection.first, variantSelection.second);
                    }
                    
                    std::cout << "DEBUG: Queueing job for " << jobProtoPath << " (defIndex " << i << ")\n";
                    tessJobs.push_back({&proto, (int)i, jobProtoPath, params, TessResult()});
                }
            }
            
            if (!foundVariantForProto) {
                tessJobs.push_back({&proto, (int)i, protoPath, params, TessResult()});
            }
        }
    }

    // Tessellation
    TessParams rootParams = getTessParams(rootPrim);
    
    std::vector<size_t> defIndices(defs.size());
    for (size_t i = 0; i < defs.size(); ++i) defIndices[i] = i;

    WorkParallelForEach(defIndices.begin(), defIndices.end(), [&](size_t idx) {
        for (TessellationJob& job : tessJobs) {
            if (job.defIndex == idx) {
                bool bTesselate = true;
                if (!prototypesFilter.empty() && prototypesFilter.find(job.prototypePath) == prototypesFilter.end()) {
                    bTesselate = false;
                }

                if (bTesselate) {
                    tesselatePart(job.result, defs[idx].second, job.params);
                }
            }
        }
    });

    // Write Geometry
    for (const auto& proto : prototypes) {
        std::vector<ProtoGeomJob> geomJobs;
        for (const auto& job : tessJobs) {
            if (job.proto == &proto) {
                geomJobs.push_back({job.prototypePath, job.result, job.params});
            }
        }
        
        writePrototypeGeometries(proto.stage, geomJobs, "", prototypesFilter);
        proto.stage->Save();
    }

    UsdPrim prototypeRoot = rootStage->GetPrimAtPath(prototypesInRootPath);
    if (prototypeRoot.IsValid()) {
        prototypeRoot.SetActive(false);
        rootStage->GetRootLayer()->Save();
    }

    if (!mark.IsClean()) {
        for (const auto& error : mark) std::cerr << "Usd: " << error.GetCommentary() << "\n";
    }
}