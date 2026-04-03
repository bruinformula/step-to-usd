#include <iostream>
#include <optional>
#include <string>
#include <filesystem>
#include <map>
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

#include <pxr/usd/usdGeom/xform.h>
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
#include "stepFilePrototypesAPI.h"
#include "stepFilePrototypes.h"

#include "UsdStepExporter.h"
#include "StepModel.h"
#include "Logger.h"

PXR_NAMESPACE_USING_DIRECTIVE

static TessParams getTessParams(
    UsdPrim prim,
    const TessParams& defaultParams = {}
) {
    TessParams params = defaultParams;

    if (prim.HasAPI<AutolibStepFilePrototypesAPI>()) {
        AutolibStepFilePrototypesAPI protoApi(prim);
        SdfPathVector targets;
        protoApi.GetStepDefaultParamsRel().GetForwardedTargets(&targets);
        if (!targets.empty()) {
            UsdPrim targetPrim = prim.GetStage()->GetPrimAtPath(targets[0]);
            if (targetPrim.IsValid() && targetPrim.HasAPI<AutolibStepTessellationAPI>()) {
                params = getTessParams(targetPrim, params);
            }
        }
    }

    AutolibStepTessellationAPI api(prim);

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

    updateIfAuthored(api.GetStepFixTimeoutAttr(), &params.fixTimeout);
    updateIfAuthored(api.GetStepMeshTimeoutAttr(), &params.meshTimeout);
    updateIfAuthored(api.GetStepRemeshTimeoutAttr(), &params.remeshTimeout);


    return params;
}

std::map<SdfPath, TessParams> resolveParams(
    const UsdPrim& prim, 
    const TessParams& defaultParams
) {
    bool initialActive = prim.IsActive();
    if (prim.HasAuthoredActive() && !initialActive) {
        prim.SetActive(true);
    }

    std::map<SdfPath, TessParams> results;
    
    TessParams primParams = getTessParams(prim, defaultParams);
    
    std::vector<std::string> vsetNames;
    prim.GetVariantSets().GetNames(&vsetNames);

    if (vsetNames.empty()) {
        results[prim.GetPath()] = primParams;
    } else {
        // Since a prim can have multiple variant sets, need a resolution 
        // of all variant selections to fully evaluate every permutation 
        // authored on this prim.
        for (const std::string& name : vsetNames) {
            UsdVariantSet vset = prim.GetVariantSet(name);
            std::string originalSelection = vset.GetVariantSelection();
            std::vector<std::string> variantNames = vset.GetVariantNames();

            for (const std::string& variantName : variantNames) {
                vset.SetVariantSelection(variantName);
                
                // Re-evaluate params in case this variant authos new opinions
                TessParams variantParams = getTessParams(prim, primParams);
                SdfPath variantPath = prim.GetPath().AppendVariantSelection(name, variantName);
                
                results[variantPath] = variantParams;
            }
            
            // Restore original selection
            if (!originalSelection.empty()) {
                vset.SetVariantSelection(originalSelection);
            } else {
                vset.ClearVariantSelection();
            }
        }
    }
    /*
    std::cout << "Params for prim" << prim.GetPath().GetString() << ":\n";
    for (const auto& [path, params] : results) {
        std::cout << "  " << path.GetString() << ": "
                  << "meshLinearDeflection=" << params.meshLinearDeflection << ", "
                  << "meshAngularDeflection=" << params.meshAngularDeflection << ", "
                  << "sketchDefl=" << params.sketchDeflection << "\n";
    }
    */

    if (prim.HasAuthoredActive()) {
        prim.SetActive(initialActive);
    }
    return results;
}

bool UsdStepExporter::isPrototypeActiveInFilter(
    const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
    const SdfPath& rootPrimPath,
    const std::string& variantSetName,
    const std::string& variantName,
    const SdfPath& prototypePath
) {
    if (selectedPaths.empty())
        return true;

    SdfPath basePrototypesKey = rootPrimPath.AppendChild(TfToken("Prototypes"));
    SdfPath prototypesKey = basePrototypesKey;
    if (!variantSetName.empty())
        prototypesKey = prototypesKey.AppendVariantSelection(variantSetName, variantName);

    for (const SdfPath& sel : selectedPaths) {
        // Direct selections of the variant stage or base stage components
        if (sel == prototypesKey || prototypesKey.HasPrefix(sel))
            return true;
        if (sel == basePrototypesKey || basePrototypesKey.HasPrefix(sel))
            return true;
        if (sel == rootPrimPath || rootPrimPath.HasPrefix(sel))
            return true;
    }

    SdfPath absProtoPath = prototypePath.ReplacePrefix(
        SdfPath::AbsoluteRootPath().AppendChild(TfToken("Prototypes")),
        prototypesKey
    );
    SdfPath absProtoPathClean = absProtoPath.StripAllVariantSelections();
    SdfPath protoNoInnerVariant = prototypesKey.AppendChild(prototypePath.GetNameToken());

    for (const SdfPath& sel : selectedPaths) {
        if (sel == absProtoPath || sel.HasPrefix(absProtoPath) || absProtoPath.HasPrefix(sel))
            return true;

        if (sel == absProtoPathClean || sel.HasPrefix(absProtoPathClean) || absProtoPathClean.HasPrefix(sel))
            return true;
            
        if (sel == protoNoInnerVariant || sel.HasPrefix(protoNoInnerVariant) || protoNoInnerVariant.HasPrefix(sel))
            return true;
    }

    return false;
}

std::optional<SdfReference> UsdStepExporter::getPrototypesDefaultParams(const UsdPrim& prototypesPrim) {
    AutolibStepFilePrototypesAPI api(prototypesPrim);

    SdfPathVector targets;
    api.GetStepDefaultParamsRel().GetForwardedTargets(&targets);

    if (targets.empty()) {
        LOG_ERR("No default params target specified on prototypes prim. Using hardcoded defaults.");
        return std::nullopt;
    }

    UsdPrim paramsPrim = prototypesPrim.GetStage()->GetPrimAtPath(targets[0]);

    if (!paramsPrim.IsValid()) {
        LOG_ERR("Default params target " + targets[0].GetString() + " is invalid. Using hardcoded defaults.");
        return std::nullopt;
    }

    if (!paramsPrim.HasAPI<AutolibStepTessellationAPI>()) {
        LOG_ERR("Default params target " + targets[0].GetString() + " does not have AutolibStepTessellationAPI. Using hardcoded defaults.");
        return std::nullopt;
    }

    SdfPrimSpecHandleVector stack = paramsPrim.GetPrimStack();

    if (stack.empty()) {
        LOG_ERR("Default params target " + targets[0].GetString() + " has empty prim stack. Using hardcoded defaults.");
        return std::nullopt;
    }

    SdfReference reference(stack[0]->GetLayer()->GetIdentifier(), stack[0]->GetPath());

    return reference;
}
// Stage Filtering
struct StageFilterInfo {
    bool makeFresh = false; // whole stage targeted
    bool hasSpecificPrototypes = false; // individual prototypes targeted
};

static void resolveStageFilterInfo(
    const SdfPath& rootPrimPath,
    const SdfPath& prototypesInRootPath,
    const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
    std::unordered_map<SdfPath, StageFilterInfo, SdfPath::Hash>& stageFilterMap
) {
    for (const SdfPath& selectedPath : selectedPaths) {
        const SdfPath cleanPath = selectedPath.StripAllVariantSelections();

        // Find the most specific prefix of selectedPath 
        // whose clean form == prototypesInRootPath.
        SdfPath stageKey;
        for (const SdfPath& prefix : selectedPath.GetPrefixes()) {
            if (prefix.StripAllVariantSelections() == prototypesInRootPath)
                stageKey = prefix; // keep updating
        }
        if (selectedPath.StripAllVariantSelections() == prototypesInRootPath)
            stageKey = selectedPath;

        if (stageKey.IsEmpty()) {
            // selectedPath is at or above rootPrimPath
            // Derive the stage key by finding the rootPrimPath-level prefix with its
            // variant selections, then appending /Prototypes.
            SdfPath rootLevelKey;
            for (const SdfPath& prefix : selectedPath.GetPrefixes()) {
                if (prefix.StripAllVariantSelections() == rootPrimPath)
                    rootLevelKey = prefix;
            }
            if (selectedPath.StripAllVariantSelections() == rootPrimPath)
                rootLevelKey = selectedPath;

            if (rootLevelKey.IsEmpty()) continue; // unrelated path

            stageKey = rootLevelKey.AppendChild(TfToken("Prototypes"));
            stageFilterMap[stageKey].makeFresh = true;
            continue;
        }

        StageFilterInfo& info = stageFilterMap[stageKey];
        const bool isSpecificPrototype =
            cleanPath.HasPrefix(prototypesInRootPath) && (cleanPath != prototypesInRootPath);

        if (isSpecificPrototype)
            info.hasSpecificPrototypes = true;
        else
            info.makeFresh = true;
    }

    // Warn when a stage is redundantly targeted 
    // at both levels. The whole-stage wins.
    for (const auto& [stageKey, info] : stageFilterMap) {
        if (info.makeFresh && info.hasSpecificPrototypes) {
            LOG_WARN("SelectedPaths targets both the entire prototype stage and specific "
                    "prototypes within it for stage [" + stageKey.GetAsString() +
                    "]. The whole stage will be rebuilt regardless.");
        }
    }

}

void UsdStepExporter::populateUsd(
    const StepModel& model, 
    UsdStageRefPtr rootStage,
    UsdPrim& rootPrim,
    const std::unordered_set<SdfPath, SdfPath::Hash> selectedPaths
) {
    LOG_SCOPED_TIMER("UsdStepExporter::populateUsd");
    TfErrorMark mark;
    rootStage->Unload();

    fs::path rootFilePath = fs::canonical(rootStage->GetRootLayer()->GetResolvedPath().GetPathString()).remove_filename();
    std::string baseName = model.stepPath.stem().string();

    SdfPath assemblyPath("/Assembly");
    SdfPath prototypesPath("/Prototypes");
    SdfPath rootPrimPath = rootPrim.GetPath();
    SdfPath assemblyInRootPath = rootPrimPath.AppendChild(TfToken("Assembly"));
    SdfPath prototypesInRootPath = rootPrimPath.AppendChild(TfToken("Prototypes"));

    std::unordered_set<SdfPath, SdfPath::Hash> rootVariantPaths;

    UsdPrim protoPrim = rootStage->GetPrimAtPath(prototypesInRootPath);
    if (protoPrim) {
        rootVariantPaths = getVariantsOnPrim(protoPrim);
    }

    auto isRootVariantSelected = [&](const SdfPath& variantPath) -> bool {
        for (const SdfPath& sel : selectedPaths) {
            if (sel == variantPath || sel.HasPrefix(variantPath) || variantPath.HasPrefix(sel))
                return true;

            SdfPath rootPrimPathStr = variantPath.StripAllVariantSelections();
            SdfPath basePrototypesPath = rootPrimPathStr.AppendChild(TfToken("Prototypes"));
            if (sel == basePrototypesPath || sel.HasPrefix(basePrototypesPath) || basePrototypesPath.HasPrefix(sel)) {
                return true;
            }
        }
        return false;
    };

    LOG_INFO("Root prim path: " + rootPrimPath.GetString());
    printVariants("root prim", rootVariantPaths, selectedPaths, [&](const SdfPath& p) {
        return !selectedPaths.empty() && isRootVariantSelected(p);
    });
    printSelectedPrototypes(rootStage, selectedPaths);

    {
        // Clear existing payloads from the root stage beforehand to prevent OpenUSD core crashes
        // and noisy warnings when the prototype layers are modified later on disk.
        SdfChangeBlock block;

        if (selectedPaths.count(prototypesInRootPath)) {
            LOG_WARN("selectedPaths contains /Prototypes root. whole prototypes hierarchy will be rebuilt.");
        }

        if (rootVariantPaths.empty()) {
            if (UsdPrim p = rootStage->GetPrimAtPath(prototypesInRootPath))
                p.GetPayloads().ClearPayloads();
        } else {
            for (const SdfPath& path : rootVariantPaths) {
                if (!selectedPaths.empty() && !isRootVariantSelected(path)) {
                    continue; // only clear payloads that are bing re tesselated
                }
                std::pair<std::string, std::string> variantSelection = path.GetVariantSelection();
                const std::string& variantSetName = variantSelection.first;
                const std::string& variantName = variantSelection.second;

                UsdVariantSet varSet = rootStage->GetPrimAtPath(prototypesInRootPath).GetVariantSet(variantSetName);

                varSet.SetVariantSelection(variantName);
                UsdEditContext ctx(varSet.GetVariantEditContext());
                UsdPrim prototypesPrim = rootStage->OverridePrim(prototypesInRootPath);
                prototypesPrim.GetPayloads().ClearPayloads();
            }
        }
    }

    std::unordered_map<SdfPath, StageFilterInfo, SdfPath::Hash> stageFilterMap;
    resolveStageFilterInfo(rootPrimPath, prototypesInRootPath, selectedPaths, stageFilterMap);

    // Look up whether a given (variantSetName, variantName) stage should be built fresh.
    auto getStageMakeFresh = [&](const std::string& variantSetName, const std::string& variantName) -> bool {
        if (selectedPaths.empty()) return true;

        SdfPath stageKey;
        
        if (variantSetName.empty()) {
            stageKey = prototypesInRootPath;
        } else {
            stageKey = prototypesInRootPath.AppendVariantSelection(variantSetName, variantName);
        }

        auto it = stageFilterMap.find(stageKey);
        if (it == stageFilterMap.end()) return false; // stage not referenced at all
        return it->second.makeFresh;
    };

    // Setup Prototype Stages for all variants
    std::vector<PrototypeContainer> prototypes;

    if (rootVariantPaths.empty()) {
        fs::path prototypesStageFilePath = rootFilePath / (baseName + "-prototypes.usdc");
        bool makeFreshStage = getStageMakeFresh("", "");

        UsdStageRefPtr prototypesStage = UsdStepExporter::initUsdStage(prototypesStageFilePath, rootPrimPath, makeFreshStage);

        UsdPrim existingPrototypesRoot = prototypesStage->GetPrimAtPath(prototypesPath);
        if (existingPrototypesRoot.IsValid() && !existingPrototypesRoot.IsActive())
            existingPrototypesRoot.SetActive(true);

        prototypesStage->SetMetadata(TfToken("metersPerUnit"), model.metersPerUnit);
        AutolibStepFilePrototypes prototypesScope = AutolibStepFilePrototypes::Define(prototypesStage, prototypesPath);
        prototypesStage->SetDefaultPrim(prototypesScope.GetPrim());
        prototypesStage->Save();

        prototypes.push_back({"", "", prototypesStageFilePath, prototypesStage});

    } else {
        baseName += "-";
        for (const SdfPath& variantPath : rootVariantPaths) {
            if (!selectedPaths.empty() && !isRootVariantSelected(variantPath)) {
                continue; // skip variants not in selectedPaths when selectedPaths is non-empty
            }
            std::pair<std::string, std::string> variantSelection = variantPath.GetVariantSelection();
            const std::string& variantSetName = variantSelection.first;
            const std::string& variantName = variantSelection.second;

            fs::path variantSubPath = rootFilePath / variantSetName;
            if (!fs::exists(variantSubPath)) {
                if (!fs::create_directory(variantSubPath)) {
                    std::cerr << "Error: Failed to create directory " << variantSubPath << "\n";
                }
            }

            fs::path prototypesStageFilePath = variantSubPath / (baseName + variantSetName + "-" + variantName + "-prototypes.usdc");
            bool makeFreshStage = getStageMakeFresh(variantSetName, variantName);

            UsdStageRefPtr prototypesStage = UsdStepExporter::initUsdStage(prototypesStageFilePath, rootPrimPath, makeFreshStage);

            UsdPrim existingPrototypesRoot = prototypesStage->GetPrimAtPath(prototypesPath);
            if (existingPrototypesRoot.IsValid() && !existingPrototypesRoot.IsActive())
                existingPrototypesRoot.SetActive(true);

            prototypesStage->SetMetadata(TfToken("metersPerUnit"), model.metersPerUnit);
            AutolibStepFilePrototypes prototypesScope = AutolibStepFilePrototypes::Define(prototypesStage, prototypesPath);
            prototypesStage->SetDefaultPrim(prototypesScope.GetPrim());
            prototypesStage->Save();

            prototypes.push_back({variantSetName, variantName, prototypesStageFilePath, prototypesStage});
        }
    }
    
    // Assembly Stage
    fs::path assemblyStageFilePath = rootFilePath / (model.stepPath.stem().string() + "-assembly.usdc");
    bool shouldCreateAssembly = !fs::exists(assemblyStageFilePath); // don't repopulate the stage if it alread exists 
    UsdStageRefPtr assemblyStage = UsdStepExporter::initUsdStage(assemblyStageFilePath, rootPrimPath, shouldCreateAssembly);
    rootPrim = rootStage->GetPrimAtPath(rootPrimPath);
    
    assemblyStage->SetMetadata(TfToken("metersPerUnit"), model.metersPerUnit);
    assemblyStage->SetDefaultPrim(assemblyStage->OverridePrim(rootPrimPath));
    UsdGeomXform::Define(assemblyStage, assemblyInRootPath);

    SdfLayerHandle rootLayer = rootStage->GetRootLayer();
    std::string assemblyRelativeFilePath = fs::relative(assemblyStageFilePath, rootFilePath).string();
    
    // Write Xforms
    std::vector<std::pair<TDF_Label, TopoDS_Shape>> defs(model.definitionShapes.begin(), model.definitionShapes.end());
    LabelMap<SdfPath> prototypePaths;

    { // Write Prototypes in their own stages
        const UsdPrim& prototypesPrim = rootStage->GetPrimAtPath(prototypesInRootPath);

        for (const auto& proto : prototypes) {
            if (!proto.variantSetName.empty()) {
                UsdVariantSet varSet = rootStage->GetPrimAtPath(prototypesInRootPath).GetVariantSet(proto.variantSetName);
                varSet.SetVariantSelection(proto.variantName);
            }

            writeCadPart(proto.stage, prototypesPrim, SdfPath("/CADPart"));
            bool makeFreshStage = getStageMakeFresh(proto.variantSetName, proto.variantName);

            writePrototypeXformsInPrototypesStage(
                proto.stage, 
                rootPrim, 
                defs, 
                prototypesPath, 
                selectedPaths, 
                rootPrimPath, 
                proto.variantSetName, 
                proto.variantName, 
                prototypePaths, 
                makeFreshStage
            );
            proto.stage->Save();
        }
    }

    writePrototypeOverridesInAssemblyStage(assemblyStage, rootPrim, prototypePaths);
    std::vector<SdfPath> nodePaths = computeNodePaths(model.partNodes, assemblyInRootPath);
    
    if (shouldCreateAssembly) { 
        writeAssemblyXforms(
            assemblyStage, 
            rootPrim.GetPrimPath(), 
            model.partNodes, 
            nodePaths, 
            prototypePaths
        );
        assemblyStage->Save();
    }

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
            UsdVariantSet varSet = rootStage->GetPrimAtPath(prototypesInRootPath).GetVariantSet(proto.variantSetName);
            varSet.SetVariantSelection(proto.variantName);
            UsdEditContext ctx(varSet.GetVariantEditContext());
            UsdPrim prototypesPrim = rootStage->OverridePrim(prototypesInRootPath);
            prototypesPrim.GetPayloads().ClearPayloads();
            prototypesPrim.GetPayloads().AddPayload(SdfPayload(payloadPath, prototypesPath));
        } else {
            UsdPrim prototypesPrim = rootStage->OverridePrim(prototypesInRootPath);
            prototypesPrim.GetPayloads().ClearPayloads();
            prototypesPrim.GetPayloads().AddPayload(SdfPayload(payloadPath, prototypesPath));
        }
    }
    
    rootStage->GetRootLayer()->Save();
    rootStage->Reload();
    rootPrim = rootStage->GetPrimAtPath(rootPrimPath);
    rootStage->Load(rootPrim.GetPath()); // Ensure payloads are loaded for variant discovery!
    rootStage->GetRootLayer()->Save();

    printVariants("root prim", getVariantsOnPrim(rootPrim), selectedPaths, [&](const SdfPath& p) {
        return !selectedPaths.empty() && selectedPaths.count(p);
    });
    // Flatten Tessellation Jobs
    std::vector<TessellationJob> tessJobs;
    for (const auto& proto : prototypes) {

        if (!proto.variantSetName.empty()) {
            UsdVariantSet varSet = rootStage->GetPrimAtPath(prototypesInRootPath).GetVariantSet(proto.variantSetName);
            varSet.SetVariantSelection(proto.variantName);
        }

        // We use rootStage to observe any variant opinions authored on the root file
        UsdPrim protoRootPrim = rootStage->GetPrimAtPath(prototypesInRootPath);

        bool initialValue = protoRootPrim.IsActive();
        protoRootPrim.SetActive(true);

        for (const UsdPrim& child : protoRootPrim.GetChildren()) {
            std::unordered_set<SdfPath, SdfPath::Hash> childVariantPaths = getVariantsOnPrim(child);
            if (!childVariantPaths.empty()) {
                std::string logLabel = child.GetName().GetString();
                if (!proto.variantSetName.empty()) {
                    logLabel += " (root: " + proto.variantSetName + "=" + proto.variantName + ")";
                }

                printVariants(logLabel, childVariantPaths, selectedPaths, [&](const SdfPath& p) {
                    return !selectedPaths.empty() && isPrototypeActiveInFilter(selectedPaths, rootPrimPath, proto.variantSetName, proto.variantName, p);
                });
            }
        }

        protoRootPrim.SetActive(initialValue);

        TessParams rootParams = getTessParams(rootPrim);
        TessParams variantLevelParams = getTessParams(protoRootPrim, rootParams);
        
        std::map<SdfPath, TessParams> paramsBank;
        bool initialActive = protoRootPrim.IsActive();
        if (protoRootPrim.HasAuthoredActive() && !initialActive) {
            protoRootPrim.SetActive(true);
        }
        for (const UsdPrim& child : protoRootPrim.GetChildren()) {
            std::map<SdfPath, TessParams> childParams = resolveParams(child, variantLevelParams);
            paramsBank.insert(childParams.begin(), childParams.end());
        }
        if (protoRootPrim.HasAuthoredActive()) {
            protoRootPrim.SetActive(initialActive);
        }

        bool runMesherInParallel = !getStageMakeFresh(proto.variantSetName, proto.variantName);

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
                    
                    // We need the prototypePath to include the variant
                    // selections so writer knows where to author
                    SdfPath jobProtoPath = protoPath;
                    auto variantSelection = kv.first.GetVariantSelection();
                    if (!variantSelection.first.empty()) {
                        jobProtoPath = jobProtoPath.AppendVariantSelection(variantSelection.first, variantSelection.second);
                    }
                    
                    //std::cout << "DEBUG: Queueing job for " << jobProtoPath.GetString() << " (defIndex " << i << ")\n";
                    tessJobs.push_back({&proto, (int)i, jobProtoPath, params, TessResult(), runMesherInParallel});
                }
            }
            
            if (!foundVariantForProto) {
                tessJobs.push_back({&proto, (int)i, protoPath, params, TessResult(), runMesherInParallel});
            }
        }
    }

    // Tessellation
    TessParams rootParams = getTessParams(rootPrim);
    
    tessellateGeometry(tessJobs, defs, selectedPaths, rootPrimPath);

    // Write Geometry
    LOG_DEBUG("Preparing to gather geometry jobs...");
    LOG_DEBUG("Starting geometry writing for " + std::to_string(prototypes.size()) + " prototypes.");
    for (const auto& proto : prototypes) {
        LOG_DEBUG("Processing prototype stage: " + proto.stage->GetRootLayer()->GetIdentifier() + " (variant: " + proto.variantSetName + "=" + proto.variantName + ")");
        std::vector<ProtoGeomJob> geomJobs;
        for (const auto& job : tessJobs) {
            if (job.proto == &proto) {
                geomJobs.push_back({job.prototypePath, job.result, job.params});
            }
        }
        
        LOG_DEBUG("Writing " + std::to_string(geomJobs.size()) + " geometry jobs to prototype stage.");
        writePrototypeGeometries(proto.stage, geomJobs, selectedPaths, rootPrimPath, proto.variantSetName, proto.variantName);
        LOG_DEBUG("Saving prototype stage.");
        proto.stage->Save();
    }

    LOG_DEBUG("Deactivating original prototype root in root stage: " + prototypesInRootPath.GetString());
    UsdPrim prototypeRoot = rootStage->GetPrimAtPath(prototypesInRootPath);
    if (prototypeRoot.IsValid()) {
        prototypeRoot.SetActive(false);
        rootStage->GetRootLayer()->Save();
        LOG_DEBUG("Prototype root deactivated and root layer saved.");
    }

    if (!mark.IsClean()) {
        for (const auto& error : mark) std::cerr << "Usd: " << error.GetCommentary() << "\n";
    }
}