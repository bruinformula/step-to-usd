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

    // Meshing
    updateIfAuthored(api.GetStepMeshLinearDeflectionAttr(), &params.meshLinearDeflection);
    updateIfAuthored(api.GetStepMeshAngularDeflectionAttr(), &params.meshAngularDeflection);
    updateIfAuthored(api.GetStepMeshMinSizeAttr(), &params.meshMinSize);
    updateIfAuthored(api.GetStepSelfIntersectionThresholdAttr(), &params.selfIntersectionThreshold);
    updateIfAuthored(api.GetStepMaxNumberRemeshPassesAttr(), &params.maxNumberRemeshPasses);

    updateIfAuthored(api.GetStepFixTimeoutAttr(), &params.fixTimeout);
    updateIfAuthored(api.GetStepMeshTimeoutAttr(), &params.meshTimeout);
    updateIfAuthored(api.GetStepRemeshTimeoutAttr(), &params.remeshTimeout);

    // Wireframe
    updateIfAuthored(api.GetStepWireframeCombineCurvesAttr(), &params.wireframeCombineCurves);
    updateIfAuthored(api.GetStepWireframeDeflectionAttr(), &params.wireframeDeflection);
    updateIfAuthored(api.GetStepWireframeTypeAttr(), &params.wireframeMode.type);
    updateIfAuthored(api.GetStepWireframeSamplingAttr(), &params.wireframeMode.sampling);

    // Sketch
    updateIfAuthored(api.GetStepSketchCombineCurvesAttr(), &params.sketchCombineCurves);
    updateIfAuthored(api.GetStepSketchDeflectionAttr(), &params.sketchDeflection);
    updateIfAuthored(api.GetStepSketchTypeAttr(), &params.sketchMode.type);
    updateIfAuthored(api.GetStepSketchSamplingAttr(), &params.sketchMode.sampling);

    // Other
    updateIfAuthored(api.GetStepRenderPurposeThresholdAttr(), &params.renderPurposeThreshold);
    updateIfAuthored(api.GetStepEnableSurfaceSubsetsAttr(), &params.enableSurfaceSubsets);
    updateIfAuthored(api.GetStepEnableUVsAttr(), &params.enableUVs);
    updateIfAuthored(api.GetStepEnableSurfaceIDAttr(), &params.enableSurfaceID);
    updateIfAuthored(api.GetStepEnableIsBoundaryVertexAttr(), &params.enableIsBoundaryVertex);

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
    const SdfPath& containerPrimPath,
    const std::string& variantSetName,
    const std::string& variantName,
    const SdfPath& prototypePath
) {
    if (selectedPaths.empty())
        return true;

    // Resolve the fully qualified prototypes path for the current container variant permutation
    SdfPath basePrototypesKey = containerPrimPath.AppendChild(TfToken("Prototypes"));
    SdfPath prototypesKey = variantSetName.empty() ? basePrototypesKey 
                          : basePrototypesKey.AppendVariantSelection(variantSetName, variantName);

    // Project prototypePath onto this container
    SdfPath absoluteProtoPath;
    if (prototypePath.HasPrefix(basePrototypesKey)) {
        absoluteProtoPath = prototypePath.ReplacePrefix(basePrototypesKey, prototypesKey);
    } else {
        absoluteProtoPath = prototypePath.ReplacePrefix(
            SdfPath::AbsoluteRootPath().AppendChild(TfToken("Prototypes")),
            prototypesKey
        );
    }

    // A path is active if it shares a common structural prefix with a selected path
    // and all explicit variant selections present in both paths strictly match.
    for (const SdfPath& sel : selectedPaths) {
        SdfPath cleanSel = sel.StripAllVariantSelections();
        SdfPath cleanAbs = absoluteProtoPath.StripAllVariantSelections();
        
        // They must be topologically related
        if (!cleanSel.HasPrefix(cleanAbs) && !cleanAbs.HasPrefix(cleanSel)) continue;

        // Check for any conflicting variant selections along the paths
        bool hasConflictingVariant = false;
        
        std::map<SdfPath, std::pair<std::string, std::string>> selVariants;
        for (const SdfPath& p : sel.GetPrefixes()) {
            if (p.IsPrimVariantSelectionPath()) {
                selVariants[p.StripAllVariantSelections()] = p.GetVariantSelection();
            }
        }
        
        std::map<SdfPath, std::pair<std::string, std::string>> absVariants;
        for (const SdfPath& p : absoluteProtoPath.GetPrefixes()) {
            if (p.IsPrimVariantSelectionPath()) {
                absVariants[p.StripAllVariantSelections()] = p.GetVariantSelection();
            }
        }

        for (const auto& [cleanPath, varPair] : selVariants) {
            auto it = absVariants.find(cleanPath);
            if (it != absVariants.end()) {
                if (it->second != varPair) {
                    hasConflictingVariant = true;
                    break;
                }
            }
        }

        if (!hasConflictingVariant) return true;
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
    const SdfPath& containerPrimPath,
    const SdfPath& prototypesInContainerPath,
    const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
    std::unordered_map<SdfPath, StageFilterInfo, SdfPath::Hash>& stageFilterMap
) {
    for (const SdfPath& selectedPath : selectedPaths) {
        const SdfPath cleanPath = selectedPath.StripAllVariantSelections();

        // Find the most specific prefix of selectedPath 
        // whose clean form == prototypesInContainerPath.
        SdfPath stageKey;
        for (const SdfPath& prefix : selectedPath.GetPrefixes()) {
            if (prefix.StripAllVariantSelections() == prototypesInContainerPath)
                stageKey = prefix; // keep updating
        }
        if (selectedPath.StripAllVariantSelections() == prototypesInContainerPath)
            stageKey = selectedPath;

        if (stageKey.IsEmpty()) {
            // selectedPath is at or above containerPrimPath
            // Derive the stage key by finding the containerPrimPath-level prefix with its
            // variant selections, then appending /Prototypes.
            SdfPath containerLevelKey;
            for (const SdfPath& prefix : selectedPath.GetPrefixes()) {
                if (prefix.StripAllVariantSelections() == containerPrimPath)
                    containerLevelKey = prefix;
            }
            if (selectedPath.StripAllVariantSelections() == containerPrimPath)
                containerLevelKey = selectedPath;

            if (containerLevelKey.IsEmpty()) continue; // unrelated path

            stageKey = containerLevelKey.AppendChild(TfToken("Prototypes"));
            stageFilterMap[stageKey].makeFresh = true;
            continue;
        }

        StageFilterInfo& info = stageFilterMap[stageKey];
        const bool isSpecificPrototype =
            cleanPath.HasPrefix(prototypesInContainerPath) && (cleanPath != prototypesInContainerPath);

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

bool UsdStepExporter::validateVariants(
    UsdStageRefPtr containerStage,
    const SdfPath& containerPrimPath,
    const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths
) {
    LOG_INFO("Container prim path: " + containerPrimPath.GetString());
    containerStage->Load(containerPrimPath);

    for (const SdfPath& sel : selectedPaths) {
        if (!sel.ContainsPrimVariantSelection()) continue;
        
        SdfPath currentPath = SdfPath::AbsoluteRootPath();
        for (const SdfPath& prefix : sel.GetPrefixes()) {
            if (prefix.ContainsPrimVariantSelection() && prefix != currentPath) {
                std::pair<std::string, std::string> varSel = prefix.GetVariantSelection();
                if (!varSel.first.empty()) {
                    SdfPath primPath = prefix.StripAllVariantSelections();
                    UsdPrim prim = containerStage->GetPrimAtPath(primPath);
                    if (prim) {
                        if (!prim.HasVariantSets()) {
                            LOG_ERR("Selected path " + sel.GetString() + " contains variant selection {" + varSel.first + "=" + varSel.second + "} but prim " + primPath.GetString() + " has no variant sets.");
                            return false;
                        }
                        UsdVariantSet varSet = prim.GetVariantSet(varSel.first);
                        if (!varSet.IsValid()) {
                            LOG_ERR("Invalid variant set in selected path: {" + varSel.first + "=" + varSel.second + "} on prim " + primPath.GetString());
                            return false;
                        }
                        std::vector<std::string> varNames = varSet.GetVariantNames();
                        if (!varNames.empty() && std::find(varNames.begin(), varNames.end(), varSel.second) == varNames.end()) {
                            LOG_ERR("Invalid variant selection: {" + varSel.first + "=" + varSel.second + "} on prim " + primPath.GetString());
                            return false;
                        }
                    }
                }
            }
            currentPath = prefix;
        }
    }

    return true;
}

void UsdStepExporter::populateUsd(
    const StepModel& model, 
    UsdStageRefPtr containerStage,
    UsdPrim& containerPrim,
    const std::unordered_set<SdfPath, SdfPath::Hash> selectedPaths
) {
    LOG_SCOPED_TIMER("UsdStepExporter::populateUsd");
    TfErrorMark mark;

    if (!validateVariants(containerStage, containerPrim.GetPath(), selectedPaths)) {
        return;
    }

    containerStage->Unload();

    fs::path containerFilePath = fs::canonical(containerStage->GetRootLayer()->GetResolvedPath().GetPathString()).remove_filename();
    std::string baseName = model.stepPath.stem().string();

    SdfPath assemblyPath("/Assembly");
    SdfPath prototypesPath("/Prototypes");
    SdfPath containerPrimPath = containerPrim.GetPath();
    SdfPath assemblyInContainerPath = containerPrimPath.AppendChild(TfToken("Assembly"));
    SdfPath prototypesInContainerPath = containerPrimPath.AppendChild(TfToken("Prototypes"));

    std::unordered_set<SdfPath, SdfPath::Hash> containerVariantPaths;

    UsdPrim protoPrim = containerStage->GetPrimAtPath(prototypesInContainerPath);
    if (protoPrim) {
        containerVariantPaths = getVariantsOnPrim(protoPrim);
    }

    auto isContainerVariantSelected = [&](const SdfPath& variantPath) -> bool {
        if (selectedPaths.empty()) return true;
        for (const SdfPath& sel : selectedPaths) {
            if (sel == variantPath || sel.HasPrefix(variantPath) || variantPath.HasPrefix(sel))
                return true;

            SdfPath containerPrimPathStr = variantPath.StripAllVariantSelections();
            bool hasConflictingVariant = false;
            for (const SdfPath& prefix : sel.GetPrefixes()) {
                if (prefix.ContainsPrimVariantSelection() && prefix.StripAllVariantSelections() == containerPrimPathStr) {
                    if (prefix != variantPath) {
                        hasConflictingVariant = true;
                        break;
                    }
                }
            }
            if (hasConflictingVariant) continue;

            if (sel == containerPrimPathStr || sel.HasPrefix(containerPrimPathStr) || containerPrimPathStr.HasPrefix(sel)) {
                return true;
            }
        }
        return false;
    };

    {
        // Clear existing payloads from the container stage beforehand to prevent OpenUSD core crashes
        // and noisy warnings when the prototype layers are modified later on disk.
        SdfChangeBlock block;

        bool hasSpecificPrototype = false;
        bool hasContainerRoot = false;
        for (const SdfPath& sel : selectedPaths) {
            SdfPath cleanPath = sel.StripAllVariantSelections();
            if (cleanPath == prototypesInContainerPath) {
                hasContainerRoot = true;
            } else if (cleanPath.HasPrefix(prototypesInContainerPath) && cleanPath != prototypesInContainerPath) {
                hasSpecificPrototype = true;
            }
        }

        if (hasContainerRoot && hasSpecificPrototype) {
            LOG_WARN("selectedPaths contains /Prototypes container. whole prototypes hierarchy will be rebuilt.");
        }

        if (containerVariantPaths.empty()) {
            if (UsdPrim p = containerStage->GetPrimAtPath(prototypesInContainerPath))
                p.GetPayloads().ClearPayloads();
        } else {
            for (const SdfPath& path : containerVariantPaths) {
                if (!selectedPaths.empty() && !isContainerVariantSelected(path)) {
                    continue; // only clear payloads that are bing re tesselated
                }
                std::pair<std::string, std::string> variantSelection = path.GetVariantSelection();
                const std::string& variantSetName = variantSelection.first;
                const std::string& variantName = variantSelection.second;

                UsdVariantSet varSet = containerStage->GetPrimAtPath(prototypesInContainerPath).GetVariantSet(variantSetName);

                varSet.SetVariantSelection(variantName);
                UsdEditContext ctx(varSet.GetVariantEditContext());
                UsdPrim prototypesPrim = containerStage->OverridePrim(prototypesInContainerPath);
                prototypesPrim.GetPayloads().ClearPayloads();
            }
        }
    }

    std::unordered_map<SdfPath, StageFilterInfo, SdfPath::Hash> stageFilterMap;
    resolveStageFilterInfo(containerPrimPath, prototypesInContainerPath, selectedPaths, stageFilterMap);

    // Look up whether a given (variantSetName, variantName) stage should be built fresh.
    auto getStageMakeFresh = [&](const std::string& variantSetName, const std::string& variantName) -> bool {
        if (selectedPaths.empty()) return true;

        SdfPath stageKey;
        
        if (variantSetName.empty()) {
            stageKey = prototypesInContainerPath;
        } else {
            stageKey = prototypesInContainerPath.AppendVariantSelection(variantSetName, variantName);
        }

        auto it = stageFilterMap.find(stageKey);
        if (it == stageFilterMap.end()) return false; // stage not referenced at all
        return it->second.makeFresh;
    };

    // Setup Prototype Stages for all variants
    std::vector<PrototypeContainer> prototypes;

    if (containerVariantPaths.empty()) {
        fs::path prototypesStageFilePath = containerFilePath / (baseName + "-prototypes.usdc");
        bool makeFreshStage = getStageMakeFresh("", "");

        UsdStageRefPtr prototypesStage = UsdStepExporter::initUsdStage(prototypesStageFilePath, makeFreshStage);

        UsdPrim existingPrototypesContainer = prototypesStage->GetPrimAtPath(prototypesPath);
        if (existingPrototypesContainer.IsValid() && !existingPrototypesContainer.IsActive())
            existingPrototypesContainer.SetActive(true);

        prototypesStage->SetMetadata(TfToken("metersPerUnit"), model.metersPerUnit);
        AutolibStepFilePrototypes prototypesScope = AutolibStepFilePrototypes::Define(prototypesStage, prototypesPath);
        prototypesStage->SetDefaultPrim(prototypesScope.GetPrim());
        prototypesStage->Save();

        prototypes.push_back({"", "", prototypesStageFilePath, prototypesStage});

    } else {
        baseName += "-";
        for (const SdfPath& variantPath : containerVariantPaths) {
            if (!selectedPaths.empty() && !isContainerVariantSelected(variantPath)) {
                continue; // skip variants not in selectedPaths when selectedPaths is non-empty
            }
            std::pair<std::string, std::string> variantSelection = variantPath.GetVariantSelection();
            const std::string& variantSetName = variantSelection.first;
            const std::string& variantName = variantSelection.second;

            fs::path variantSubPath = containerFilePath / variantSetName;
            if (!fs::exists(variantSubPath)) {
                if (!fs::create_directory(variantSubPath)) {
                    std::cerr << "Error: Failed to create directory " << variantSubPath << "\n";
                }
            }

            fs::path prototypesStageFilePath = variantSubPath / (baseName + variantSetName + "-" + variantName + "-prototypes.usdc");
            bool makeFreshStage = getStageMakeFresh(variantSetName, variantName);

            UsdStageRefPtr prototypesStage = UsdStepExporter::initUsdStage(prototypesStageFilePath, makeFreshStage);

            UsdPrim existingPrototypesContainer = prototypesStage->GetPrimAtPath(prototypesPath);
            if (existingPrototypesContainer.IsValid() && !existingPrototypesContainer.IsActive())
                existingPrototypesContainer.SetActive(true);

            prototypesStage->SetMetadata(TfToken("metersPerUnit"), model.metersPerUnit);
            AutolibStepFilePrototypes prototypesScope = AutolibStepFilePrototypes::Define(prototypesStage, prototypesPath);
            prototypesStage->SetDefaultPrim(prototypesScope.GetPrim());
            prototypesStage->Save();

            prototypes.push_back({variantSetName, variantName, prototypesStageFilePath, prototypesStage});
        }
    }
    
    // Assembly Stage
    fs::path assemblyStageFilePath = containerFilePath / (model.stepPath.stem().string() + "-assembly.usdc");
    bool shouldCreateAssembly = !fs::exists(assemblyStageFilePath); // don't repopulate the stage if it alread exists 
    UsdStageRefPtr assemblyStage = UsdStepExporter::initUsdStage(assemblyStageFilePath, shouldCreateAssembly);
    containerPrim = containerStage->GetPrimAtPath(containerPrimPath);
    
    assemblyStage->SetMetadata(TfToken("metersPerUnit"), model.metersPerUnit);
    assemblyStage->SetDefaultPrim(assemblyStage->OverridePrim(containerPrimPath));
    UsdGeomXform::Define(assemblyStage, assemblyInContainerPath);

    SdfLayerHandle containerLayer = containerStage->GetRootLayer();
    std::string assemblyRelativeFilePath = fs::relative(assemblyStageFilePath, containerFilePath).string();
    
    // Write Xforms
    std::vector<std::pair<TDF_Label, TopoDS_Shape>> defs(model.definitionShapes.begin(), model.definitionShapes.end());
    LabelMap<SdfPath> prototypePaths;

    { // Write prototypes and assembly references in their own stages
        const UsdPrim& prototypesPrim = containerStage->GetPrimAtPath(prototypesInContainerPath);

        for (const auto& proto : prototypes) {
            if (!proto.variantSetName.empty()) {
                UsdVariantSet varSet = containerStage->GetPrimAtPath(prototypesInContainerPath).GetVariantSet(proto.variantSetName);
                varSet.SetVariantSelection(proto.variantName);
            }

            writeCadPart(proto.stage, prototypesPrim, SdfPath("/CADPart"));
            bool makeFreshStage = getStageMakeFresh(proto.variantSetName, proto.variantName);

            writePrototypeXformsInPrototypesStage(
                proto.stage, 
                containerPrim, 
                defs, 
                prototypesPath, 
                selectedPaths, 
                containerPrimPath, 
                proto.variantSetName, 
                proto.variantName, 
                prototypePaths, 
                makeFreshStage
            );
            proto.stage->Save();
        }
    }

    writePrototypeOverridesInAssemblyStage(assemblyStage, containerPrim, prototypePaths);
    std::vector<SdfPath> nodePaths = computeNodePaths(model.partNodes, assemblyInContainerPath);
    
    if (shouldCreateAssembly) { 
        writeAssemblyXforms(
            assemblyStage, 
            containerPrim.GetPrimPath(), 
            model.partNodes, 
            nodePaths, 
            prototypePaths
        );
        assemblyStage->Save();
    }

    bool alreadyExists = false;
    for (const auto& path : containerLayer->GetSubLayerPaths()) {
        if (path == assemblyRelativeFilePath) { alreadyExists = true; break; }
    }

    if (!alreadyExists) containerLayer->InsertSubLayerPath(assemblyRelativeFilePath);
    
    // Payload logic on container stage
    for (const auto& proto : prototypes) {
        std::string payloadPath = fs::relative(proto.filePath, containerFilePath).string();
        
        if (!proto.variantSetName.empty()) {
            UsdVariantSet varSet = containerStage->GetPrimAtPath(prototypesInContainerPath).GetVariantSet(proto.variantSetName);
            varSet.SetVariantSelection(proto.variantName);
            UsdEditContext ctx(varSet.GetVariantEditContext());
            UsdPrim prototypesPrim = containerStage->OverridePrim(prototypesInContainerPath);
            prototypesPrim.GetPayloads().ClearPayloads();
            prototypesPrim.GetPayloads().AddPayload(SdfPayload(payloadPath, prototypesPath));
        } else {
            UsdPrim prototypesPrim = containerStage->OverridePrim(prototypesInContainerPath);
            prototypesPrim.GetPayloads().ClearPayloads();
            prototypesPrim.GetPayloads().AddPayload(SdfPayload(payloadPath, prototypesPath));
        }
    }
    
    containerStage->GetRootLayer()->Save();
    containerStage->Reload();
    containerPrim = containerStage->GetPrimAtPath(containerPrimPath);
    containerStage->Load(containerPrim.GetPath()); // Ensure payloads are loaded for variant discovery!
    containerStage->GetRootLayer()->Save();

    // Flatten Tessellation Jobs
    std::vector<TessellationJob> tessJobs;
    for (const auto& proto : prototypes) {

        if (!proto.variantSetName.empty()) {
            UsdVariantSet varSet = containerStage->GetPrimAtPath(prototypesInContainerPath).GetVariantSet(proto.variantSetName);
            varSet.SetVariantSelection(proto.variantName);
        }

        // We use containerStage to observe any variant opinions authored on the container file
        UsdPrim protocontainerPrim = containerStage->GetPrimAtPath(prototypesInContainerPath);

        TessParams containerParams = getTessParams(containerPrim);
        TessParams variantLevelParams = getTessParams(protocontainerPrim, containerParams);
        
        std::map<SdfPath, TessParams> paramsBank;
        bool initialActive = protocontainerPrim.IsActive();
        if (protocontainerPrim.HasAuthoredActive() && !initialActive) {
            protocontainerPrim.SetActive(true);
        }
        for (const UsdPrim& child : protocontainerPrim.GetChildren()) {
            std::map<SdfPath, TessParams> childParams = resolveParams(child, variantLevelParams);
            paramsBank.insert(childParams.begin(), childParams.end());
        }
        if (protocontainerPrim.HasAuthoredActive()) {
            protocontainerPrim.SetActive(initialActive);
        }

        bool runMesherInParallel = !getStageMakeFresh(proto.variantSetName, proto.variantName);

        for (size_t i = 0; i < defs.size(); ++i) {
            SdfPath protoPath = prototypePaths.at(defs[i].first); // /Prototypes/rod0

            SdfPath paramKeyPath = prototypesInContainerPath.AppendChild(protoPath.GetNameToken());            
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
    TessParams containerParams = getTessParams(containerPrim);
    
    tessellateGeometry(tessJobs, defs, selectedPaths, containerPrimPath);

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
        writePrototypeGeometries(proto.stage, geomJobs, selectedPaths, containerPrimPath, proto.variantSetName, proto.variantName);
        LOG_DEBUG("Saving prototype stage.");
        proto.stage->Save();
    }

    LOG_DEBUG("Deactivating original prototype container in container stage: " + prototypesInContainerPath.GetString());
    UsdPrim prototypeContainer = containerStage->GetPrimAtPath(prototypesInContainerPath);
    if (prototypeContainer.IsValid()) {
        prototypeContainer.SetActive(false);
        containerStage->GetRootLayer()->Save();
        LOG_DEBUG("Prototype container deactivated and container layer saved.");
    }

    if (!mark.IsClean()) {
        for (const auto& error : mark) std::cerr << "Usd: " << error.GetCommentary() << "\n";
    }
}