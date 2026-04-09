#include <iostream>
#include <optional>
#include <string>
#include <filesystem>
#include <cmath>
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
#include <pxr/usd/usd/primRange.h>

#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdGeom/scope.h>
#include <pxr/usd/usdGeom/imageable.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdGeom/metrics.h>

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
#include "stepFilePrototypesAPI.h"
#include "stepFileContainer.h"
#include "stepFilePrototypes.h"

#include "UsdStepExporter.h"
#include "StepModel.h"
#include "Logger.h"

PXR_NAMESPACE_USING_DIRECTIVE

static bool stageNeedsUnitReset(const fs::path& stagePath, double expectedMetersPerUnit) {
    if (!fs::exists(stagePath)) return false;

    UsdStageRefPtr stage = UsdStage::Open(stagePath.string(), UsdStage::LoadNone);
    if (!stage) {
        LOG_WARN("Failed to open existing stage for unit check: " + stagePath.string() + ". Forcing rebuild.");
        return true;
    }

    if (!stage->HasAuthoredMetadata(TfToken("metersPerUnit"))) {
        return true;
    }

    double stageMetersPerUnit = 0.0;
    if (!stage->GetMetadata(TfToken("metersPerUnit"), &stageMetersPerUnit)) {
        return true;
    }

    constexpr double kUnitTolerance = 1e-12;
    return std::abs(stageMetersPerUnit - expectedMetersPerUnit) > kUnitTolerance;
}

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
    
    LOG_DEBUG("Params for prim" + prim.GetPath().GetString() + ":");
    for (const auto& [path, params] : results) {
        LOG_DEBUG("  " + path.GetString() + ": "
                + "meshLinearDeflection=" + std::to_string(params.meshLinearDeflection) + ", "
                + "meshAngularDeflection=" + std::to_string(params.meshAngularDeflection) + ", "
                + "sketchDefl=" + std::to_string(params.sketchDeflection));
    }
    

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

    // We want the path *in the container stage* because WritePrims will pair this 
    // with a relative path pointing back to the container stage.
    SdfReference reference("", targets[0]);

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
        const bool isSpecificPrototype = (cleanPath.HasPrefix(prototypesInContainerPath) && (cleanPath != prototypesInContainerPath)); // Protect variant specific subsets here

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

std::optional<UsdStepExporter> UsdStepExporter::create(
    const fs::path& inputUsdFile
) {
    UsdStageRefPtr stage = UsdStage::Open(inputUsdFile, UsdStage::LoadNone);
    if (!stage) {
        LOG_ERR("Failed to create stage at " + inputUsdFile.string());
        return std::nullopt;
    }

    std::unordered_set<SdfAssetPath, SdfAssetPath::Hash> referencedStepAssetPaths;

    // Do a scan for all refernced Step Assets, so 
    // we can load them in parallel and cache the 
    // results to avoid redundant parsing of the same STEP file.
    // Helper to extract asset paths from a prim under its current variant context
    auto collectFromPrim = [&](const UsdPrim& prim) {
        if (!prim.HasAPI<AutolibStepFileContainerAPI>()) return;

        AutolibStepFileContainer container(prim);
        UsdAttribute pathAttr = container.GetStepSourceAssetAttr();

        SdfAssetPath sdfAssetPath;
        if (!pathAttr.Get(&sdfAssetPath)) {
            LOG_ERR("Failed to get asset path from UsdAttribute");
            return;
        }

        referencedStepAssetPaths.insert(sdfAssetPath);
    };

    // Recursively iterate all variant combinations for a prim
    std::function<void(const UsdPrim&, const std::vector<std::string>&, int)> collectAllVariants;
    collectAllVariants = [&](
        const UsdPrim& prim,
        const std::vector<std::string>& variantSetNames,
        int depth)
    {
        if (depth == static_cast<int>(variantSetNames.size())) {
            // All variant sets have a selectionm
            collectFromPrim(prim);
            return;
        }

        const std::string& setName = variantSetNames[depth];
        UsdVariantSet variantSet   = prim.GetVariantSets().GetVariantSet(setName);
        const std::string original = variantSet.GetVariantSelection();

        for (const std::string& variantName : variantSet.GetVariantNames()) {
            variantSet.SetVariantSelection(variantName);
            collectAllVariants(prim, variantSetNames, depth + 1);
        }

        // Restore original selection
        if (original.empty()) {
            variantSet.ClearVariantSelection();
        } else {
            variantSet.SetVariantSelection(original);
        }
    };

    for (const auto& prim : stage->TraverseAll()) {
        const std::vector<std::string> variantSetNames =
            prim.GetVariantSets().GetNames();

        if (variantSetNames.empty()) {
            collectFromPrim(prim);
        } else {
            collectAllVariants(prim, variantSetNames, 0);
        }
    }

    std::unordered_map<SdfAssetPath, StepModel, SdfAssetPath::Hash> modelCache;

    {
        LOG_SCOPED_TIMER("Load and Parse STEP Models (" + std::to_string(referencedStepAssetPaths.size()) + " files)");
        WorkParallelForEach( referencedStepAssetPaths.begin(), referencedStepAssetPaths.end(), [&](const SdfAssetPath& assetPath) {
            std::string resolvedPath = assetPath.GetResolvedPath();

            if (resolvedPath.empty()) {
                LOG_ERR("Failed to resolve path to: " + assetPath.GetAssetPath());
                return;
            }

            std::optional<StepModel> optModel = StepModel::loadFromFile(resolvedPath);

            if (!optModel.has_value()) {
                LOG_ERR("Failed to load STEP model from " + resolvedPath);
                return;
            }

            modelCache.insert_or_assign(assetPath, std::move(*optModel));
        });
    }

    return UsdStepExporter(stage, modelCache);
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

    // Generated layers follow the root stage unit if authored.
    // If missing, author a deterministic fallback of 1.0 meters-per-unit on the root.
    constexpr double fallbackMetersPerUnit = 1.0;
    double outputMetersPerUnit = fallbackMetersPerUnit;
    if (UsdGeomStageHasAuthoredMetersPerUnit(containerStage)) {
        outputMetersPerUnit = UsdGeomGetStageMetersPerUnit(containerStage);
    } else {
        UsdGeomSetStageMetersPerUnit(containerStage, fallbackMetersPerUnit);
    }
    const double sourceToOutputScale = model.metersPerUnit / outputMetersPerUnit;

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

        // Keep the pre-clear pass only for full runs. For filtered runs, clear payloads
        // only on variants that are actually rewritten below.
        if (selectedPaths.empty()) {
            SdfChangeBlock block;
            if (containerVariantPaths.empty()) {
                if (UsdPrim p = containerStage->GetPrimAtPath(prototypesInContainerPath))
                    p.GetPayloads().ClearPayloads();
            } else {
                for (const SdfPath& path : containerVariantPaths) {
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
        if (it != stageFilterMap.end()) return it->second.makeFresh; 

        // Add Fallback: Check if the base container path (no variant) is targeted for a full hierarchy refresh
        it = stageFilterMap.find(prototypesInContainerPath);
        if (it != stageFilterMap.end()) return it->second.makeFresh;

        return false;
    };

    // Setup Prototype Stages for all variants
    std::vector<PrototypeContainer> prototypes;

    if (containerVariantPaths.empty()) {
        fs::path prototypesStageFilePath = containerFilePath / (baseName + "-prototypes.usdc");
        bool makeFreshStage = getStageMakeFresh("", "");
        if (stageNeedsUnitReset(prototypesStageFilePath, outputMetersPerUnit)) {
            LOG_INFO("Resetting prototypes stage due to metersPerUnit mismatch: " + prototypesStageFilePath.string());
            makeFreshStage = true;
        }

        UsdStageRefPtr prototypesStage = UsdStepExporter::initUsdStage(prototypesStageFilePath, makeFreshStage);

        UsdPrim existingPrototypesContainer = prototypesStage->GetPrimAtPath(prototypesPath);
        if (existingPrototypesContainer.IsValid() && !existingPrototypesContainer.IsActive())
            existingPrototypesContainer.SetActive(true);

        UsdGeomSetStageMetersPerUnit(prototypesStage, outputMetersPerUnit);
        prototypesStage->SetMetadata(TfToken("metersPerUnit"), outputMetersPerUnit);
        AutolibStepFilePrototypes prototypesScope = AutolibStepFilePrototypes::Define(prototypesStage, prototypesPath);
        prototypesStage->SetDefaultPrim(prototypesScope.GetPrim());
        prototypesStage->Save();

        prototypes.push_back({"", "", prototypesStageFilePath, prototypesStage, makeFreshStage});

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
            if (stageNeedsUnitReset(prototypesStageFilePath, outputMetersPerUnit)) {
                LOG_INFO("Resetting prototypes stage due to metersPerUnit mismatch: " + prototypesStageFilePath.string());
                makeFreshStage = true;
            }

            UsdStageRefPtr prototypesStage = UsdStepExporter::initUsdStage(prototypesStageFilePath, makeFreshStage);

            UsdPrim existingPrototypesContainer = prototypesStage->GetPrimAtPath(prototypesPath);
            if (existingPrototypesContainer.IsValid() && !existingPrototypesContainer.IsActive())
                existingPrototypesContainer.SetActive(true);

            UsdGeomSetStageMetersPerUnit(prototypesStage, outputMetersPerUnit);
            AutolibStepFilePrototypes prototypesScope = AutolibStepFilePrototypes::Define(prototypesStage, prototypesPath);
            prototypesStage->SetDefaultPrim(prototypesScope.GetPrim());
            prototypesStage->Save();

            prototypes.push_back({variantSetName, variantName, prototypesStageFilePath, prototypesStage, makeFreshStage});
        }
    }
    
    // Assembly Stage
    fs::path assemblyStageFilePath = containerFilePath / (model.stepPath.stem().string() + "-assembly.usdc");
    bool shouldCreateAssembly = !fs::exists(assemblyStageFilePath); // don't repopulate the stage if it alread exists 
    if (stageNeedsUnitReset(assemblyStageFilePath, outputMetersPerUnit)) {
        LOG_INFO("Resetting assembly stage due to metersPerUnit mismatch: " + assemblyStageFilePath.string());
        shouldCreateAssembly = true;
    }
    UsdStageRefPtr assemblyStage = UsdStepExporter::initUsdStage(assemblyStageFilePath, shouldCreateAssembly);
    containerPrim = containerStage->GetPrimAtPath(containerPrimPath);
    
    UsdGeomSetStageMetersPerUnit(assemblyStage, outputMetersPerUnit);
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
                proto.makeFreshStage
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
            prototypePaths,
            sourceToOutputScale
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

        bool runMesherInParallel = !proto.makeFreshStage;

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
                    params.unitScale = sourceToOutputScale;
                    
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
                params.unitScale = sourceToOutputScale;
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

static void populate(
    UsdStageRefPtr stage
) {

}