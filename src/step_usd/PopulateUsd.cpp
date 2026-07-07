#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <filesystem>
#include <cmath>
#include <map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <TDF_Label.hxx>
#include <TopoDS_Shape.hxx>

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
#include <pxr/usd/sdf/layerUtils.h> 
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/payload.h>
#include <pxr/usd/sdf/primSpec.h>
#include <pxr/usd/sdf/proxyTypes.h>
#include <pxr/usd/sdf/reference.h>

#pragma pop_macro("Handle")

#include "stepTessellationAPI.h"
#include "stepContainerAPI.h"
#include "stepPrototypesAPI.h"
#include "stepContainer.h"
#include "stepPrototypes.h"

#include "StepUsdPipeline.h"
#include "OpenCascadeAssembly.h"
#include "Logger.h"
#include "UsdUtils.h"

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

bool StepUsdPipeline::isPrototypeActiveInFilter(
    const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
    const SdfPath& prototypePath,
    const std::string& variantSetName,
    const std::string& variantName
) {
    if (selectedPaths.empty())
        return true;

    // Resolve the fully qualified prototypes path for the current container variant permutation
    SdfPath basePrototypesKey = this->pathConfig.containerPrimPath.AppendChild(TfToken("Prototypes"));
    SdfPath prototypesKey;

    if (variantSetName.empty()) {
        prototypesKey = basePrototypesKey;
    } else {
        prototypesKey = basePrototypesKey.AppendVariantSelection(variantSetName, variantName);
    }

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

bool StepUsdPipeline::isAssemblyActiveInFilter(
    const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
    const SdfPath& prototypePath
) {
    if (selectedPaths.empty()) return true;

    // remove all {Variant=Selection}
    SdfPath cleanProto = prototypePath.StripAllVariantSelections();
    SdfPath cleanContainer = this->pathConfig.containerPrimPath.StripAllVariantSelections();
    SdfPath assemblyBase = cleanContainer.AppendChild(TfToken("Assembly"));

    if (!cleanProto.HasPrefix(assemblyBase)) {
        return false;
    }
    // Check against selections
    for (const SdfPath& sel : selectedPaths) {
        SdfPath cleanSel = sel.StripAllVariantSelections();
        // Selection is a parent: /Container or /Container/Assembly
        // Selection is the prim or a child: /Container/Assembly/rod_1 or /Container/Assembly/rod_1/screw
        if (cleanProto.HasPrefix(cleanSel) || cleanSel.HasPrefix(cleanProto)) {
            return true;
        }
    }

    return false;
}

void resolveStageFilterInfo(
    const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
    const SdfPath& containerPrimPath,
    const SdfPath& prototypesPath,
    std::unordered_map<SdfPath, StageFilterInfo, SdfPath::Hash>& stageFilterMap
) {
    for (const SdfPath& selectedPath : selectedPaths) {
        const SdfPath cleanPath = selectedPath.StripAllVariantSelections();

        // Find the most specific prefix of selectedPath 
        // whose clean form == prototypesPath.
        SdfPath stageKey;
        for (const SdfPath& prefix : selectedPath.GetPrefixes()) {
            if (prefix.StripAllVariantSelections() == prototypesPath)
                stageKey = prefix; // keep updating
        }
        if (selectedPath.StripAllVariantSelections() == prototypesPath)
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
        const bool isSpecificPrototype = (cleanPath.HasPrefix(prototypesPath) && (cleanPath != prototypesPath)); // Protect variant specific subsets here

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

// Look up whether a given (variantSetName, variantName) stage should be built fresh.
bool getStageMakeFresh(
    const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
    const SdfPath& containerPrimPath,
    const SdfPath& prototypesPath,
    const std::string& variantSetName, 
    const std::string& variantName,
    const std::unordered_map<SdfPath, StageFilterInfo, SdfPath::Hash>& stageFilterMap
) {
    if (selectedPaths.empty()) return true;

    SdfPath stageKey;
    if (variantSetName.empty()) {
        stageKey = prototypesPath;
    } else {
        stageKey = containerPrimPath.AppendVariantSelection(variantSetName, variantName).AppendChild(TfToken("Prototypes"));
    }

    auto it = stageFilterMap.find(stageKey);
    if (it != stageFilterMap.end()) return it->second.makeFresh;

    // Fallback: if /Prototypes (without container variant) is selected, refresh all variant stages.
    it = stageFilterMap.find(prototypesPath);
    if (it != stageFilterMap.end()) return it->second.makeFresh;

    return false;
};

bool StepUsdPipeline::populatePrototypeContainers(
    const UsdPrim& containerPrim,
    const UsdStageRefPtr& containerStage,
    const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
    fs::path rootPath,
    std::string baseName,
    const std::unordered_set<SdfPath, SdfPath::Hash>& containerVariantPaths,
    double outputMetersPerUnit,
    const std::unordered_map<SdfPath, StageFilterInfo, SdfPath::Hash> stageFilterMap,
    std::vector<PrototypeContainer>& prototypes
) {

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

    AutolibStepContainer container(containerPrim);

    auto getOpenCascadeAssembly = [&](void) -> std::shared_ptr<OpenCascadeAssembly> {
        if (!container.GetStepSourceAssetAttr().HasAuthoredValue()) return nullptr;
        UsdAttribute pathAttr = container.GetStepSourceAssetAttr();
        
        SdfAssetPath sdfAssetPath;
        if (!pathAttr.Get(&sdfAssetPath)) {
            LOG_ERR("Failed to get asset path from UsdAttribute");
            return nullptr;
        }

        fs::path assetPath = sdfAssetPath.GetResolvedPath();

        // Load the model, using the cache to avoid re-parsing the same STEP file.
        auto iter = modelCache.find(sdfAssetPath);
        if (iter == modelCache.end()) {
            LOG_ERR("Model not found in cache for asset path: " + assetPath.string());
            return nullptr;
        }
        return std::make_shared<OpenCascadeAssembly>(iter->second);
    };

    auto getPrototypesStagePath = [&](void) -> std::optional<fs::path> {
        UsdAttribute pathAttr = container.GetStepOutputAssetAttr();
        if (!pathAttr.HasAuthoredValue()) {
            return std::nullopt;
        }

        SdfAssetPath sdfAssetPath;
        if (!pathAttr.Get(&sdfAssetPath)) {
            LOG_WARN("Failed to get asset path from UsdAttribute");
            return std::nullopt;
        }

        const std::string& rawPath = sdfAssetPath.GetAssetPath();
        if (rawPath.empty()) {
            LOG_WARN("step:outputAsset authored but empty for container prim at "
                    + containerPrim.GetPath().GetAsString());
            return std::nullopt;
        }

        std::vector<SdfPropertySpecHandle> propStack = pathAttr.GetPropertyStack();
        if (propStack.empty()) {
            LOG_WARN("Could not determine anchor layer for step:outputAsset on " + containerPrim.GetPath().GetAsString());
            return std::nullopt;
        }

        SdfLayerHandle anchorLayer = propStack.front()->GetLayer();
        fs::path anchoredPath(SdfComputeAssetPathRelativeToLayer(anchorLayer, rawPath));

        return anchoredPath;
    };

    const char* docString = "An autogenerated layer containing the assembly and prototypes";

    if (containerVariantPaths.empty()) {
        fs::path prototypesStageFilePath;
        auto prototypesStagePathOpt = getPrototypesStagePath();

        if (prototypesStagePathOpt.has_value()) {
            prototypesStageFilePath = prototypesStagePathOpt.value();
        } else {
            prototypesStageFilePath = rootPath / (baseName + "-prototypes.usdc");
        }

        std::error_code ec;
        fs::create_directories(prototypesStageFilePath.parent_path(), ec);
        if (ec) {
            LOG_ERR("Failed to create output directory " + prototypesStageFilePath.parent_path().string() + ": " + ec.message());
            return false;
        }
        
        bool makeFreshStage = getStageMakeFresh(selectedPaths, this->pathConfig.containerPrimPath, this->pathConfig.prototypesPath, "", "", stageFilterMap);
        if (stageNeedsUnitReset(prototypesStageFilePath, outputMetersPerUnit)) {
            LOG_INFO("Resetting prototypes stage due to metersPerUnit mismatch: " + prototypesStageFilePath.string());
            makeFreshStage = true;
        }

        UsdStageRefPtr prototypesStage = initUsdStage(prototypesStageFilePath, makeFreshStage);

        UsdPrim existingPrototypesContainer = prototypesStage->GetPrimAtPath(this->pathConfig.prototypesInContainerPath);
        if (existingPrototypesContainer.IsValid() && !existingPrototypesContainer.IsActive())
            existingPrototypesContainer.SetActive(true);

        UsdGeomSetStageMetersPerUnit(prototypesStage, outputMetersPerUnit);
        prototypesStage->SetMetadata(TfToken("metersPerUnit"), outputMetersPerUnit);
        prototypesStage->GetRootLayer()->SetDocumentation(docString);
        UsdPrim containerPrimInPrototypes = prototypesStage->OverridePrim(this->pathConfig.containerPrimPath);
        prototypesStage->SetDefaultPrim(containerPrimInPrototypes.GetPrim());
        prototypesStage->GetRootLayer()->SetDocumentation("Auto generated file that define the prototypes for the assembly");

        AutolibStepPrototypes prototypesScope = AutolibStepPrototypes::Define(prototypesStage, this->pathConfig.prototypesPath);

        UsdGeomXform::Define(prototypesStage, this->pathConfig.assemblyPath);
        
        std::shared_ptr<OpenCascadeAssembly> modelPtr = getOpenCascadeAssembly();
        if (!modelPtr) {
            LOG_ERR("Failed to load OpenCascadeAssembly for container prim at path: " + containerPrim.GetPath().GetAsString());
            return false;
        }

        fs::path prototypesStageDirectory = fs::path(prototypesStage->GetRootLayer()->GetRealPath()).parent_path();
        fs::path containerStagePath = containerStage->GetRootLayer()->GetRealPath();
        std::string subLayerPath = fs::relative(containerStagePath, prototypesStageDirectory).string();

        // std::cout << "Adding sublayer to container stage: " << subLayerPath << std::endl;

        addStageSubLayer(prototypesStage, subLayerPath);

        UsdPrim prototypesInPrototypesStage = prototypesStage->GetPrimAtPath(this->pathConfig.prototypesPath);
        if (prototypesInPrototypesStage.IsValid() && !prototypesInPrototypesStage.IsActive())
            prototypesInPrototypesStage.SetActive(true);

        prototypesStage->Save();
        prototypes.push_back({modelPtr, "", "",  makeFreshStage, prototypesStage, containerStage });
    } else {

        for (const SdfPath& variantPath : containerVariantPaths) {
            if (!selectedPaths.empty() && !isContainerVariantSelected(variantPath)) {
                continue;
            }
            std::pair<std::string, std::string> variantSelection = variantPath.GetVariantSelection();
            const std::string& variantSetName = variantSelection.first;
            const std::string& variantName = variantSelection.second;

            UsdVariantSet vset = containerPrim.GetVariantSet(variantSetName);
            vset.SetVariantSelection(variantName);

            fs::path variantSubPath = rootPath / variantSetName;
            fs::path prototypesStageFilePath;
            auto prototypesStagePathOpt = getPrototypesStagePath();

            if (prototypesStagePathOpt.has_value()) {
                prototypesStageFilePath = prototypesStagePathOpt.value();
            } else {
                prototypesStageFilePath = variantSubPath / (variantSetName + "-" + variantName + "-prototypes.usdc");
            }

            std::error_code ec;
            fs::create_directories(prototypesStageFilePath.parent_path(), ec);
            if (ec) {
                LOG_ERR("Failed to create output directory " + prototypesStageFilePath.parent_path().string() + ": " + ec.message());
                continue;
            }
            
            bool makeFreshStage = getStageMakeFresh(selectedPaths, this->pathConfig.containerPrimPath, this->pathConfig.prototypesPath, variantSetName, variantName, stageFilterMap);
            if (stageNeedsUnitReset(prototypesStageFilePath, outputMetersPerUnit)) {
                LOG_INFO("Resetting prototypes stage due to metersPerUnit mismatch: " + prototypesStageFilePath.string());
                makeFreshStage = true;
            }

            UsdStageRefPtr prototypesStage = initUsdStage(prototypesStageFilePath, makeFreshStage);

            UsdPrim existingPrototypesContainer = prototypesStage->GetPrimAtPath(this->pathConfig.prototypesPath);
            if (existingPrototypesContainer.IsValid() && !existingPrototypesContainer.IsActive())
                existingPrototypesContainer.SetActive(true);

            UsdGeomSetStageMetersPerUnit(prototypesStage, outputMetersPerUnit);
            AutolibStepPrototypes prototypesScope = AutolibStepPrototypes::Define(prototypesStage, this->pathConfig.prototypesPath);
            
            UsdPrim containerPrimInPrototypes = prototypesStage->OverridePrim(this->pathConfig.containerPrimPath);
            prototypesStage->GetRootLayer()->SetDocumentation(docString);
            prototypesStage->SetDefaultPrim(containerPrimInPrototypes);
            prototypesStage->GetRootLayer()->SetDocumentation("Auto generated file that define the prototypes for the assembly");
            UsdGeomXform::Define(prototypesStage, this->pathConfig.assemblyPath);

            fs::path prototypesStageDirectory = fs::path(prototypesStage->GetRootLayer()->GetRealPath()).parent_path();
            fs::path containerStagePath = containerStage->GetRootLayer()->GetRealPath();
            std::string subLayerPath = fs::relative(containerStagePath, prototypesStageDirectory).string();

            addStageSubLayer(prototypesStage, subLayerPath);

            if (!vset.IsValid()) {
                LOG_ERR("Variant set [" + variantSetName + "] not found on container prim for variant [" + variantName + "]");
                continue;
            }

            std::shared_ptr<OpenCascadeAssembly> modelPtr = getOpenCascadeAssembly();
            if (!modelPtr) {
                LOG_ERR("Failed to load OpenCascadeAssembly for container prim at path: " + containerPrim.GetPath().GetAsString());
                return false;
            }

            UsdPrim prototypesInPrototypesStage = prototypesStage->GetPrimAtPath(this->pathConfig.prototypesPath);
            if (prototypesInPrototypesStage.IsValid() && !prototypesInPrototypesStage.IsActive())
                prototypesInPrototypesStage.SetActive(true);

            prototypesStage->Save();
            prototypes.push_back({modelPtr, variantSetName, variantName, makeFreshStage, prototypesStage, containerStage });
        }
    }
    
    return true;
}

bool StepUsdPipeline::populateParamsBank(
    const UsdStageRefPtr& containerStage,
    const UsdPrim& containerPrim,
    const PrototypeContainer& proto,
    const TessParams& variantLevelParams,
    std::map<SdfPath, TessParams>& paramsBank
) {
    fs::path containerFilePath = fs::canonical(
        containerStage->GetRootLayer()->GetResolvedPath().GetPathString()
    );

    UsdStageRefPtr throwawayStage = UsdStage::Open(
        containerFilePath.string(),
        UsdStage::LoadAll
    );

    if (!throwawayStage) {
        LOG_WARN("Could not open throwaway stage from " + containerFilePath.string()
                 + ", falling back to variant-level params.");
        return false;
    }

    throwawayStage->SetEditTarget(
        UsdEditTarget(throwawayStage->GetSessionLayer())
    );

    SdfPath throwawayContainerPath = containerPrim.GetPath();
    SdfPath throwawayPrototypesPath =
        throwawayContainerPath.AppendChild(TfToken("Prototypes"));

    UsdPrim throwawayContainer =
        throwawayStage->GetPrimAtPath(throwawayContainerPath);

    if (!proto.variantSetName.empty() && throwawayContainer.IsValid()) {
        throwawayContainer.GetVariantSet(proto.variantSetName).SetVariantSelection(proto.variantName);
    }

    UsdPrim prototypesOnThrowaway =
        throwawayStage->GetPrimAtPath(throwawayPrototypesPath);

    LOG_DEBUG("prototypesOnThrowaway valid: " + std::string(prototypesOnThrowaway.IsValid() ? "yes" : "no"));

    if (!prototypesOnThrowaway.IsValid()) {
        return false;
    }

    bool wasActive = prototypesOnThrowaway.IsActive();
    if (!wasActive) {
        prototypesOnThrowaway.SetActive(true);
    }

    for (const UsdPrim& child : prototypesOnThrowaway.GetAllChildren()) {
        auto childParams = resolveParams(child, variantLevelParams);

        for (const auto& [throwawayPath, params] : childParams) {
            SdfPath remapped = throwawayPath.ReplacePrefix(
                throwawayPrototypesPath,
                this->pathConfig.prototypesPath
            );
            paramsBank[remapped] = params;
        }
    }

    if (!wasActive) {
        prototypesOnThrowaway.SetActive(wasActive);
    }

    return true;
}

bool StepUsdPipeline::populateTessellationJobs(
    const std::vector<PrototypeContainer>& prototypes,
    const UsdStageRefPtr& containerStage,
    const UsdPrim& containerPrim,
    const std::vector<std::pair<TDF_Label, TopoDS_Shape>>& defs,
    const LabelMap<SdfPath>& prototypePaths,
    double sourceToOutputScale,
    std::vector<TessellationJob>& tessJobs
) {
    for (const auto& proto : prototypes) {

        if (!proto.variantSetName.empty()) {
            UsdVariantSet varSet = containerPrim.GetVariantSet(proto.variantSetName);
            varSet.SetVariantSelection(proto.variantName);
        }

        // We use containerStage to observe any variant opinions authored on the container file
        UsdPrim protocontainerPrim = containerStage->GetPrimAtPath(this->pathConfig.prototypesInContainerPath);

        TessParams containerParams = getTessParams(containerPrim);
        TessParams variantLevelParams = getTessParams(protocontainerPrim, containerParams);

        std::map<SdfPath, TessParams> paramsBank;
        bool sucess = populateParamsBank(
            containerStage,
            containerPrim,
            proto,
            variantLevelParams,
            paramsBank
        );
        
        if (!sucess) {
            LOG_WARN("Failed to populate params bank for prototypes [" + proto.stage->GetRootLayer()->GetRealPath() + "].");
        }

        bool runMesherInParallel = !proto.makeFreshStage;
        std::shared_ptr<PrototypeContainer> protoPtr = std::make_shared<PrototypeContainer>(proto);

        for (size_t i = 0; i < defs.size(); ++i) {
            auto it = prototypePaths.find(defs[i].first);
            if (it == prototypePaths.end()) {
                std::cerr << "Warning: No prototype path found for definition label: " << defs[i].first << std::endl;
                continue;
            }
            SdfPath protoPath = it->second;

            SdfPath paramKeyPath = this->pathConfig.prototypesPath.AppendChild(protoPath.GetNameToken());
            // Re-apply any variant extensions based on variants stored in `proto` 
            // However, resolveParams currently spits out nested paths if variants were found inside it.
            // If wonderful_model.usda has per-prototype opinions like `/Prototypes/rod_1{quality=draft}`
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
                    tessJobs.push_back({protoPtr, (int)i, jobProtoPath, params, TessResult(), runMesherInParallel});
                }
            }
            
            if (!foundVariantForProto) {
                params.unitScale = sourceToOutputScale;
                tessJobs.push_back({protoPtr, (int)i, protoPath, params, TessResult(), runMesherInParallel});
            }
        }
    }
    return true;
}

bool StepUsdPipeline::buildPrototypeAndAssemblyStages(
    const OpenCascadeAssembly& model,
    const std::vector<PrototypeContainer>& prototypes,
    const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
    fs::path rootPath,
    const UsdStageRefPtr& containerStage,
    const UsdPrim& containerPrim,
    double outputMetersPerUnit,
    double sourceToOutputScale,

    LabelMap<SdfPath>& prototypePaths,
    std::vector<std::pair<TDF_Label, TopoDS_Shape>>& defs
) {
    std::vector<SdfPath> nodePaths;

    {
        nodePaths.resize(model.partNodes.size());
        nodePaths[0] = this->pathConfig.assemblyPath;

        for (size_t i = 1; i < model.partNodes.size(); i++) {
            TfErrorMark mark;
            const OpenCascadeAssembly::PartNode& node = model.partNodes[i];

            SdfPath parentPath;
            if (node.parentIdx == 0) {
                parentPath = this->pathConfig.assemblyPath;
            } else {
                parentPath = nodePaths[node.parentIdx];
            }

            if (parentPath.IsEmpty()) continue;

            // instanceLabel entry is unique per slot in the XCAF tree,
            // so two rods under the same parent get different suffixes
            std::string suffix = stableLabelSuffix(node.instanceLabel);
            bool hasRealName = !node.name.empty() && !isAutoGeneratedName(node.name);
            std::string finalName;
            if (hasRealName) {
                finalName = sanitizeUsdName(node.name) + "__" + suffix;
            } else {
                finalName = "__" + suffix;
            }

            nodePaths[i] = parentPath.AppendChild(TfToken(finalName));

            if (!mark.IsClean()) {
                for (const auto& error : mark)
                    std::cerr << "Usd Error: " << error.GetCommentary() << "\n";
            }
        }
    }

    // Write prototype stages
    for (const auto& proto : prototypes) {

        if (!proto.variantSetName.empty()) {
            UsdVariantSet varSet = containerPrim.GetVariantSet(proto.variantSetName);
            varSet.SetVariantSelection(proto.variantName);
        }

        const UsdPrim& prototypesPrim = containerStage->GetPrimAtPath(this->pathConfig.prototypesInContainerPath);

        if (!prototypesPrim.IsValid()) {
            LOG_WARN("Invalid prototypes prim at: " + this->pathConfig.prototypesInContainerPath.GetString());
            return false;
        }

        writePartClass(proto.stage, prototypesPrim, SdfPath("/Part"));

        LabelMap<SdfPath> variantPrototypePaths; // fresh per variant
        writePrototypeXformsInPrototypesStage(
            proto.stage,
            defs,
            selectedPaths,
            proto.variantSetName,
            proto.variantName,
            model.definitionNames,
            variantPrototypePaths,
            proto.makeFreshStage
        );
        prototypePaths.insert(variantPrototypePaths.begin(), variantPrototypePaths.end());
        
        bool isAssemblyInFilter = isAssemblyActiveInFilter(selectedPaths, this->pathConfig.containerPrimPath);
        if (!isAssemblyInFilter) {
            LOG_DEBUG("Assembly stage will be skipped in generation because it is not targeted by selectedPaths.");
        }
        
        bool shouldCreateAssembly = proto.makeFreshStage || isAssemblyInFilter;
        
        if (shouldCreateAssembly) {
            writeAssemblyXforms(
                proto.stage,
                model.partNodes,
                nodePaths,
                prototypePaths,
                sourceToOutputScale
            );
        }
        proto.stage->Save();
    }

    return true;
}

// Stage Filtering
static bool validateVariants(
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

std::optional<StepUsdPipeline> StepUsdPipeline::create(
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
        if (!prim.HasAPI<AutolibStepContainerAPI>()) return;

        AutolibStepContainer container(prim);
        if (!container.GetStepSourceAssetAttr().HasAuthoredValue()) return;

        UsdAttribute pathAttr = container.GetStepSourceAssetAttr();

        SdfAssetPath sdfAssetPath;
        if (!pathAttr.Get(&sdfAssetPath)) {
            LOG_ERR("Failed to get asset path from UsdAttribute");
            return;
        }

        if (sdfAssetPath.GetAssetPath().empty()) {
            LOG_ERR("Resolved asset path is empty for prim: " + prim.GetPath().GetString());
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

    std::unordered_map<SdfAssetPath, OpenCascadeAssembly, SdfAssetPath::Hash> modelCache;

    {
        LOG_SCOPED_TIMER("Load and Parse STEP Models (" + std::to_string(referencedStepAssetPaths.size()) + " files)");
        WorkParallelForEach( referencedStepAssetPaths.begin(), referencedStepAssetPaths.end(), [&](const SdfAssetPath& assetPath) {
            std::string resolvedPath = assetPath.GetResolvedPath();

            if (resolvedPath.empty()) {
                LOG_ERR("Failed to resolve path to: " + assetPath.GetAssetPath());
                return;
            }

            std::optional<OpenCascadeAssembly> optModel = OpenCascadeAssembly::loadFromFile(resolvedPath);

            if (!optModel.has_value()) {
                LOG_ERR("Failed to load STEP model from " + resolvedPath);
                return;
            }

            modelCache.insert_or_assign(assetPath, std::move(*optModel));
        });
    }

    return StepUsdPipeline(stage, modelCache);
}

void StepUsdPipeline::populateUsd(
    UsdStageRefPtr containerStage,
    UsdPrim& containerPrim,
    const std::unordered_set<SdfPath, SdfPath::Hash> selectedInContainerPaths
) {
    LOG_SCOPED_TIMER("StepUsdPipeline::populateUsd");
    TfErrorMark mark;
    
    AutolibStepContainer container(containerPrim);

    if (!container.GetStepSourceAssetAttr().HasAuthoredValue()) return;
    UsdAttribute pathAttr = container.GetStepSourceAssetAttr();
    
    SdfAssetPath sdfAssetPath;
    if (!pathAttr.Get(&sdfAssetPath)) {
        LOG_ERR("Failed to get asset path from UsdAttribute");
        return;
    }

    fs::path assetPath = sdfAssetPath.GetResolvedPath();
    LOG_INFO("Processing STEP file: " + assetPath.string());

    // Load the model, using the cache to avoid re-parsing the same STEP file.
    auto iter = modelCache.find(sdfAssetPath);
    if (iter == modelCache.end()) {
        LOG_ERR("Model not found in cache for asset path: " + assetPath.string());
        return;
    }

    const OpenCascadeAssembly& model = iter->second;

    //model.debugPrintInstances();

    if (!validateVariants(containerStage, containerPrim.GetPath(), selectedInContainerPaths)) {
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

    containerStage->Unload();

    fs::path containerFilePath = fs::canonical(containerStage->GetRootLayer()->GetResolvedPath().GetPathString()).remove_filename();

    this->pathConfig.prototypesInContainerPath = SdfPath("/Prototypes").ReplacePrefix(SdfPath::AbsoluteRootPath(), containerPrim.GetPath());

    SdfPath containerParentPath = containerPrim.GetPath().GetParentPath();
    this->pathConfig.containerPrimPath = containerPrim.GetPath().ReplacePrefix(containerParentPath, SdfPath::AbsoluteRootPath());
    this->pathConfig.assemblyPath = SdfPath("/Assembly").ReplacePrefix(SdfPath::AbsoluteRootPath(), this->pathConfig.containerPrimPath);
    this->pathConfig.prototypesPath = SdfPath("/Prototypes").ReplacePrefix(SdfPath::AbsoluteRootPath(), this->pathConfig.containerPrimPath);

    // Normalize paths from container-space to raw-space
    std::unordered_set<SdfPath, SdfPath::Hash> selectedPaths = reparentPaths(containerParentPath, selectedInContainerPaths);
    std::unordered_set<SdfPath, SdfPath::Hash> containerVariantPaths = reparentPaths(containerParentPath, getVariantsOnPrim(containerPrim));

    std::unordered_map<SdfPath, StageFilterInfo, SdfPath::Hash> stageFilterMap;
    resolveStageFilterInfo(selectedPaths, this->pathConfig.containerPrimPath, this->pathConfig.prototypesPath, stageFilterMap);

    // Create the root layer for the container stage

    std::string baseName = model.stepPath.stem().string();
    fs::path rootPath = containerFilePath;

    // Setup Prototype Stages for all variants
    std::vector<PrototypeContainer> prototypes;
    populatePrototypeContainers(
        containerPrim,
        containerStage,
        selectedPaths,
        rootPath,
        baseName,
        containerVariantPaths,
        outputMetersPerUnit,
        stageFilterMap,
        prototypes
    );
    
    // Write Xforms
    const double sourceToOutputScale = model.metersPerUnit / outputMetersPerUnit;
    std::vector<std::pair<TDF_Label, TopoDS_Shape>> defs(
        model.definitionShapes.begin(),
        model.definitionShapes.end()
    );

    std::sort(
        defs.begin(), 
        defs.end(), 
        [this](const std::pair<TDF_Label, TopoDS_Shape>& a, const std::pair<TDF_Label, TopoDS_Shape>& b) {
            return stableLabelSuffix(a.first) < stableLabelSuffix(b.first);
        }
    );

    LabelMap<SdfPath> prototypePaths;
    buildPrototypeAndAssemblyStages(
        model,
        prototypes,
        selectedPaths,
        rootPath,
        containerStage,
        containerPrim,
        outputMetersPerUnit,
        sourceToOutputScale,
        prototypePaths,
        defs
    );

    // Tessellation Jobs
    std::vector<TessellationJob> tessJobs;
    populateTessellationJobs(
        prototypes,
        containerStage,
        containerPrim,
        defs,
        prototypePaths,
        sourceToOutputScale,
        tessJobs
    );

    // Tessellation  
    tessellateGeometry(tessJobs, defs, selectedPaths);

    // Write Geometry
    LOG_DEBUG("Preparing to gather geometry jobs...");
    LOG_DEBUG("Starting geometry writing for " + std::to_string(prototypes.size()) + " prototypes.");
    for (const auto& proto : prototypes) {
        LOG_DEBUG("Processing prototype stage: " + proto.stage->GetRootLayer()->GetIdentifier() + " (variant: " + proto.variantSetName + "=" + proto.variantName + ")");
        std::vector<ProtoGeomJob> geomJobs;
        for (const auto& job : tessJobs) {
            if (job.proto->stage->GetRootLayer()->GetRealPath() == proto.stage->GetRootLayer()->GetRealPath()) {
                geomJobs.push_back({job.prototypePath, job.result, job.params});
            }
        }
        
        LOG_DEBUG("Writing " + std::to_string(geomJobs.size()) + " geometry jobs to prototype stage.");
        writePrototypeGeometries(proto.stage, geomJobs, selectedPaths, proto.variantSetName, proto.variantName);
        LOG_DEBUG("Saving prototype stage.");
        proto.stage->Save();
    }

    LOG_DEBUG("Deactivating original prototype container in container stage: " + this->pathConfig.prototypesPath.GetString());

    if (!mark.IsClean()) {
        for (const auto& error : mark) std::cerr << "Usd: " << error.GetCommentary() << "\n";
    }
}