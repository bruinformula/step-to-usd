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

#include "StepUSD/StepUsdPipeline.h"
#include "StepUSD/OpenCascadeAssembly.h"
#include "StepUSD/Logger.h"
#include "StepUSD/UsdUtils.h"

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

static void findMatchingPrefixWithoutVariants(
    const SdfPath& path,
    const SdfPath& targetPath,
    SdfPath& resolvedPrefix
) {
    // Find the prefix whose path, after removing variant selections,
    // matches targetPath. Return the original prefix so its variants
    // are preserved.
    for (const SdfPath& prefix : path.GetPrefixes()) {
        if (prefix.StripAllVariantSelections() == targetPath) {
            resolvedPrefix = prefix;
            return;
        }
    }

    if (path.StripAllVariantSelections() == targetPath) {
        resolvedPrefix = path;
        return;
    }
}

enum class StageSelection {
    None,
    SpecificPrototype,
    WholeStage
};

StageSelection classifySelection(
    const SdfPath& selectedPath,
    const SdfPath& containerPrimPath,
    const SdfPath& prototypesPath,
    SdfPath& stageKey)
{

    // If /Prototypes or /Prototypes/** were selected 
    findMatchingPrefixWithoutVariants(selectedPath, prototypesPath, stageKey);
    if (!stageKey.IsEmpty()) {
        const SdfPath cleanPath = selectedPath.StripAllVariantSelections();
        if (cleanPath == prototypesPath)
            return StageSelection::WholeStage;

        return StageSelection::SpecificPrototype;
    }

    // If the container was selected 
    SdfPath containerKey = {};
    findMatchingPrefixWithoutVariants(selectedPath, containerPrimPath, containerKey);

    if (containerKey.IsEmpty())
        return StageSelection::None;

    // We should target the Prototypes
    stageKey = containerKey.AppendChild(TfToken("Prototypes"));

    return StageSelection::WholeStage;
}

void resolveStageFilterInfo(
    const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
    const SdfPath& containerPrimPath,
    const SdfPath& prototypesPath,
    std::unordered_map<SdfPath, StageFilterInfo, SdfPath::Hash>& stageFilterMap)
{
    for (const SdfPath& selectedPath : selectedPaths) {
        SdfPath stageKey;
        StageSelection selection = classifySelection(selectedPath, containerPrimPath, prototypesPath, stageKey);
        
        switch (selection) {
            case StageSelection::SpecificPrototype:
                stageFilterMap[stageKey].hasSpecificPrototypes = true;
                break;

            case StageSelection::WholeStage:
                stageFilterMap[stageKey].makeFresh = true;
                break;

            case StageSelection::None:
                break;
        }
    }

    for (const auto& [stageKey, info] : stageFilterMap) {
        if (info.makeFresh && info.hasSpecificPrototypes) {
            LOG_WARN(
                "SelectedPaths targets both the entire prototype stage and "
                "specific prototypes within it for stage [" +
                stageKey.GetAsString() +
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
    const std::unordered_set<SdfPath, SdfPath::Hash>& containerVariantPaths,
    double outputMetersPerUnit,
    std::vector<PrototypeContainer>& prototypes
) {

    std::unordered_map<SdfPath, StageFilterInfo, SdfPath::Hash> stageFilterMap;
    resolveStageFilterInfo(selectedPaths, this->pathConfig.containerPrimPath, this->pathConfig.prototypesPath, stageFilterMap);

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

    // The step container indicates entry for whatever model that I would like to mesh
    // It has three members that are resolved in lambdas below which are effectivly 
    // inputs and outputs.
    AutolibStepContainer container(containerPrim);

    // Resolves the OpenCascade document for the cacheAsset and sourceAsset
    auto getOpenCascadeAssembly = [&](void) -> std::shared_ptr<OpenCascadeAssembly> {
        if (!container.GetStepSourceAssetAttr().HasAuthoredValue()) return nullptr;
        UsdAttribute sourceAttr = container.GetStepSourceAssetAttr();
        
        SdfAssetPath sourceAssetPath;
        if (!sourceAttr.Get(&sourceAssetPath)) {
            LOG_ERR("Failed to get asset path from UsdAttribute");
            return nullptr;
        }

        UsdAttribute cacheAttr = container.GetStepCacheAssetAttr();
        
        SdfAssetPath cacheAssetPath;
        if (!cacheAttr.Get(&cacheAssetPath)) {
            LOG_ERR("Failed to get asset path from UsdAttribute");
            return nullptr;
        }

        fs::path sourcePath = sourceAssetPath.GetResolvedPath();
        fs::path cachePath = cacheAssetPath.GetResolvedPath();

        // Load the model, using the cache to avoid re-parsing the same STEP file.
        StepBundleKey key{sourcePath, cachePath, containerPrim.GetPath()};
        auto iter = modelCache.find(key);
        if (iter == modelCache.end()) {
            LOG_ERR("Model not found in cache for asset path: " + sourcePath.string());
            return nullptr;
        }
        return std::make_shared<OpenCascadeAssembly>(iter->second);
    };

    // Resolves the output path for the prototypes and assembly to be written to
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

    // A given prim may contain variants. Each variant will resolve to its own stage.
    // This is just a check to enforce output path existence and uniqueness  
    std::vector<std::pair<std::string, std::string>> prototypesInits;

    if (containerVariantPaths.empty()) {
        prototypesInits.push_back({"", ""});
    } else {
        // just in case different variants point to the same output assets
        // don't want to overwrite
        std::unordered_set<fs::path> seenOutputAssets;
        for (const SdfPath& variantPath : containerVariantPaths) {
            if (!selectedPaths.empty() && !isContainerVariantSelected(variantPath)) {
                continue;
            }

            std::pair<std::string, std::string> variantSelection = variantPath.GetVariantSelection();
            const std::string& variantSetName = variantSelection.first;
            const std::string& variantName = variantSelection.second;

            UsdVariantSet vset = containerPrim.GetVariantSet(variantSetName);
            vset.SetVariantSelection(variantName);

            auto prototypesStagePathOpt = getPrototypesStagePath();
            if (!prototypesStagePathOpt.has_value()) continue;

            // Can't write to an empty path. Though one will be 
            // automatically generated if it doesn't exist
            fs::path prototypesStageFilePath = prototypesStagePathOpt.value();
            if (prototypesStageFilePath.empty()) {
                LOG_ERR("Prototypes stage file path is empty");
                return false;
            }

            // Give a warning about muliple variants that will output 
            // the same file and overwrite each other 
            if (seenOutputAssets.count(prototypesStageFilePath)) {
                LOG_ERR("Duplicate output asset detected: " + prototypesStageFilePath.string());
                return false;
            }
            seenOutputAssets.insert(prototypesStageFilePath);

            prototypesInits.push_back(std::move(variantSelection));
        }
    }
    
    const char* docString = "An autogenerated layer containing the assembly and prototypes";
    fs::path containerFilePath = fs::canonical(
        containerStage->GetRootLayer()->GetResolvedPath().GetPathString()
    );
    
    for (const auto& init : prototypesInits) {
        const std::string& variantSetName = init.first;
        const std::string& variantName = init.second;
        const bool hasVariant = !variantSetName.empty();

        if (hasVariant) {
            UsdVariantSet vset = containerPrim.GetVariantSet(variantSetName);
            if (!vset.IsValid()) {
                LOG_ERR("Variant set [" + variantSetName + "] not found on container prim for variant [" + variantName + "]");
                continue;
            }
            vset.SetVariantSelection(variantName);
        }

        std::shared_ptr<OpenCascadeAssembly> modelPtr = getOpenCascadeAssembly();
        if (!modelPtr) {
            LOG_ERR("Failed to load OpenCascadeAssembly for container prim at path: " + containerPrim.GetPath().GetAsString());
            return false;
        }

        // Get the stage name if it exists 
        // Otherwise author some based on the variant names
        fs::path prototypesStageFilePath;
        auto prototypesStagePathOpt = getPrototypesStagePath();
        if (prototypesStagePathOpt.has_value()) {
            prototypesStageFilePath = prototypesStagePathOpt.value();
        } else if (hasVariant) {
            fs::path variantSubPath = containerFilePath / variantSetName;
            prototypesStageFilePath = variantSubPath / (variantSetName + "-" + variantName + "-prototypes.usdc");
        } else {
            std::string baseName = modelPtr->stepPath.stem().string();
            prototypesStageFilePath = containerFilePath / (baseName + "-prototypes.usdc");
        }

        std::error_code ec;
        fs::create_directories(prototypesStageFilePath.parent_path(), ec);
        if (ec) {
            LOG_ERR("Failed to create output directory " + prototypesStageFilePath.parent_path().string() + ": " + ec.message());
            if (hasVariant) continue;  
            return false;
        }

        // determine based on the selected paths whether the stage should be made anew
        bool makeFreshStage = getStageMakeFresh(selectedPaths, this->pathConfig.containerPrimPath,
                                                this->pathConfig.prototypesPath, variantSetName, variantName, stageFilterMap);
        if (stageNeedsUnitReset(prototypesStageFilePath, outputMetersPerUnit)) {
            LOG_INFO("Resetting prototypes stage due to metersPerUnit mismatch: " + prototypesStageFilePath.string());
            makeFreshStage = true;
        }

        UsdStageRefPtr prototypesStage = initUsdStage(prototypesStageFilePath, makeFreshStage);

        const SdfPath& prototypesPath = this->pathConfig.prototypesPath;

        UsdGeomSetStageMetersPerUnit(prototypesStage, outputMetersPerUnit);
        prototypesStage->SetMetadata(TfToken("metersPerUnit"), outputMetersPerUnit);
        prototypesStage->GetRootLayer()->SetDocumentation(docString);

        AutolibStepPrototypes prototypesScope = AutolibStepPrototypes::Define(prototypesStage, prototypesPath);

        UsdPrim containerPrimInPrototypes = prototypesStage->OverridePrim(this->pathConfig.containerPrimPath);
        prototypesStage->SetDefaultPrim(containerPrimInPrototypes);

        UsdGeomXform::Define(prototypesStage, this->pathConfig.assemblyPath);

        fs::path prototypesStageDirectory = fs::path(prototypesStage->GetRootLayer()->GetRealPath()).parent_path();
        fs::path containerStagePath = containerStage->GetRootLayer()->GetRealPath();
        std::string subLayerPath = fs::relative(containerStagePath, prototypesStageDirectory).string();

        addStageSubLayer(prototypesStage, subLayerPath);

        const double sourceToOutputScale = modelPtr->metersPerUnit / outputMetersPerUnit;

        prototypesStage->Save();
        prototypes.push_back({modelPtr, variantSetName, variantName, makeFreshStage, prototypesStage, containerStage, sourceToOutputScale});
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
    SdfPath throwawayPrototypesPath = throwawayContainerPath.AppendChild(TfToken("Prototypes"));

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

    // Just in case the protoypes prims was set active = false 
    prototypesOnThrowaway.SetActive(true);

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


    return true;
}

bool StepUsdPipeline::populateTessellationJobs(
    const std::vector<PrototypeContainer>& prototypes,
    const UsdStageRefPtr& containerStage,
    const UsdPrim& containerPrim,
    std::vector<TessellationJob>& tessJobs
) {
    for (const auto& proto : prototypes) {

        if (!proto.variantSetName.empty()) {
            UsdVariantSet varSet = containerPrim.GetVariantSet(proto.variantSetName);
            varSet.SetVariantSelection(proto.variantName);
        }

        // We use containerStage to observe any variant opinions authored on the container file
        UsdPrim protocontainerPrim = containerStage->GetPrimAtPath(this->pathConfig.prototypesPath);

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

        std::shared_ptr<PrototypeContainer> protoPtr = std::make_shared<PrototypeContainer>(proto);

        const auto& defs = proto.model->getDefinitionShapes();

        for (size_t i = 0; i < defs.size(); ++i) {
            auto it = proto.model->prototypePaths.find(defs[i].first);
            if (it == proto.model->prototypePaths.end()) {
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
                    params.unitScale = proto.sourceToOutputScale;
                    
                    // We need the prototypePath to include the variant
                    // selections so writer knows where to author
                    SdfPath jobProtoPath = protoPath;
                    auto variantSelection = kv.first.GetVariantSelection();
                    if (!variantSelection.first.empty()) {
                        jobProtoPath = jobProtoPath.AppendVariantSelection(variantSelection.first, variantSelection.second);
                    }
                    
                    //std::cout << "DEBUG: Queueing job for " << jobProtoPath.GetString() << " (defIndex " << i << ")\n";
                    tessJobs.push_back({protoPtr, (int)i, jobProtoPath, params, TessellationRoutine()});
                }
            }
            
            if (!foundVariantForProto) {
                params.unitScale = proto.sourceToOutputScale;
                tessJobs.push_back({protoPtr, (int)i, protoPath, params, TessellationRoutine()});
            }
        }
    }
    return true;
}

bool StepUsdPipeline::buildPrototypeStages(
    std::vector<PrototypeContainer>& prototypes,
    const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
    const UsdStageRefPtr& containerStage,
    const UsdPrim& containerPrim
) {
    // Write prototype stages
    for (auto& proto : prototypes) {

        if (!proto.variantSetName.empty()) {
            UsdVariantSet varSet = containerPrim.GetVariantSet(proto.variantSetName);
            varSet.SetVariantSelection(proto.variantName);
        }

        const UsdPrim& prototypesPrim = containerStage->GetPrimAtPath(this->pathConfig.prototypesPath);

        if (!prototypesPrim.IsValid()) {
            LOG_WARN("Invalid prototypes prim at: " + this->pathConfig.prototypesPath.GetString());
            return false;
        }

        writePartClass(proto.stage, prototypesPrim, SdfPath("/Part"));

        LabelMap<SdfPath> variantPrototypePaths; // fresh per variant
        writePrototypeXforms(
            proto,
            variantPrototypePaths
        );
        proto.model->prototypePaths.insert(variantPrototypePaths.begin(), variantPrototypePaths.end());
        
        bool isAssemblyInFilter = isAssemblyActiveInFilter(selectedPaths, this->pathConfig.containerPrimPath);
        if (!isAssemblyInFilter) {
            LOG_DEBUG("Assembly stage will be skipped in generation because it is not targeted by selectedPaths.");
        }
        
        bool shouldCreateAssembly = proto.makeFreshStage || isAssemblyInFilter;
        
        std::vector<SdfPath> nodePaths = proto.model->getNodePaths(this->pathConfig.assemblyPath);

        if (shouldCreateAssembly) {
            writeAssemblyXforms(
                proto,
                nodePaths
            );
        }
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

    std::unordered_set<StepBundleKey, StepBundleKey::Hash> referencedAssetBundles;

    // Do a scan for all refernced Step Assets, so 
    // we can load them in parallel and cache the 
    // results to avoid redundant parsing of the same STEP file.
    // Helper to extract asset paths from a prim under its current variant context
    auto collectFromPrim = [&](const UsdPrim& prim) {
        if (!prim.HasAPI<AutolibStepContainerAPI>()) return;

        AutolibStepContainer container(prim);
        if (!container.GetStepSourceAssetAttr().HasAuthoredValue()) return;

        UsdAttribute sourceAttr = container.GetStepSourceAssetAttr();
        UsdAttribute cacheAttr = container.GetStepCacheAssetAttr();

        SdfAssetPath sourceAssetPath;
        if (!sourceAttr.Get(&sourceAssetPath)) {
            LOG_ERR("Failed to get sourceAsset path from UsdAttribute");
            return;
        }

        if (sourceAssetPath.GetAssetPath().empty()) {
            LOG_ERR("Resolved sourceAsset path is empty for prim: " + prim.GetPath().GetString());
            return;
        }
        SdfAssetPath cacheAssetPath;
        if (!cacheAttr.Get(&cacheAssetPath)) {
            LOG_ERR("Failed to get cacheAsset path from UsdAttribute");
            return;
        }

        if (cacheAssetPath.GetAssetPath().empty()) {
            LOG_ERR("Resolved cacheAsset path is empty for prim: " + prim.GetPath().GetString());
            return;
        }

        fs::path stepPath = sourceAssetPath.GetResolvedPath();
        fs::path xbfPath = cacheAssetPath.GetResolvedPath();

        if (xbfPath.empty()) {
            xbfPath = stepPath;
            xbfPath.replace_extension("xbf");
            LOG_WARN("XBF path is missing for STEP file: " + prim.GetPath().GetString());
        }

        // This should probably be a part of usd validate 
        for (const auto& bundle : referencedAssetBundles) {
            bool xbfMissMatch = bundle.xbfPath != xbfPath;
            bool stepMissMatch = bundle.stepPath != stepPath;

            if (!stepMissMatch && xbfMissMatch) {
                LOG_ERR("Found prim where sourceAsset points to different cacheAssets for prim: " + 
                    prim.GetPath().GetString() + " and " + bundle.primPath.GetString());
                return;
            }

            if (stepMissMatch && !xbfMissMatch) {
                LOG_ERR("Found prim where cacheAsset points to different sourceAssets for prim: " + 
                    prim.GetPath().GetString() + " and " + bundle.primPath.GetString());
                return;
            }
        }

        referencedAssetBundles.insert({stepPath, xbfPath, prim.GetPath()});
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
        const std::vector<std::string> variantSetNames = prim.GetVariantSets().GetNames();

        if (variantSetNames.empty()) {
            collectFromPrim(prim);
        } else {
            collectAllVariants(prim, variantSetNames, 0);
        }
    }

    std::unordered_map<StepBundleKey, OpenCascadeAssembly, StepBundleKey::Hash> modelCache;

    {
        LOG_SCOPED_TIMER("Load and Parse STEP Models (" + std::to_string(referencedAssetBundles.size()) + " files)");
        WorkParallelForEach( referencedAssetBundles.begin(), referencedAssetBundles.end(), [&](const StepBundleKey& bundle) {

            if (bundle.stepPath.empty()) {
                LOG_ERR("Failed to resolve path to: " + bundle.stepPath.string());
                return;
            }

            fs::path xbfPath;

            if (bundle.xbfPath.empty()) {
                xbfPath = bundle.stepPath;
                xbfPath.replace_extension("xbf");
                LOG_WARN("XBF path is missing for STEP file: " + bundle.primPath.GetString());
            } else {
                xbfPath = bundle.xbfPath;
            }

            std::optional<OpenCascadeAssembly> optModel = OpenCascadeAssembly::loadFromFile(bundle.stepPath, xbfPath);

            if (!optModel.has_value()) {
                LOG_ERR("Failed to load STEP model from " + bundle.stepPath.string());
                return;
            }

            modelCache.insert_or_assign(bundle, std::move(*optModel));
        });
    }

    return StepUsdPipeline(stage, modelCache);
}

void StepUsdPipeline::populateUsd(
    UsdStageRefPtr containerStage,
    UsdPrim& containerPrim,
    const std::unordered_set<SdfPath, SdfPath::Hash> selectedPaths
) {
    LOG_SCOPED_TIMER("StepUsdPipeline::populateUsd");
    TfErrorMark mark;

    // Are the variants requested real and on the stage
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

    fs::path containerFilePath = fs::canonical(containerStage->GetRootLayer()->GetResolvedPath().GetPathString()).remove_filename();

    // These are a set of "globals" used by the program 
    // If the container prim is /Model
    // prototypesPath -> /Model/Prototypes
    // assemblyPath -> /Model/Assembly
    this->pathConfig.containerPrimPath = containerPrim.GetPath();
    this->pathConfig.prototypesPath = SdfPath("/Prototypes").ReplacePrefix(SdfPath::AbsoluteRootPath(), containerPrim.GetPath());
    this->pathConfig.assemblyPath = SdfPath("/Assembly").ReplacePrefix(SdfPath::AbsoluteRootPath(), this->pathConfig.containerPrimPath);

    // Normalize paths from container-space to raw-space
    std::unordered_set<SdfPath, SdfPath::Hash> containerVariantPaths = getVariantsOnPrim(containerPrim);

    // Setup Prototype Stages for all variants.
    std::vector<PrototypeContainer> prototypes;
    populatePrototypeContainers(
        containerPrim,
        containerStage,
        selectedPaths,
        containerVariantPaths,
        outputMetersPerUnit,
        prototypes
    );
    
    // Writes a bunch of Xforms but these don't contain any primvar data just 
    // sort of a shell with just the flattened prototypes and the transformed 
    // assembly prims with reference arcs to the prototypes 
    buildPrototypeStages(
        prototypes,
        selectedPaths,
        containerStage,
        containerPrim
    );

    if (!mark.IsClean()) {
        for (const auto& error : mark) 
            LOG_ERR("After buildPrototypeStages: " + error.GetCommentary());
    }

    // Generates the tessellation jobs to populate the "shell" generated earlier. 
    std::vector<TessellationJob> tessJobs;
    populateTessellationJobs(
        prototypes,
        containerStage,
        containerPrim,
        tessJobs
    );

    tessellateGeometry(tessJobs, selectedPaths);

    // Writing takes place at this seperate step for efficency reasons.
    // There is both the edit routing of the variants and such
    // There is allow the opprotunity to write in huge SdfChangeBlocks
    // Which ensures the stage recomposition happens only once
 
    LOG_DEBUG("Started writing " + std::to_string(prototypes.size()) + " prototypes.");
    for (const auto& proto : prototypes) {
        LOG_DEBUG("Starting geometry writing for: " + proto.stage->GetRootLayer()->GetIdentifier() + " (variant: " + proto.variantSetName + "=" + proto.variantName + ")");
        std::vector<ProtoGeomJob> geomJobs;
        for (const auto& job : tessJobs) {
            if (job.proto->stage->GetRootLayer()->GetRealPath() == proto.stage->GetRootLayer()->GetRealPath()) {
                geomJobs.push_back({job.prototypePath, job.params, job.routine});
            }
        }
        
        LOG_DEBUG("Writing " + std::to_string(geomJobs.size()) + " geometry jobs to prototype stage.");
        writePrototypeGeometries(proto.stage, geomJobs, selectedPaths, proto.variantSetName, proto.variantName);
        LOG_DEBUG("Saving prototype stage.");
        proto.stage->Save();
    }

    if (!mark.IsClean()) {
        for (const auto& error : mark) 
            LOG_ERR("Usd: " + error.GetCommentary());
    }
}