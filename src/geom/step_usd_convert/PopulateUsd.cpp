#include <iostream>
#include <optional>
#include <string>
#include <filesystem>
#include <cmath>
#include <map>
#include <unordered_set>
#include <utility>
#include <vector>


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

bool UsdStepExporter::isPrototypeActiveInFilter(
    const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
    const SdfPath& containerPrimPath,
    const SdfPath& prototypePath,
    const std::string& variantSetName,
    const std::string& variantName
) {
    if (selectedPaths.empty())
        return true;

    // Resolve the fully qualified prototypes path for the current container variant permutation
    SdfPath basePrototypesKey = containerPrimPath.AppendChild(TfToken("Prototypes"));
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

bool UsdStepExporter::isAssemblyActiveInFilter(
    const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
    const SdfPath& containerPrimPath,
    const SdfPath& prototypePath
) {
    if (selectedPaths.empty()) return true;

    // remove all {Variant=Selection}
    SdfPath cleanProto = prototypePath.StripAllVariantSelections();
    SdfPath cleanContainer = containerPrimPath.StripAllVariantSelections();
    SdfPath assemblyBase = cleanContainer.AppendChild(TfToken("Assembly"));

    if (!cleanProto.HasPrefix(assemblyBase)) {
        return false;
    }
    // Check against selections
    for (const SdfPath& sel : selectedPaths) {
        SdfPath cleanSel = sel.StripAllVariantSelections();
        // Selection is a parent: /Container or /Container/Assembly
        // Selection is the prim or a child: /Container/Assembly/rod0 or /Container/Assembly/rod0/screw
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

bool UsdStepExporter::populatePrototypeContainers(
    const std::unordered_set<SdfPath, SdfPath::Hash>& containerVariantPaths,
    const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
    fs::path rootPath,
    std::string baseName,
    const SdfPath& prototypesPath,
    const SdfPath& prototypesInContainerPath,
    const SdfPath& containerPrimPath,
    const SdfPath& assemblyPath,
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

    if (containerVariantPaths.empty()) {
        fs::path prototypesStageFilePath = rootPath / (baseName + "-prototypes.usdc");
        bool makeFreshStage = getStageMakeFresh(selectedPaths, containerPrimPath, prototypesPath, "", "", stageFilterMap);
        if (stageNeedsUnitReset(prototypesStageFilePath, outputMetersPerUnit)) {
            LOG_INFO("Resetting prototypes stage due to metersPerUnit mismatch: " + prototypesStageFilePath.string());
            makeFreshStage = true;
        }

        UsdStageRefPtr prototypesStage = initUsdStage(prototypesStageFilePath, makeFreshStage);

        UsdPrim existingPrototypesContainer = prototypesStage->GetPrimAtPath(prototypesInContainerPath);
        if (existingPrototypesContainer.IsValid() && !existingPrototypesContainer.IsActive())
            existingPrototypesContainer.SetActive(true);

        UsdGeomSetStageMetersPerUnit(prototypesStage, outputMetersPerUnit);
        prototypesStage->SetMetadata(TfToken("metersPerUnit"), outputMetersPerUnit);
        UsdPrim containerPrimInPrototypes = prototypesStage->DefinePrim(containerPrimPath);
        prototypesStage->SetDefaultPrim(containerPrimInPrototypes.GetPrim());
        prototypesStage->GetRootLayer()->SetDocumentation("Auto generated file that define the prototypes for the assembly");

        AutolibStepFilePrototypes prototypesScope = AutolibStepFilePrototypes::Define(prototypesStage, prototypesPath);

        UsdGeomXform::Define(prototypesStage, assemblyPath);
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

            fs::path variantSubPath = rootPath / variantSetName;
            if (!fs::exists(variantSubPath)) {
                if (!fs::create_directory(variantSubPath)) {
                    std::cerr << "Error: Failed to create directory " << variantSubPath << "\n";
                }
            }

            fs::path prototypesStageFilePath = variantSubPath / (baseName + variantSetName + "-" + variantName + "-prototypes.usdc");
            bool makeFreshStage = getStageMakeFresh(selectedPaths, containerPrimPath, prototypesPath, variantSetName, variantName, stageFilterMap);
            if (stageNeedsUnitReset(prototypesStageFilePath, outputMetersPerUnit)) {
                LOG_INFO("Resetting prototypes stage due to metersPerUnit mismatch: " + prototypesStageFilePath.string());
                makeFreshStage = true;
            }

            UsdStageRefPtr prototypesStage = initUsdStage(prototypesStageFilePath, makeFreshStage);

            UsdPrim existingPrototypesContainer = prototypesStage->GetPrimAtPath(prototypesPath);
            if (existingPrototypesContainer.IsValid() && !existingPrototypesContainer.IsActive())
                existingPrototypesContainer.SetActive(true);

            UsdGeomSetStageMetersPerUnit(prototypesStage, outputMetersPerUnit);
            AutolibStepFilePrototypes prototypesScope = AutolibStepFilePrototypes::Define(prototypesStage, prototypesPath);
            
            UsdPrim containerPrimInPrototypes = prototypesStage->DefinePrim(containerPrimPath);
            prototypesStage->SetDefaultPrim(containerPrimInPrototypes.GetPrim());
            prototypesStage->GetRootLayer()->SetDocumentation("Auto generated file that define the prototypes for the assembly");
            UsdGeomXform::Define(prototypesStage, assemblyPath);

            prototypesStage->Save();
            prototypes.push_back({variantSetName, variantName, prototypesStageFilePath, prototypesStage, makeFreshStage});
        }
    }
    return true;
}

bool UsdStepExporter::populateParamsBank(
    const UsdStageRefPtr& containerStage,
    const UsdPrim& containerPrim,
    const PrototypeContainer& proto,
    const SdfPath& prototypesPath,
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

    LOG_DEBUG("prototypesOnThrowaway valid: " +
              std::string(prototypesOnThrowaway.IsValid() ? "yes" : "no"));

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
                prototypesPath
            );
            paramsBank[remapped] = params;
        }
    }

    if (!wasActive) {
        prototypesOnThrowaway.SetActive(wasActive);
    }

    return true;
}

bool UsdStepExporter::populateTessellationJobs(
    const std::vector<PrototypeContainer>& prototypes,
    const UsdStageRefPtr& containerStage,
    const UsdPrim& containerPrim,
    const SdfPath& prototypesInContainerPath,
    const SdfPath& prototypesPath,
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
        UsdPrim protocontainerPrim = containerStage->GetPrimAtPath(prototypesInContainerPath);

        TessParams containerParams = getTessParams(containerPrim);
        TessParams variantLevelParams = getTessParams(protocontainerPrim, containerParams);

        std::map<SdfPath, TessParams> paramsBank;
        bool sucess = populateParamsBank(
            containerStage,
            containerPrim,
            proto,
            prototypesPath,
            variantLevelParams,
            paramsBank
        );
        
        if (!sucess) {
            LOG_WARN("Failed to populate params bank for prototypes [" + proto.filePath.filename().string() + "].");
        }

        bool runMesherInParallel = !proto.makeFreshStage;

        for (size_t i = 0; i < defs.size(); ++i) {
            SdfPath protoPath = prototypePaths.at(defs[i].first); // /Prototypes/rod0

            SdfPath paramKeyPath = prototypesPath.AppendChild(protoPath.GetNameToken());            
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
    return true;
}

bool UsdStepExporter::buildPrototypeAndAssemblyStages(
    const StepModel& model,
    const std::vector<PrototypeContainer>& prototypes,
    const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
    const SdfPath& assemblyPath,
    const SdfPath& containerPrimPath,
    const SdfPath& prototypesPath,
    const SdfPath& prototypesInContainerPath,
    const UsdStageRefPtr& containerStage,
    const UsdPrim& containerPrim,
    const UsdStageRefPtr& rootStage,
    const fs::path& rootStageFilePath,
    const fs::path& rootPath,
    double outputMetersPerUnit,
    double sourceToOutputScale,

    LabelMap<SdfPath>& prototypePaths,
    std::vector<std::pair<TDF_Label, TopoDS_Shape>>& defs
) {
    std::vector<SdfPath> nodePaths;
    {       
        std::unordered_map<std::string, int> nameCounts;
        // compute Part Nodes paths and ensure unique 
        // names by appending a count suffix when duplicates are found.
        nodePaths.resize(model.partNodes.size());
        // pre-order guarantees parent path is always assigned before we 
        // reach any of its children or Usd will omplain about missing 
        // parent prims when we try to define them

        nodePaths[0] = assemblyPath;
        for (size_t i = 1; i < model.partNodes.size(); i++) {
            TfErrorMark mark;
            const StepModel::PartNode& node = model.partNodes[i];

            SdfPath parentPath;
            if (model.partNodes[i].parentIdx == 0) {
                parentPath = assemblyPath;
            } else {
                parentPath = nodePaths[model.partNodes[i].parentIdx];
            }

            int count = nameCounts[node.name]++;
            std::string finalName = sanitizeUsdName(node.name, count);

            if (parentPath.IsEmpty()) continue;

            nodePaths[i] = parentPath.AppendChild(TfToken(finalName));
            if (!mark.IsClean()) {
                for (const auto& error : mark) std::cerr << "Usd Error: " << error.GetCommentary() << "\n";
            }
        }
    }

    fs::path assemblyStageFilePath = rootPath / (model.stepPath.stem().string() + "-assembly.usdc");

    // Write prototype stages
    for (const auto& proto : prototypes) {

        if (!proto.variantSetName.empty()) {
            UsdVariantSet varSet = containerPrim.GetVariantSet(proto.variantSetName);
            varSet.SetVariantSelection(proto.variantName);
        }

        const UsdPrim& prototypesPrim = containerStage->GetPrimAtPath(prototypesInContainerPath);

        if (!prototypesPrim.IsValid()) {
            LOG_WARN("Invalid prototypes prim at: " + prototypesInContainerPath.GetString());
            return false;
        }

        writeCadPart(proto.stage, prototypesPrim, containerPrimPath, SdfPath("/CADPart"));

        LabelMap<SdfPath> variantPrototypePaths; // fresh per variant
        writePrototypeXformsInPrototypesStage(
            proto.stage,
            defs,
            prototypesPath,
            selectedPaths,
            containerPrimPath,
            proto.variantSetName,
            proto.variantName,
            model.definitionNames,
            variantPrototypePaths,
            proto.makeFreshStage
        );
        prototypePaths.insert(variantPrototypePaths.begin(), variantPrototypePaths.end());

        fs::path rootRelativeFilePath = fs::relative(rootStageFilePath, proto.filePath.parent_path());

        addStageSubLayer(proto.stage, rootRelativeFilePath);

        proto.stage->Save();
    }

    // Assembly stage setup
    bool assemblyNeedsUnitReset = stageNeedsUnitReset(assemblyStageFilePath, outputMetersPerUnit);

    if (assemblyNeedsUnitReset) {
        LOG_INFO("Resetting assembly stage due to metersPerUnit mismatch: " + assemblyStageFilePath.string());
    }

    bool isAssemblyInFilter = isAssemblyActiveInFilter(selectedPaths, containerPrimPath, assemblyPath);

    if (!isAssemblyInFilter) {
        LOG_DEBUG("Assembly stage will be skipped in generation because it is not targeted by selectedPaths.");
    }

    bool shouldCreateAssembly = assemblyNeedsUnitReset || isAssemblyInFilter;

    UsdStageRefPtr assemblyStage = initUsdStage(assemblyStageFilePath, shouldCreateAssembly);

    if (!assemblyStage) {
        LOG_WARN("Failed to initialize assembly stage: " +
                 assemblyStageFilePath.string());
        return false;
    }

    UsdGeomSetStageMetersPerUnit(assemblyStage, outputMetersPerUnit);

    assemblyStage->SetDefaultPrim(
        assemblyStage->OverridePrim(containerPrimPath)
    );

    assemblyStage->GetRootLayer()->SetDocumentation(
        "Auto generated file that contains the assembly hierarchy with empty overs for prototypes that are authored in a parent layer"
    );

    if (shouldCreateAssembly) {
        // Write a bunch of empty `over` so references in the 
        // assembly stage can resolve 
        writePrototypeOverridesInAssemblyStage(
            assemblyStage,
            prototypePaths
        );

        writeAssemblyXforms(
            assemblyStage,
            containerPrimPath,
            model.partNodes,
            nodePaths,
            prototypePaths,
            sourceToOutputScale
        );
    }
    // Add the assembly as a sublayer of the root layer, so it composes under the prototypes stage.

    fs::path assemblyRelativeFilePath = fs::relative(
        assemblyStageFilePath,
        rootStageFilePath.parent_path()
    );

    addStageSubLayer(rootStage, assemblyRelativeFilePath);

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
    UsdStageRefPtr containerStage,
    UsdPrim& containerPrim,
    const std::unordered_set<SdfPath, SdfPath::Hash> selectedInContainerPaths
) {
    LOG_SCOPED_TIMER("UsdStepExporter::populateUsd");
    TfErrorMark mark;
    
    AutolibStepFileContainer container(containerPrim);

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

    const StepModel& model = iter->second;

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

    SdfPath assemblyInContainerPath = SdfPath("/Assembly").ReplacePrefix(SdfPath::AbsoluteRootPath(), containerPrim.GetPath());
    SdfPath prototypesInContainerPath = SdfPath("/Prototypes").ReplacePrefix(SdfPath::AbsoluteRootPath(), containerPrim.GetPath());

    SdfPath containerParentPath = containerPrim.GetPath().GetParentPath();
    SdfPath containerPrimPath = containerPrim.GetPath().ReplacePrefix(containerParentPath, SdfPath::AbsoluteRootPath());
    SdfPath assemblyPath = SdfPath("/Assembly").ReplacePrefix(SdfPath::AbsoluteRootPath(), containerPrimPath);
    SdfPath prototypesPath = SdfPath("/Prototypes").ReplacePrefix(SdfPath::AbsoluteRootPath(), containerPrimPath);

    // Normalize paths from container-space to raw-space
    auto reparentPaths = [&](const std::unordered_set<SdfPath, SdfPath::Hash>& paths) -> std::unordered_set<SdfPath, SdfPath::Hash> {
        std::unordered_set<SdfPath, SdfPath::Hash> reparented;
        if (containerParentPath == SdfPath::AbsoluteRootPath()) {
            return paths;
        }
        for (const SdfPath& p : paths) {
            if (p.HasPrefix(containerParentPath)) {
                reparented.insert(p.ReplacePrefix(containerParentPath, SdfPath::AbsoluteRootPath()));
            } else {
                reparented.insert(p);
            }
        }
        return reparented;
    };

    std::unordered_set<SdfPath, SdfPath::Hash> containerVariantPaths = reparentPaths(getVariantsOnPrim(containerPrim));
    std::unordered_set<SdfPath, SdfPath::Hash> selectedPaths = reparentPaths(selectedInContainerPaths);

    for (const auto& path : selectedPaths) {
        std::cout << path.GetString() << std::endl;
    }

    std::unordered_map<SdfPath, StageFilterInfo, SdfPath::Hash> stageFilterMap;
    resolveStageFilterInfo(selectedPaths, containerPrimPath, prototypesPath, stageFilterMap);

    // Create the root layer for the container stage
    std::string baseName = model.stepPath.stem().string();
    fs::path rootPath = containerFilePath / (baseName);
    fs::path rootStageFilePath = rootPath / (baseName + "-container.usda");

    bool rootPathExists = fs::exists(rootPath);
    bool rootFileExists = fs::exists(rootStageFilePath);

    if (!fs::exists(rootPath)) {
        LOG_INFO("Creating output directory: " + rootPath.string());
        fs::create_directory(rootPath);
    }

    bool shouldMakeNewRootStage = !rootFileExists;
    UsdStageRefPtr rootStage = initUsdStage(rootStageFilePath, shouldMakeNewRootStage);
    UsdGeomSetStageMetersPerUnit(rootStage, outputMetersPerUnit);
    rootStage->GetRootLayer()->SetDocumentation("This is a sandwich layer that can contain overrides on prototypes before it is loaded in the container stage");

    // Setup Prototype Stages for all variants
    std::vector<PrototypeContainer> prototypes;
    populatePrototypeContainers(
        containerVariantPaths,
        selectedPaths,
        rootPath,
        baseName,
        prototypesPath,
        prototypesInContainerPath,
        containerPrimPath,
        assemblyPath,
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

    LabelMap<SdfPath> prototypePaths;
    buildPrototypeAndAssemblyStages(
        model,
        prototypes,
        selectedPaths,
        assemblyPath,
        containerPrimPath,
        prototypesPath,
        prototypesInContainerPath,
        containerStage,
        containerPrim,
        rootStage,
        rootStageFilePath,
        rootPath,
        outputMetersPerUnit,
        sourceToOutputScale,
        prototypePaths,
        defs
    );
    
    // Payload logic on container stage
    for (const auto& proto : prototypes) {
        std::string payloadPath = fs::relative(proto.filePath, containerFilePath).string();
        
        if (!proto.variantSetName.empty()) {
            UsdVariantSet varSet = containerPrim.GetVariantSet(proto.variantSetName);
            varSet.SetVariantSelection(proto.variantName);
            UsdEditContext ctx(varSet.GetVariantEditContext());
            containerPrim.GetPayloads().AddPayload(SdfPayload(payloadPath));
        } else {
            containerPrim.GetPayloads().AddPayload(SdfPayload(payloadPath));
        }
    }

    // Tessellation Jobs
    std::vector<TessellationJob> tessJobs;
    populateTessellationJobs(
        prototypes,
        containerStage,
        containerPrim,
        prototypesInContainerPath,
        prototypesPath,
        defs,
        prototypePaths,
        sourceToOutputScale,
        tessJobs
    );

    // Tessellation  
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

    LOG_DEBUG("Deactivating original prototype container in container stage: " + prototypesPath.GetString());

    if (!mark.IsClean()) {
        for (const auto& error : mark) std::cerr << "Usd: " << error.GetCommentary() << "\n";
    }
}