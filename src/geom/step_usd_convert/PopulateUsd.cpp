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
        
        // Restore to "default" variant after gathering permutations
        if (hasDefault) {
            vset.SetVariantSelection("default");
        } else {
            vset.ClearVariantSelection();
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

bool UsdStepExporter::isPrototypeActiveInFilter(
    const std::unordered_set<SdfPath, SdfPath::Hash>& filterPaths,
    const SdfPath& rootPrimPath,
    const std::string& variantSetName,
    const std::string& variantName,
    const SdfPath& prototypePath
) {
    if (filterPaths.empty()) 
        return true;

    // Build the absolute path to the Prototypes scope for this stage
    SdfPath baseStageKey = rootPrimPath;
    if (!variantSetName.empty()) {
        baseStageKey = baseStageKey.AppendVariantSelection(variantSetName, variantName);
    }
    SdfPath prototypesKey = baseStageKey.AppendChild(TfToken("Prototypes"));

    // If the entire variant stage is filtered, evaluate everything in it
    if (filterPaths.count(prototypesKey)) 
        return true;

    // If the base Prototypes scope is filtered, evaluate everything in all stages
    SdfPath basePrototypesKey = rootPrimPath.AppendChild(TfToken("Prototypes"));
    if (filterPaths.count(basePrototypesKey)) 
        return true;

    SdfPath absProtoPath = prototypePath.ReplacePrefix(SdfPath::AbsoluteRootPath().AppendChild(TfToken("Prototypes")), prototypesKey);
    
    // if a specific variant of the prim is requested
    if (filterPaths.count(absProtoPath)) 
        return true;

    // if the prim is requested
    SdfPath absProtoPathClean = absProtoPath.StripAllVariantSelections();
    if (filterPaths.count(absProtoPathClean)) 
        return true;

    return false;
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

static std::unordered_set<SdfPath, SdfPath::Hash> getVariantsOnPrim(
    const UsdPrim& prim
) {
    std::vector<std::string> variantSetNames;
    prim.GetVariantSets().GetNames(&variantSetNames);

    std::unordered_set<SdfPath, SdfPath::Hash> variantPaths;

    for (const std::string& vsetName : variantSetNames) {
        UsdVariantSet vset = prim.GetVariantSet(vsetName);
        std::vector<std::string> variantNames = vset.GetVariantNames();
        for (const std::string& variant : variantNames) {
            SdfPath variantPath = prim.GetPath().AppendVariantSelection(vsetName, variant);
            variantPaths.insert(variantPath);
        }
    }
    return variantPaths;
}

static void printVariants(
    const std::string& primLabel,
    const std::unordered_set<SdfPath, SdfPath::Hash>& variantPaths,
    const std::unordered_set<SdfPath, SdfPath::Hash> filterPaths,
    std::function<bool(const SdfPath&)> isFiltered
) {
    if (variantPaths.empty()) {
        std::cout << "No variants found on prim " << primLabel << ".\n";
        return;
    } else {
        std::cout << "Variants found on prim " << primLabel << ":\n";
        for (const SdfPath& path : variantPaths) {
            std::string filterMarker = isFiltered(path) ? " [FILTERED]" : "";
            std::cout << "  " << path.GetString() << filterMarker << "\n";
        }
    }
}

static void printFilteredPrototypes(
    UsdStageRefPtr rootStage,
    const std::unordered_set<SdfPath, SdfPath::Hash>& filterPaths
) {
    for (const SdfPath& filterPath : filterPaths) {
        std::vector<SdfPath> prefixes = filterPath.GetPrefixes();
        for (const SdfPath& prefix : prefixes) {
            if (prefix.IsPrimVariantSelectionPath()) {
                std::pair<std::string, std::string> vsel = prefix.GetVariantSelection();
                if (!vsel.first.empty()) {
                    SdfPath primPath = prefix.StripAllVariantSelections();
                    UsdPrim prim = rootStage->GetPrimAtPath(primPath);
                    if (prim.IsValid()) {
                        if (!prim.HasVariantSets() || !prim.GetVariantSets().HasVariantSet(vsel.first)) {
                            std::cerr << "Warning: prim filter requests variant set '" << vsel.first 
                                      << "' which does not exist on prim " << primPath.GetString() << "\n";
                        } else {
                            UsdVariantSet vset = prim.GetVariantSet(vsel.first);
                            std::vector<std::string> vnames = vset.GetVariantNames();
                            bool found = false;
                            for (const auto& vn : vnames) {
                                if (vn == vsel.second) { found = true; break; }
                            }
                            if (!found) {
                                std::cerr << "Warning: prim filter requests variant '" << vsel.second 
                                          << "' which does not exist in variant set '" << vsel.first 
                                          << "' on prim " << primPath.GetString() << "\n";
                            }
                        }
                    } else {
                        // SdfPath API stripping handles the leaf variant selections but doesn't resolve nested variants.
                        // We can ignore the missing prim here because USD payload logic hasn't instanced everything.
                        // Skip printing error for now, because it's generating false positives on the sub variants.
                        // Also, when variants are inside payloads, they might not be composed at the exact time
                        // we're asking without forcing full evaluations which OpenUSD discourages here.
                    }
                }
            }
        }
    }
}

// Stage Filtering
struct StageFilterInfo {
    bool makeFresh = false; // whole stage targeted
    bool hasSpecificPrototypes = false; // individual prototypes targeted
};

static void resolveStageFilterInfo(
    const SdfPath& rootPrimPath,
    const SdfPath& prototypesInRootPath,
    const std::unordered_set<SdfPath, SdfPath::Hash>& filterPaths,
    std::unordered_map<SdfPath, StageFilterInfo, SdfPath::Hash>& stageFilterMap
) {
    for (const SdfPath& filterPath : filterPaths) {
        const SdfPath cleanPath = filterPath.StripAllVariantSelections();

        // Find the most specific prefix of filterPath 
        // whose clean form == prototypesInRootPath.
        SdfPath stageKey;
        for (const SdfPath& prefix : filterPath.GetPrefixes()) {
            if (prefix.StripAllVariantSelections() == prototypesInRootPath)
                stageKey = prefix; // keep updating
        }
        if (filterPath.StripAllVariantSelections() == prototypesInRootPath)
            stageKey = filterPath;

        if (stageKey.IsEmpty()) {
            // filterPath is at or above rootPrimPath
            // Derive the stage key by finding the rootPrimPath-level prefix with its
            // variant selections, then appending /Prototypes.
            SdfPath rootLevelKey;
            for (const SdfPath& prefix : filterPath.GetPrefixes()) {
                if (prefix.StripAllVariantSelections() == rootPrimPath)
                    rootLevelKey = prefix;
            }
            if (filterPath.StripAllVariantSelections() == rootPrimPath)
                rootLevelKey = filterPath;

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
            std::cerr << "Warning: filterPaths targets both the entire prototype stage and specific " 
                    << "prototypes within it for stage [" << stageKey
                    << "]. The whole stage will be rebuilt regardless.\n";
        }
    }

}

void UsdStepExporter::populateUsd(
    const StepModel& model, 
    UsdStageRefPtr rootStage,
    UsdPrim& rootPrim,
    const std::unordered_set<SdfPath, SdfPath::Hash> filterPaths
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

    std::unordered_set<SdfPath, SdfPath::Hash> rootVariantPaths = getVariantsOnPrim(rootPrim);
    printVariants("root", rootVariantPaths, filterPaths, [&](const SdfPath& p) {
        return !filterPaths.empty() && filterPaths.count(p);
    });

    {
        // Clear existing payloads from the root stage beforehand to prevent OpenUSD core crashes
        // and noisy warnings when the prototype layers are modified later on disk.
        SdfChangeBlock block;

        if (filterPaths.count(prototypesInRootPath)) {
            std::cerr << "Warning: filterPaths contains /Prototypes root. whole prototypes hierarchy will be rebuilt.\n";
        }

        if (rootVariantPaths.empty()) {
            if (UsdPrim p = rootStage->GetPrimAtPath(prototypesInRootPath))
                p.GetPayloads().ClearPayloads();
        } else {
            for (const SdfPath& path : rootVariantPaths) {
                if (!filterPaths.empty() && filterPaths.find(path) != filterPaths.end()) {
                    continue; // only clear payloads that are bing re tesselated
                }
                std::pair<std::string, std::string> variantSelection = path.GetVariantSelection();
                const std::string& variantSetName = variantSelection.first;
                const std::string& variantName = variantSelection.second;

                UsdVariantSet varSet = rootPrim.GetVariantSet(variantSetName);

                varSet.SetVariantSelection(variantName);
                UsdEditContext ctx(varSet.GetVariantEditContext());
                UsdPrim prototypesPrim = rootStage->OverridePrim(prototypesInRootPath);
                prototypesPrim.GetPayloads().ClearPayloads();
            }
        }
    }

    std::unordered_map<SdfPath, StageFilterInfo, SdfPath::Hash> stageFilterMap;
    resolveStageFilterInfo(rootPrimPath, prototypesInRootPath, filterPaths, stageFilterMap);

    // Look up whether a given (variantSetName, variantName) stage should be built fresh.
    auto getStageMakeFresh = [&](const std::string& variantSetName, const std::string& variantName) -> bool {
        if (filterPaths.empty()) return true;

        SdfPath stageKey;
        
        if (variantSetName.empty()) {
            stageKey = prototypesInRootPath;
        } else {
            SdfPath rootPrimVariantPath = rootPrimPath.AppendVariantSelection(variantSetName, variantName);
            stageKey = rootPrimVariantPath.AppendChild(TfToken("Prototypes"));
        }

        auto it = stageFilterMap.find(stageKey);
        if (it == stageFilterMap.end()) return false; // stage not referenced at all
        return it->second.makeFresh;
    };

    // Setup Prototype Stages for all variants
    std::vector<PrototypeContainer> prototypes;

    if (rootVariantPaths.empty()) {
        fs::path prototypesStageFilePath = rootFilePath / (baseName + "-prototypes.usda");
        bool makeFreshStage = getStageMakeFresh("", "");

        UsdStageRefPtr prototypesStage =
            UsdStepExporter::initUsdStage(prototypesStageFilePath, rootPrimPath, makeFreshStage);

        UsdPrim existingPrototypesRoot = prototypesStage->GetPrimAtPath(prototypesPath);
        if (existingPrototypesRoot.IsValid() && !existingPrototypesRoot.IsActive())
            existingPrototypesRoot.SetActive(true);

        prototypesStage->SetMetadata(TfToken("metersPerUnit"), model.metersPerUnit);
        UsdGeomScope prototypesScope = UsdGeomScope::Define(prototypesStage, prototypesPath);
        prototypesStage->SetDefaultPrim(prototypesScope.GetPrim());
        prototypesStage->Save();

        prototypes.push_back({"", "", prototypesStageFilePath, prototypesStage});

    } else {
        baseName += "-";
        for (const SdfPath& variantPath : rootVariantPaths) {
            if (!filterPaths.empty() && filterPaths.find(variantPath) == filterPaths.end()) {
                continue; // skip variants not in filterPaths when filterPaths is non-empty
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

            fs::path prototypesStageFilePath = variantSubPath / (baseName + variantSetName + "-" + variantName + "-prototypes.usda");
            bool makeFreshStage = getStageMakeFresh(variantSetName, variantName);

            UsdStageRefPtr prototypesStage = UsdStepExporter::initUsdStage(prototypesStageFilePath, rootPrimPath, makeFreshStage);

            UsdPrim existingPrototypesRoot = prototypesStage->GetPrimAtPath(prototypesPath);
            if (existingPrototypesRoot.IsValid() && !existingPrototypesRoot.IsActive())
                existingPrototypesRoot.SetActive(true);

            prototypesStage->SetMetadata(TfToken("metersPerUnit"), model.metersPerUnit);
            UsdGeomScope prototypesScope = UsdGeomScope::Define(prototypesStage, prototypesPath);
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
    UsdGeomScope::Define(assemblyStage, assemblyInRootPath);

    SdfLayerHandle rootLayer = rootStage->GetRootLayer();
    std::string assemblyRelativeFilePath = fs::relative(assemblyStageFilePath, rootFilePath).string();
    
    // Write Xforms
    std::vector<std::pair<TDF_Label, TopoDS_Shape>> defs(model.definitionShapes.begin(), model.definitionShapes.end());
    LabelMap<SdfPath> prototypePaths;

    for (const auto& proto : prototypes) {
        writeCadPart(proto.stage, rootPrim, SdfPath("/CADPart"));
        bool makeFreshStage = getStageMakeFresh(proto.variantSetName, proto.variantName);

        writePrototypeXformsInPrototypesStage(
            proto.stage, 
            rootPrim, 
            defs, 
            prototypesPath, 
            filterPaths, 
            rootPrimPath, 
            proto.variantSetName, 
            proto.variantName, 
            prototypePaths, 
            makeFreshStage
        );
        proto.stage->Save();
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
    rootStage->Load(rootPrim.GetPath()); // Ensure payloads are loaded for variant discovery!

    printFilteredPrototypes(rootStage, filterPaths);

    // Flatten Tessellation Jobs
    std::vector<TessellationJob> tessJobs;
    for (const auto& proto : prototypes) {
        if (!proto.variantSetName.empty()) {
            UsdVariantSet varSet = rootPrim.GetVariantSet(proto.variantSetName);
            varSet.SetVariantSelection(proto.variantName);
        }
        
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

                printVariants(logLabel, childVariantPaths, filterPaths, [&](const SdfPath& p) {
                    return !filterPaths.empty() && !isPrototypeActiveInFilter(filterPaths, rootPrimPath, proto.variantSetName, proto.variantName, p);
                });
            }
        }

        protoRootPrim.SetActive(initialValue);

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
                    
                    // std::cout << "DEBUG: Queueing job for " << jobProtoPath << " (defIndex " << i << ")\n";
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
                bool bTesselate = isPrototypeActiveInFilter(filterPaths, rootPrimPath, job.proto->variantSetName, job.proto->variantName, job.prototypePath);
                
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
        
        writePrototypeGeometries(proto.stage, geomJobs, filterPaths, rootPrimPath, proto.variantSetName, proto.variantName);
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