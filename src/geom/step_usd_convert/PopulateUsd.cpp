#include <iostream>
#include <optional>
#include <string>
#include <chrono>
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

std::map<SdfPath, TessParams> resolveParams(
    const UsdPrim& rootPrim, // prototypes
    const TessParams& defaultParams
) {
    bool initialValue = rootPrim.IsActive();
    rootPrim.SetActive(true);
    std::map<SdfPath, TessParams> results;

    for (const UsdPrim& prim : rootPrim.GetChildren()) {
        results[prim.GetPath()] = getTessParams(prim, defaultParams);
    }

    for (const auto& params : results) {
        std::cout << "Params for " << params.first << ":\n";
        std::cout << "  meshLinearDeflection: " << params.second.meshLinearDeflection << "\n";
        std::cout << "  meshAngularDeflection: " << params.second.meshAngularDeflection << "\n";
        std::cout << "  meshMinSize: " << params.second.meshMinSize << "\n";
        std::cout << "  wireframeDeflection: " << params.second.wireframeDeflection << "\n";
        std::cout << "  wireframeMode.type: " << static_cast<int>(params.second.wireframeMode.type) << "\n";
        std::cout << "  wireframeMode.sampling: " << static_cast<int>(params.second.wireframeMode.sampling) << "\n";
        std::cout << "  sketchDeflection: " << params.second.sketchDeflection << "\n";
        std::cout << "  sketchMode.type: " << static_cast<int>(params.second.sketchMode.type) << "\n";
        std::cout << "  sketchMode.sampling: " << static_cast<int>(params.second.sketchMode.sampling) << "\n";
        std::cout << "  renderPurposeThreshold: " << params.second.renderPurposeThreshold << "\n";
        std::cout << "  selfIntersectionThreshold: " << params.second.selfIntersectionThreshold << "\n";
        std::cout << "  maxNumberRemeshPasses: " << params.second.maxNumberRemeshPasses << "\n";
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

// All data needed to write one variant's geometry, collected during the
// sequential phases so the write phase can run in parallel.
struct VariantWork {
    const PrototypeContainer* proto = nullptr;
    TessParams rootParams;
    std::map<SdfPath, TessParams> paramsBank;
    std::vector<TessResult> tessResults;
};

void UsdStepExporter::populateUsd(
    const StepModel& model, 
    UsdStageRefPtr rootStage,
    UsdPrim& rootPrim, // on the stage with the stronger opinions
    const std::unordered_set<SdfPath, SdfPath::Hash>& prototypesFilter
) {
    TfErrorMark mark;

    fs::path prototypesStageFilePath = model.stepPath.parent_path() / (model.stepPath.stem().string() + "-prototypes.usda");
    fs::path assemblyStageFilePath = model.stepPath.parent_path() / (model.stepPath.stem().string() + "-assembly.usdc");
    
    SdfPath assemblyPath("/Assembly");
    SdfPath prototypesPath("/Prototypes");

    SdfPath rootPrimPath = rootPrim.GetPath();
    SdfPath assemblyInRootPath = rootPrimPath.AppendChild(TfToken("Assembly"));
    SdfPath prototypesInRootPath = rootPrimPath.AppendChild(TfToken("Prototypes"));

    bool makeFreshStage = prototypesFilter.empty();

    // Pass rootPrimPath as the default prim anchor
    UsdStageRefPtr prototypesStage = UsdStepExporter::initUsdStage(prototypesStageFilePath, rootPrimPath, makeFreshStage);
    if (!prototypesStage) {
        std::cerr << "Failed to initialize USD stage for " << prototypesStageFilePath << "\n";
        return;
    }
    UsdPrim existingPrototypesRoot = prototypesStage->GetPrimAtPath(prototypesPath);
    if (existingPrototypesRoot.IsValid() && !existingPrototypesRoot.IsActive()) {
        existingPrototypesRoot.SetActive(true);
    }

    UsdStageRefPtr assemblyStage = UsdStepExporter::initUsdStage(assemblyStageFilePath, rootPrimPath, makeFreshStage);
    if (!assemblyStage) {
        std::cerr << "Failed to initialize USD stage for " << assemblyStageFilePath << "\n";
        return;
    }

    rootPrim = rootStage->GetPrimAtPath(rootPrimPath);
    if (!rootPrim.IsValid()) {
        std::cerr << "Root prim invalid after stage init (shared layer recomposition)\n";
        return;
    }
    
    prototypesStage->SetMetadata(TfToken("metersPerUnit"), model.metersPerUnit);
    assemblyStage->SetMetadata(TfToken("metersPerUnit"), model.metersPerUnit);

    // Create def /Prototypes in prototypes stage
    UsdGeomScope prototypesScope = UsdGeomScope::Define(prototypesStage, prototypesPath);
    
    // Create over /(root_prim)/Assembly in assembly stage
    UsdPrim assemblyRoot = assemblyStage->OverridePrim(rootPrimPath);
    UsdGeomScope assemblyScope = UsdGeomScope::Define(assemblyStage, assemblyInRootPath);

    prototypesStage->SetDefaultPrim(prototypesScope.GetPrim());
    assemblyStage->SetDefaultPrim(assemblyRoot);

    SdfLayerHandle rootLayer = rootStage->GetRootLayer();
    std::string assemblyFileName = assemblyStageFilePath.filename().string();
    
    // Check if it's already there before adding
    SdfSubLayerProxy subLayers = rootLayer->GetSubLayerPaths();
    bool alreadyExists = false;
    for (const auto& path : subLayers) {
        if (path == assemblyFileName) {
            alreadyExists = true;
            break;
        }
    }

    if (!alreadyExists) {
        rootLayer->InsertSubLayerPath(assemblyFileName);
    }

    prototypesStage->Save();
    assemblyStage->Save();

    using Clock = std::chrono::high_resolution_clock;
    using Seconds = std::chrono::duration<double>;
    auto totalStart = Clock::now();

    std::vector<std::pair<TDF_Label, TopoDS_Shape>> defs(
        model.definitionShapes.begin(),
        model.definitionShapes.end()
    );

    // Write Xforms first so USD can resolve opinions that
    // will later be populated by geometry later 
    LabelMap<SdfPath> prototypePaths;

    writePrototypeXforms(
        prototypesStage, assemblyStage, rootPrim,
        defs, prototypesPath,
        prototypesFilter, makeFreshStage,
        prototypePaths
    );

    std::vector<SdfPath> nodePaths = computeNodePaths(model.partNodes, assemblyInRootPath);
    if (makeFreshStage)
        writeAssemblyXforms(assemblyStage, rootPrim.GetPrimPath(), model.partNodes,nodePaths, prototypePaths);

    // Save both stages before adding the payload so 
    // the root stage can see the full composed 
    // hierarchy on reload
    prototypesStage->Save();
    assemblyStage->Save();

    // Add the prototypes stage as the payload
    std::string payloadPath = prototypesStageFilePath.filename().string();
    SdfPrimSpecHandle protoSpec = rootStage->GetRootLayer()->GetPrimAtPath(prototypesInRootPath);

    bool payloadAlreadyAuthored = false;
    if (protoSpec) {
        for (const SdfPayload& p : protoSpec->GetPayloadList().GetAddedOrExplicitItems()) {
            if (p.GetAssetPath() == payloadPath) {
                payloadAlreadyAuthored = true;
                break;
            }
        }
    }

    UsdPrim prototypesPrimOnRoot = rootStage->OverridePrim(prototypesInRootPath);
    if (!payloadAlreadyAuthored) {
        prototypesPrimOnRoot.GetPayloads().AddPayload(payloadPath);
        rootStage->GetRootLayer()->Save();
        rootStage->Reload();
    } else {
        // Still need to reload to pick up freshly written prototypes geometry
        rootStage->Reload();
    }

    rootPrim = rootStage->GetPrimAtPath(rootPrimPath); 
    if (!rootPrim.IsValid()) {
        std::cerr << "Root prim became invalid after reload!\n";
        return;
    }

    // LoadNone means payloads don't load automatically — force load so
    // traversal below can walk /Wonderful/Assembly/... and /Wonderful/Prototypes/...
    rootStage->Load(rootPrim.GetPath());

    // Resolve tessellation params from the prototypes 
    // prim on the root stage so that per-prototype 
    // `over` blocks authored in the usda are visible 
    // and win over the root defaults
    UsdPrim prototypePrim = rootStage->GetPrimAtPath(prototypesInRootPath);

    TessParams rootParams = getTessParams(rootPrim);

    std::map<SdfPath, TessParams> paramsBank = resolveParams(prototypePrim, rootParams);
    std::vector<TessResult> tessResults(defs.size());
    std::string logName = "";

    tessellateGeometry(
        defs, prototypePaths, prototypesPath, prototypesInRootPath,
        logName, rootParams, tessResults, paramsBank, prototypesFilter
    );

    writePrototypeGeometries(
        prototypesStage, rootPrim.GetPrimPath(),
        defs, prototypePaths,
        tessResults, rootParams, paramsBank, prototypesFilter
    );

    // Hide /Prototypes from renderers via the root stage override,
    // not on the prototypes stage itself (which would break re-runs)
    UsdPrim prototypeRoot = rootStage->GetPrimAtPath(prototypesInRootPath);
    if (prototypeRoot.IsValid()) {
        prototypeRoot.SetActive(false);
        rootStage->GetRootLayer()->Save();
    }

    prototypesStage->Save();

    if (!mark.IsClean()) {
        for (const auto& error : mark)
            std::cerr << "Usd: " << error.GetCommentary() << "\n";
    }
}

void UsdStepExporter::populateUsdVariant(
    const StepModel& model, 
    UsdStageRefPtr rootStage,
    UsdPrim& rootPrim, // on the stage with the stronger opinions
    const std::map<std::string, std::vector<std::string>>& variantSetNameToVariantNames,
    const std::unordered_set<SdfPath, SdfPath::Hash>& prototypesFilter
) {
    TfErrorMark mark;

    std::string basePath = model.stepPath.parent_path() / (model.stepPath.stem().string() + "-");
    fs::path assemblyStageFilePath = model.stepPath.parent_path() / (model.stepPath.stem().string() + "-assembly.usdc");

    SdfPath prototypesPath("/Prototypes");

    SdfPath rootPrimPath = rootPrim.GetPath();
    SdfPath assemblyInRootPath = rootPrimPath.AppendChild(TfToken("Assembly"));
    SdfPath prototypesInRootPath = rootPrimPath.AppendChild(TfToken("Prototypes"));

    // Pass rootPrimPath as the default prim anchor
    UsdVariantSets rootVariantSets = rootPrim.GetVariantSets();
    std::vector<PrototypeContainer> prototypes;
    
    // if the user specifies a prototype to remesh
    // don't clean house with the ones that are already there
    bool makeFreshStage = prototypesFilter.empty();

    // Initialize Prototypes
    std::mutex protoMutex;
    WorkParallelForEach(variantSetNameToVariantNames.begin(), variantSetNameToVariantNames.end(), [&](const auto& entry) -> void {
        const std::string& set = entry.first;
        const std::vector<std::string>& rootVariantNames = entry.second;

        for (const auto& variant : rootVariantNames) {
            fs::path prototypesStageFilePath = basePath + set + "-" + variant + "-prototypes.usda";

            UsdStageRefPtr prototypesStage = UsdStepExporter::initUsdStage(prototypesStageFilePath, rootPrimPath, makeFreshStage);
            if (!prototypesStage) {
                std::cerr << "Failed to initialize USD stage for " << prototypesStageFilePath << "\n";
                return;
            }

            UsdPrim existingPrototypesRoot = prototypesStage->GetPrimAtPath(prototypesPath);
            if (existingPrototypesRoot.IsValid() && !existingPrototypesRoot.IsActive()) {
                existingPrototypesRoot.SetActive(true);
            }

            prototypesStage->SetMetadata(TfToken("metersPerUnit"), model.metersPerUnit);
            UsdGeomScope prototypesScope = UsdGeomScope::Define(prototypesStage, prototypesPath);
            prototypesStage->SetDefaultPrim(prototypesScope.GetPrim());

            prototypesStage->Save();

            PrototypeContainer container;
            container.variantSetName = set;
            container.variantName = variant;
            container.filePath = prototypesStageFilePath;
            container.stage = prototypesStage;
            {
                std::lock_guard<std::mutex> lock(protoMutex);
                prototypes.push_back(container);
            }
        }
    });

    // Initialize Assembly
    UsdStageRefPtr assemblyStage = UsdStepExporter::initUsdStage(assemblyStageFilePath, rootPrimPath, makeFreshStage);
    if (!assemblyStage) {
        std::cerr << "Failed to initialize USD stage for " << assemblyStageFilePath << "\n";
        return;
    }

    rootPrim = rootStage->GetPrimAtPath(rootPrimPath);
    if (!rootPrim.IsValid()) {
        std::cerr << "Root prim invalid after stage init (shared layer recomposition)\n";
        return;
    }

    assemblyStage->SetMetadata(TfToken("metersPerUnit"), model.metersPerUnit);

    UsdPrim assemblyRoot = assemblyStage->OverridePrim(rootPrimPath);
    UsdGeomScope::Define(assemblyStage, assemblyInRootPath);

    assemblyStage->SetDefaultPrim(assemblyRoot);

    SdfLayerHandle rootLayer = rootStage->GetRootLayer();
    std::string assemblyFileName = assemblyStageFilePath.filename().string();

    // Check if it's already there before adding
    SdfSubLayerProxy subLayers = rootLayer->GetSubLayerPaths();
    bool alreadyExists = false;
    for (const auto& path : subLayers) {
        if (path == assemblyFileName) {
            alreadyExists = true;
            break;
        }
    }

    if (!alreadyExists) {
        rootLayer->InsertSubLayerPath(assemblyFileName);
    }

    assemblyStage->Save();

    using Clock = std::chrono::high_resolution_clock;
    using Seconds = std::chrono::duration<double>;

    std::vector<std::pair<TDF_Label, TopoDS_Shape>> defs(
        model.definitionShapes.begin(),
        model.definitionShapes.end()
    );
    
    // Write Xforms first so USD can resolve opinions that
    // will later be populated by geometry later 
    LabelMap<SdfPath> prototypePaths;

    // Populate prototypePaths and prototypeInRootPaths, and write assembly overrides
    writePrototypeXforms(
        nullptr, assemblyStage,
        rootPrim, defs, prototypesPath,
        prototypesFilter, makeFreshStage,
        prototypePaths
    );

    // Write xforms into each variant's prototypes stage
    for (const auto& proto : prototypes) {
        writePrototypeXforms(
            proto.stage, nullptr,
            rootPrim, defs, prototypesPath,
            prototypesFilter, makeFreshStage,
            prototypePaths
        );
    }

    std::vector<SdfPath> nodePaths = computeNodePaths(model.partNodes, assemblyInRootPath);
    if (makeFreshStage)
        writeAssemblyXforms(assemblyStage, rootPrim.GetPrimPath(), model.partNodes, nodePaths, prototypePaths);

    // Save both stages before adding the payload so 
    // the root stage can see the full composed 
    // hierarchy on reload

    for (const auto& proto : prototypes) {
        proto.stage->Save();
    }
    assemblyStage->Save();

    // Add the prototypes stage as the payload

    for (const auto& proto : prototypes) {
        std::string payloadPath = proto.filePath.filename().string();

        UsdVariantSet varSet = rootPrim.GetVariantSet(proto.variantSetName);
        varSet.SetVariantSelection(proto.variantName);

        UsdEditContext ctx(varSet.GetVariantEditContext());
        UsdPrim prototypesPrimOnRoot = rootStage->OverridePrim(prototypesInRootPath);

        SdfPrimSpecHandle protoSpec = rootStage->GetEditTarget().GetPrimSpecForScenePath(prototypesInRootPath);

        bool payloadAlreadyAuthored = false;
        if (protoSpec) {
            for (const SdfPayload& p : protoSpec->GetPayloadList().GetAddedOrExplicitItems()) {
                if (p.GetAssetPath() == payloadPath) {
                    payloadAlreadyAuthored = true;
                    break;
                }
            }
        }

        if (!payloadAlreadyAuthored) {
            prototypesPrimOnRoot.GetPayloads().AddPayload(payloadPath);
        }
    }

    rootStage->GetRootLayer()->Save();
    // Still need to reload to pick up freshly written prototypes geometry
    rootStage->Reload();

    rootPrim = rootStage->GetPrimAtPath(rootPrimPath);
    if (!rootPrim.IsValid()) {
        std::cerr << "Root prim became invalid after reload!\n";
        return;
    }

    // LoadNone means payloads don't load automatically — force load so
    // traversal below can walk /Wonderful/Assembly/... and /Wonderful/Prototypes/...
    rootStage->Load(rootPrim.GetPath());

    // Resolve tessellation params from the prototypes 
    // prim on the root stage so that per-prototype 
    // `over` blocks authored in the usd are visible 
    // and win over the root defaults.

    // The variant loop is split into three sequential phases before the
    // parallel write so that thread-safety constraints are respected:

    std::vector<VariantWork> variantWork(prototypes.size());

    // Resolve params for every variant
    for (size_t vi = 0; vi < prototypes.size(); vi++) {
        const PrototypeContainer& proto = prototypes[vi];
        variantWork[vi].proto = &proto;

        UsdVariantSet varSet = rootPrim.GetVariantSet(proto.variantSetName);
        varSet.SetVariantSelection(proto.variantName);

        UsdPrim prototypePrim = rootStage->GetPrimAtPath(prototypesInRootPath);

        variantWork[vi].rootParams = getTessParams(rootPrim);

        TessParams variantLevelParams = getTessParams(prototypePrim, variantWork[vi].rootParams);

        variantWork[vi].paramsBank = resolveParams(prototypePrim, variantLevelParams);
        variantWork[vi].tessResults.resize(defs.size());
    }

    // Tessellate each variant sequentially
    for (VariantWork& work : variantWork) {
        std::string logLabel = "variant " + work.proto->variantSetName + ", " + work.proto->variantName;
        tessellateGeometry(
            defs, prototypePaths, prototypesPath, prototypesInRootPath,
            logLabel, work.rootParams, work.tessResults, work.paramsBank, prototypesFilter
        );
    }

    // Write geometry for all variants in parallel
    WorkParallelForEach(variantWork.begin(), variantWork.end(), [&](VariantWork& work) {
        writePrototypeGeometries(
            work.proto->stage, rootPrim.GetPrimPath(),
            defs, prototypePaths,
            work.tessResults, work.rootParams, work.paramsBank, prototypesFilter
        );
        work.proto->stage->Save();
    });

    // Hide /Prototypes from renderers via the root stage override,
    // not on the prototypes stage itself so that the geometry is still
    // visible when editing the prototypes stage directly.
    UsdPrim prototypeRoot = rootStage->GetPrimAtPath(prototypesInRootPath);
    if (prototypeRoot.IsValid()) {
        prototypeRoot.SetActive(false);
        rootStage->GetRootLayer()->Save();
    }

    if (!mark.IsClean()) {
        for (const auto& error : mark)
            std::cerr << "Usd: " << error.GetCommentary() << "\n";
    }
}