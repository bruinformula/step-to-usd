#include <string>

#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/usd/inherits.h>
#include <pxr/usd/usd/payloads.h>
#include <pxr/usd/usd/variantSets.h>
#include <pxr/usd/usd/editContext.h>

#include <pxr/usd/usdGeom/scope.h>

#include <pxr/base/work/loops.h>

#include <opencascade/OSD_Parallel.hxx>
#include <opencascade/TopExp_Explorer.hxx>

#include "stepTessellationAPI.h"
#include "tokens.h"

#include "UsdStepExporter.h"

PXR_NAMESPACE_USING_DIRECTIVE

template <typename T>
void updateIfAuthored(const UsdAttribute& attr, T* value, bool& primHasAnyValue) {
    bool hasValue = attr.HasValue();
    primHasAnyValue |= hasValue;
    if (hasValue) {
        attr.Get(value);
    }
}

// Helper to read a token attr and convert to CurveType
template <>
void updateIfAuthored(const UsdAttribute& attr, CurveSampling* value, bool& primHasAnyValue) {
    bool hasValue = attr.HasValue();
    primHasAnyValue |= hasValue;
    if (!hasValue) return;

    TfToken token;
    if (!attr.Get(&token)) {
        *value = CurveSampling::Underlying;
        return;
    }

    if (token == AutolibTokens->underlying) {
        *value = CurveSampling::Underlying;
        return;
    } else if (token == AutolibTokens->resampled) {
        *value = CurveSampling::Resampled;
        return;
    }
}

template <>
void updateIfAuthored(const UsdAttribute& attr, CurveType* value, bool& primHasAnyValue) {
    bool hasValue = attr.HasValue();
    primHasAnyValue |= hasValue;
    if (!hasValue) return;

    TfToken token;
    if (!attr.Get(&token)) {
        *value = CurveType::None;
        return;
    }

    if (token == AutolibTokens->none) {
        *value = CurveType::None;
        return;
    } else if (token == AutolibTokens->linear) {
        *value = CurveType::Linear;
        return;
    } else if (token == AutolibTokens->catmullRom) {
        *value = CurveType::CatmullRom;
        return;
    }
}

static void traverseForTessParams(
    UsdPrim prim, 
    TessParams currentParams,
    std::map<SdfPath, TessParams>& partNodes
) {
    AutolibStepTessellationAPI api(prim);

    bool hasAnyValue = false;
    updateIfAuthored(api.GetStepMeshLinearDeflectionAttr(), &currentParams.meshLinearDeflection, hasAnyValue);
    updateIfAuthored(api.GetStepMeshAngularDeflectionAttr(), &currentParams.meshAngularDeflection, hasAnyValue);
    updateIfAuthored(api.GetStepMeshMinSizeAttr(), &currentParams.meshMinSize, hasAnyValue);

    updateIfAuthored(api.GetStepWireframeDeflectionAttr(), &currentParams.wireframeDeflection, hasAnyValue);
    updateIfAuthored(api.GetStepWireframeTypeAttr(), &currentParams.wireframeMode.type, hasAnyValue);
    updateIfAuthored(api.GetStepWireframeSamplingAttr(), &currentParams.wireframeMode.sampling, hasAnyValue);

    updateIfAuthored(api.GetStepSketchDeflectionAttr(), &currentParams.sketchDeflection, hasAnyValue);
    updateIfAuthored(api.GetStepSketchTypeAttr(), &currentParams.sketchMode.type, hasAnyValue);
    updateIfAuthored(api.GetStepSketchSamplingAttr(), &currentParams.sketchMode.sampling, hasAnyValue);

    updateIfAuthored(api.GetStepRenderPurposeThresholdAttr(), &currentParams.renderPurposeThreshold, hasAnyValue);
    updateIfAuthored(api.GetStepSelfIntersectionThresholdAttr(), &currentParams.selfIntersectionThreshold, hasAnyValue);
    updateIfAuthored(api.GetStepMaxNumberRemeshPassesAttr(), &currentParams.maxNumberRemeshPasses, hasAnyValue);

    partNodes[prim.GetPath()] = currentParams;

    // Avoid parts that inherit from /CADPart
    for (const SdfPath& inheritPath : prim.GetInherits().GetAllDirectInherits()) {
        if (inheritPath == SdfPath("/CADPart")) {
            return;
        }
    }

    for (const auto& child : prim.GetFilteredChildren(UsdPrimAllPrimsPredicate)) {
        traverseForTessParams(child, currentParams, partNodes);
    }
}

std::map<SdfPath, TessParams> resolveParams(
    const UsdPrim& rootPrim,
    const TessParams& defaultParams
) {
    std::map<SdfPath, TessParams> results;
    traverseForTessParams(rootPrim, defaultParams, results);

    return results;
}

TessParams getTessParams(
    UsdPrim prim
) {
    AutolibStepTessellationAPI api(prim);

    TessParams params;

    bool hasAnyValue = false;
    updateIfAuthored(api.GetStepMeshLinearDeflectionAttr(), &params.meshLinearDeflection, hasAnyValue);
    updateIfAuthored(api.GetStepMeshAngularDeflectionAttr(), &params.meshAngularDeflection, hasAnyValue);
    updateIfAuthored(api.GetStepMeshMinSizeAttr(), &params.meshMinSize, hasAnyValue);

    updateIfAuthored(api.GetStepWireframeDeflectionAttr(), &params.wireframeDeflection, hasAnyValue);
    updateIfAuthored(api.GetStepWireframeTypeAttr(), &params.wireframeMode.type, hasAnyValue);
    updateIfAuthored(api.GetStepWireframeSamplingAttr(), &params.wireframeMode.sampling, hasAnyValue);

    updateIfAuthored(api.GetStepSketchDeflectionAttr(), &params.sketchDeflection, hasAnyValue);
    updateIfAuthored(api.GetStepSketchTypeAttr(), &params.sketchMode.type, hasAnyValue);
    updateIfAuthored(api.GetStepSketchSamplingAttr(), &params.sketchMode.sampling, hasAnyValue);

    updateIfAuthored(api.GetStepRenderPurposeThresholdAttr(), &params.renderPurposeThreshold, hasAnyValue);
    updateIfAuthored(api.GetStepSelfIntersectionThresholdAttr(), &params.selfIntersectionThreshold, hasAnyValue);
    updateIfAuthored(api.GetStepMaxNumberRemeshPassesAttr(), &params.maxNumberRemeshPasses, hasAnyValue);

    return params;
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
    UsdPrim& rootPrim // on the stage with the stronger opinions
) {
    TfErrorMark mark;

    fs::path prototypesStageFilePath = model.stepPath.parent_path() / (model.stepPath.stem().string() + "-prototypes.usdc");
    fs::path assemblyStageFilePath = model.stepPath.parent_path() / (model.stepPath.stem().string() + "-assembly.usdc");
    
    SdfPath assemblyPath("/Assembly");
    SdfPath prototypesPath("/Prototypes");

    SdfPath rootPrimPath = rootPrim.GetPath();
    SdfPath assemblyInRootPath = rootPrimPath.AppendChild(TfToken("Assembly"));
    SdfPath prototypesInRootPath = rootPrimPath.AppendChild(TfToken("Prototypes"));

    // Pass rootPrimPath as the default prim anchor
    UsdStageRefPtr prototypesStage = UsdStepExporter::initUsdStage(prototypesStageFilePath, rootPrimPath, true);
    if (!prototypesStage) {
        std::cerr << "Failed to initialize USD stage for " << prototypesStageFilePath << "\n";
        return;
    }

    UsdPrim existingPrototypesRoot = prototypesStage->GetPrimAtPath(prototypesPath);
    if (existingPrototypesRoot.IsValid() && !existingPrototypesRoot.IsActive()) {
        existingPrototypesRoot.SetActive(true);
    }

    UsdStageRefPtr assemblyStage = UsdStepExporter::initUsdStage(assemblyStageFilePath, rootPrimPath);
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

    // Create /Wonderful in both files, and nest the scopes underneath
    UsdGeomScope prototypesScope = UsdGeomScope::Define(prototypesStage, prototypesPath);
    
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

    auto tessStart = Clock::now();
    std::atomic<int> tessCompleted(0);
    const int total = (int)defs.size();

    // Write Xforms first so USD can resolve opinions that
    // will later be populated by geometry later 
    LabelMap<SdfPath> prototypePaths;
    LabelMap<SdfPath> prototypeInAssemblyPaths;

    std::unordered_map<std::string, int> protoNameCounts;
    const int protoTotal = (int)defs.size();
    int protoCompleted = 0;

    for (int i = 0; i < protoTotal; i++) {
        std::string rawName = getLabelName(defs[i].first);
        if (rawName.empty()) {
            rawName = "Def_" + std::to_string(i);
        }
        int protoCount = protoNameCounts[rawName]++;
        std::string name = sanitizeUsdName(rawName, protoCount);
        SdfPath protoPath = prototypesPath.AppendChild(TfToken(name));

        if (!writePrototypeXform(prototypesStage, protoPath, i)) {
            std::cerr << "\r[" << ++protoCompleted << "/" << protoTotal << "] Writing prototypes..." << std::flush;
            continue;
        }

        SdfPath assemblyProtoPath = prototypesInRootPath.AppendChild(TfToken(name));

        assemblyStage->OverridePrim(assemblyProtoPath);

        prototypePaths[defs[i].first] = protoPath;
        prototypeInAssemblyPaths[defs[i].first] = assemblyProtoPath;
        //std::cout << "  Prototype " << protoPath << " -> " << r.points.size() << " verts\n";
        std::cerr << "\r[" << ++protoCompleted << "/" << protoTotal << "] Writing prototypes..." << std::flush;
    }

    std::vector<SdfPath> nodePaths = computeNodePaths(model.partNodes, assemblyInRootPath);
    writeAssemblyXforms(model.partNodes, assemblyStage, nodePaths, prototypeInAssemblyPaths);

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
    
    OSD_Parallel::For(0, total, [&](int i) {
        const TopoDS_Shape& defShape = defs[i].second;
        SdfPath protoPath = prototypePaths[defs[i].first];

        // Translate /Prototypes/plate0 -> /Wonderful/Prototypes/plate0
        // so it matches the keys in paramsBank which come from the root stage
        SdfPath protoPathInRoot = protoPath.IsEmpty() ? SdfPath() : prototypesInRootPath.AppendPath(protoPath.MakeRelativePath(prototypesPath));

        const TessParams& params = paramsBank.count(protoPathInRoot) ? paramsBank.at(protoPathInRoot) : rootParams;
        
        if (defShape.IsNull()) {
            int done = ++tessCompleted;
            std::cerr << "\r[" << done << "/" << total << "] Tessellating..." << std::flush;
            return;
        }

        auto partStart = Clock::now();
        std::string partName = getLabelName(defs[i].first);

        try {
            TessResult result;
            if (tesselatePart(result, defShape, params))
                tessResults[i] = std::move(result);
        } catch (const Standard_Failure& e) {
            std::cerr << "\nOCC exception during tessellation of def " << i << ": " << e.GetMessageString() << "\n";
        }

        double elapsed = Seconds(Clock::now() - partStart).count();
        if (elapsed > 10.0) {
            std::cerr << "\n[SLOW] def " << i << " (" << partName << ")"
                    << "  time=" << elapsed << "s"
                    << "  faces=" << [&]{ int n=0; for(TopExp_Explorer e(defShape,TopAbs_FACE);e.More();e.Next()) n++; return n; }()
                    << "\n";
        }

        int done = ++tessCompleted;
        std::cerr << "\r[" << done << "/" << total << "] Tessellating..." << std::flush;
    });

    std::cerr << "\n";
    
    protoCompleted = 0;
    for (int i = 0; i < protoTotal; i++) {
        SdfPath protoPath = prototypePaths[defs[i].first];
        SdfPath protoPathInRoot = protoPath.IsEmpty() ? SdfPath() : prototypesInRootPath.AppendPath(protoPath.MakeRelativePath(prototypesPath));

        const TessParams& params = paramsBank.count(protoPathInRoot) ? paramsBank.at(protoPathInRoot) : rootParams;
        const TessResult& r = tessResults[i];

        if (!writePrototypeGeometry(prototypesStage, protoPath, r, params, i)) {
            std::cerr << "\r[" << ++protoCompleted << "/" << protoTotal << "] Writing prototypes..." << std::flush;
            continue;
        }

        prototypePaths[defs[i].first] = protoPath;
        //std::cout << "  Prototype " << protoPath << " -> " << r.points.size() << " verts\n";
        std::cerr << "\r[" << ++protoCompleted << "/" << protoTotal << "] Writing prototypes..." << std::flush;
    }
    std::cerr << "\n";

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
    const std::map<std::string, std::vector<std::string>>& variantSetNameToVariantNames
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

    // Initialize Prototypes
    std::mutex protoMutex;
    WorkParallelForEach(variantSetNameToVariantNames.begin(), variantSetNameToVariantNames.end(), [&](const auto& entry) -> void {
        const std::string& set = entry.first;
        const std::vector<std::string>& rootVariantNames = entry.second;

        for (const auto& variant : rootVariantNames) {
            fs::path prototypesStageFilePath = basePath + set + "-" + variant + "-prototypes.usdc";

            UsdStageRefPtr prototypesStage = UsdStepExporter::initUsdStage(prototypesStageFilePath, rootPrimPath, true);
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
    UsdStageRefPtr assemblyStage = UsdStepExporter::initUsdStage(assemblyStageFilePath, rootPrimPath);
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

    const int total = (int)defs.size();

    // Write Xforms first so USD can resolve opinions that
    // will later be populated by geometry later 
    LabelMap<SdfPath> prototypePaths;
    LabelMap<SdfPath> prototypeInAssemblyPaths;

    std::unordered_map<std::string, int> protoNameCounts;
    const int protoTotal = (int)defs.size();
    int protoCompleted = 0;

    for (int i = 0; i < protoTotal; i++) {
        std::string rawName = getLabelName(defs[i].first);
        if (rawName.empty()) {
            rawName = "Def_" + std::to_string(i);
        }
        int protoCount = protoNameCounts[rawName]++;
        std::string name = sanitizeUsdName(rawName, protoCount);
        SdfPath protoPath = prototypesPath.AppendChild(TfToken(name));
        SdfPath assemblyProtoPath = prototypesInRootPath.AppendChild(TfToken(name));

        prototypePaths[defs[i].first] = protoPath;
        prototypeInAssemblyPaths[defs[i].first] = assemblyProtoPath;

        assemblyStage->OverridePrim(assemblyProtoPath);

        for (const auto& proto : prototypes) {
            if (!writePrototypeXform(proto.stage, protoPath, i)) {
                continue;
            }
        }
        std::cerr << "\r[" << ++protoCompleted << "/" << protoTotal << "] Writing prototypes..." << std::flush;
    }

    std::vector<SdfPath> nodePaths = computeNodePaths(model.partNodes, assemblyInRootPath);
    writeAssemblyXforms(model.partNodes, assemblyStage, nodePaths, prototypeInAssemblyPaths);

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
        variantWork[vi].paramsBank = resolveParams(prototypePrim, variantWork[vi].rootParams);
        variantWork[vi].tessResults.resize(defs.size());
    }

    // Tessellate each variant
    for (VariantWork& work : variantWork) {
        const TessParams& rootParams = work.rootParams;

        // /Wonderful/Prototypes is inactive in the root layer so USD does not
        // compose its children — paramsBank will only ever contain the parent
        // path itself. Walk up from the prototype path to the nearest ancestor
        // that is in the bank before falling back to rootParams. This means
        // params authored on /Wonderful/Prototypes propagate to all prototypes,
        // and any future per-prototype `over` blocks will still win because the
        // exact path hits first.
        auto findParams = [&](const SdfPath& path) -> const TessParams& {
            SdfPath p = path;
            while (!p.IsEmpty() && p != SdfPath::AbsoluteRootPath()) {
                auto it = work.paramsBank.find(p);
                if (it != work.paramsBank.end()) return it->second;
                p = p.GetParentPath();
            }
            return rootParams;
        };

        std::atomic<int> tessCompleted(0);

        OSD_Parallel::For(0, total, [&](int i) {
            const TopoDS_Shape& defShape = defs[i].second;
            SdfPath protoPath = prototypePaths[defs[i].first];

            // Change the prim references from 
            // /Prototypes/plate0 -> /Wonderful/Prototypes/plate0
            // so it matches the keys in paramsBank which come from the root stage
            SdfPath protoPathInRoot = protoPath.IsEmpty() ? SdfPath() : prototypesInRootPath.AppendPath(protoPath.MakeRelativePath(prototypesPath));

            const TessParams& params = findParams(protoPathInRoot);

            if (defShape.IsNull()) {
                int done = ++tessCompleted;
                std::cerr << "\r[" << done << "/" << total << "] Tessellating " << work.proto->variantName << "..." << std::flush;
                return;
            }

            auto partStart = Clock::now();
            std::string partName = getLabelName(defs[i].first);

            try {
                TessResult result;
                if (tesselatePart(result, defShape, params))
                    work.tessResults[i] = std::move(result);
            } catch (const Standard_Failure& e) {
                std::cerr << "\nOCC exception during tessellation of def " << i << ": " << e.GetMessageString() << "\n";
            }

            double elapsed = Seconds(Clock::now() - partStart).count();
            if (elapsed > 10.0) {
                std::cerr << "\n[SLOW] def " << i << " (" << partName << ")"
                        << "  time=" << elapsed << "s"
                        << "  faces=" << [&]{ int n=0; for(TopExp_Explorer e(defShape,TopAbs_FACE);e.More();e.Next()) n++; return n; }()
                        << "\n";
            }

            int done = ++tessCompleted;
            std::cerr << "\r[" << done << "/" << total << "] Tessellating " << work.proto->variantName << "..." << std::flush;
        });

        std::cerr << "\n";
    }

    // Write geometry for all variants in parallelz
    WorkParallelForEach(variantWork.begin(), variantWork.end(), [&](VariantWork& work) {
        const TessParams& rootParams = work.rootParams;

        auto findParams = [&](const SdfPath& path) -> const TessParams& {
            SdfPath p = path;
            while (!p.IsEmpty() && p != SdfPath::AbsoluteRootPath()) {
                auto it = work.paramsBank.find(p);
                if (it != work.paramsBank.end()) return it->second;
                p = p.GetParentPath();
            }
            return rootParams;
        };

        int completed = 0;
        for (int i = 0; i < protoTotal; i++) {
            SdfPath protoPath = prototypePaths[defs[i].first];
            SdfPath protoPathInRoot = protoPath.IsEmpty() ? SdfPath() : prototypesInRootPath.AppendPath(protoPath.MakeRelativePath(prototypesPath));

            const TessParams& params = findParams(protoPathInRoot);
            const TessResult& r = work.tessResults[i];

            if (!writePrototypeGeometry(work.proto->stage, protoPath, r, params, i)) {
                std::cerr << "\r[" << ++completed << "/" << protoTotal << "] Writing " << work.proto->variantName << "..." << std::flush;
                continue;
            }

            std::cerr << "\r[" << ++completed << "/" << protoTotal << "] Writing " << work.proto->variantName << "..." << std::flush;
        }
        std::cerr << "\n";

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