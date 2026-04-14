#pragma once

#include <limits>
#include <vector>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/common.h>
#include <pxr/usd/usd/timeCode.h>
#include <pxr/usd/usd/attribute.h>

#include <pxr/usd/sdf/reference.h>

#include <pxr/base/vt/array.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/gf/vec2f.h>

#pragma pop_macro("Handle")

#include "StepModel.h"

PXR_NAMESPACE_USING_DIRECTIVE

struct TessParams {
    // Meshing
    double meshLinearDeflection = 1.0f;       // Linear deflection as fraction of bounding-box diagonal
    double meshAngularDeflection = 0.5f;      // Angular deflection in radians
    double meshMinSize = 0.0;                // Minimum triangle edge length as fraction of bounding-box diagonal
    double meshSelfIntersectionThreshold = 1e-3; // Threshold for detecting self-intersecting triangles
    int meshMaxNumberRemeshPasses = 1;           // Maximum remesh passes
    double meshFixPrecision = 1e-7;          // Tolerance for shape fixing
    double meshFixTolerance = 1e-7;       // Tolerance for meshing operations

    // Timeout in milliseconds
    uint64_t meshFixTimeout = 3000;            
    uint64_t meshMeshTimeout = 3000;           
    uint64_t meshRemeshTimeout = 3000;   
    
    enum class CurveType {
        None,                // ain't got nothing on me
        Linear,              // polyline using the tessellated mesh boundary vertices directly
        Cubic          // cubic Catmull-Rom using the tessellated mesh boundary vertices directly
    };

    enum class CurveSampling {
        Underlying, // polyline using the tessellated mesh boundary vertices
        Resampled   // resampled from the underlying curve geometry
    };

    struct CurveMode {
        CurveType type;
        CurveSampling sampling;
    };

    // Wireframe
    double wireframeDeflection = 0.01f;     
    CurveMode wireframeMode = { CurveType::Linear, CurveSampling::Underlying };
    bool wireframeCombineCurves = true;
    bool wireframeEmbedSurfaceNormals = true;

    // Sketch
    double sketchDeflection = 0.005f;       
    CurveMode sketchMode = { CurveType::Linear, CurveSampling::Underlying };
    bool sketchCombineCurves = true;
    bool sketchEmbedSurfaceNormals = true;

    // Sketch Plane 
    double sketchPlaneLinearDeflection = 0.01f;
    double sketchPlaneAngularDeflection = 0.5f;
    double sketchPlaneMinSize = 0.0;
    double sketchPlaneCombineTolerance = 1e-5;
    double sketchPlaneFixPrecision = 1e-7;
    double sketchPlaneFixTolerance = 1e-7;

    uint64_t sketchPlaneFixTimeout = 3000;            
    uint64_t sketchPlaneMeshTimeout = 3000;     

    // Other 
    double renderPurposeThreshold = std::numeric_limits<double>::infinity();
    bool meshEnableSurfaceSubsets = false;     
    bool meshEnableUVs = true;
    bool meshEnableSurfaceID = false;
    bool meshEnableIsBoundaryVertex = false;
    double unitScale = 1.0;                // Internal exporter scale: source model units -> target USD units.
    // in the units of the model along the diagonal. 
    // if proto is smaller it gets marked as a render only asset
};

struct TessResult {
    SdfPath targetLayer;

    VtArray<GfVec3f> points;
    VtArray<GfVec3f> normals;
    VtArray<int> faceVertexCounts;
    VtArray<int> faceVertexIndices;

    VtArray<GfVec2f> perSurfaceUVs;
    VtArray<int> surfaceIDs;
    VtArray<bool> isBoundaryVertex;

    struct SurfaceIDBounds {
        int startIdx;
        int endIdx;
        int surfaceID;
    };

    std::vector<SurfaceIDBounds> surfaceIDBounds; // start, end index for a particular face id 

    // Wireframe curves 
    VtArray<GfVec3f> curvePoints;
    VtArray<int> wireframeCounts;
    VtArray<GfVec3f> wireframeSurfaceNormals;
    VtArray<int> curveContinuity;

    // Sketch curves
    VtArray<GfVec3f> sketchPoints;
    VtArray<int> sketchCounts;
    VtArray<GfVec3f> sketchSurfaceNormals;

    // Sketch planes reconstructed from closed free-edge loops
    VtArray<GfVec3f> sketchPlanePoints;
    VtArray<GfVec3f> sketchPlaneNormals;
    VtArray<int> sketchPlaneFaceVertexCounts;
    VtArray<int> sketchPlaneFaceVertexIndices;

    struct SketchPlaneBounds {
        int pointStart;
        int pointCount;
        int faceCountStart;
        int faceCountCount;
        int faceIndexStart;
        int faceIndexCount;
        int normalStart;
        int normalCount;
    };

    std::vector<SketchPlaneBounds> sketchPlaneBounds;

    // Render Purpose
    bool renderOnly; 
};

struct StageFilterInfo {
    bool makeFresh = false; // whole stage targeted
    bool hasSpecificPrototypes = false; // individual prototypes targeted
};

struct UsdStepExporter {

    static std::optional<UsdStepExporter> create(
        const fs::path& inputUsdFile
    );

    void populateUsd(
        UsdStageRefPtr containerStage,
        UsdPrim& containerPrim,
        const std::unordered_set<SdfPath, SdfPath::Hash> selectedPaths
    );

    UsdStepExporter(
        UsdStageRefPtr cs, 
        std::unordered_map<SdfAssetPath, StepModel, SdfAssetPath::Hash> mc
    ) : containerStage(std::move(cs)), modelCache(std::move(mc)) {}

    UsdStageRefPtr containerStage;
    std::unordered_map<SdfAssetPath, StepModel, SdfAssetPath::Hash> modelCache;

private:

    struct PrototypeContainer {
        std::string variantSetName;
        std::string variantName;
        fs::path filePath;
        UsdStageRefPtr stage;
        bool makeFreshStage = false;
    };

    struct TessellationJob {
        const PrototypeContainer* proto;
        int defIndex;
        SdfPath prototypePath;
        TessParams params;
        TessResult result;
        bool runMesherInParallel = false;
    };

    struct ProtoGeomJob {
        SdfPath protoPath;
        TessResult result;
        TessParams params;
    };

    static bool isAssemblyActiveInFilter(
        const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
        const SdfPath& containerPrimPath,
        const SdfPath& prototypePath
    );

    static bool isPrototypeActiveInFilter(
        const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
        const SdfPath& containerPrimPath,
        const SdfPath& prototypePath,
        const std::string& variantSetName,
        const std::string& variantName
    );

    static bool tessellatePart(
        TessResult& result, 
        const TopoDS_Shape& defShape, 
        const TessParams& params,
        bool parallel = false
    );

    static void tessellateGeometry(
        std::vector<TessellationJob>& tessJobs,
        const std::vector<std::pair<TDF_Label, TopoDS_Shape>>& defs,
        const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
        const SdfPath& containerPrimPath
    );

    static bool populatePrototypeContainers(
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
    );

    static bool buildPrototypeAndAssemblyStages(
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
    );

    static bool populateParamsBank(
        const UsdStageRefPtr& containerStage,
        const UsdPrim& containerPrim,
        const PrototypeContainer& proto,
        const SdfPath& prototypesPath,
        const TessParams& variantLevelParams,
        std::map<SdfPath, TessParams>& paramsBank
    );

    static bool populateTessellationJobs(
        const std::vector<PrototypeContainer>& prototypes,
        const UsdStageRefPtr& containerStage,
        const UsdPrim& containerPrim,
        const SdfPath& prototypesInContainerPath,
        const SdfPath& prototypesPath,
        const std::vector<std::pair<TDF_Label, TopoDS_Shape>>& defs,
        const LabelMap<SdfPath>& prototypePaths,
        double sourceToOutputScale,
        std::vector<TessellationJob>& tessJobs
    );

    static void writeAssemblyXforms(
        UsdStageRefPtr stage, 
        const SdfPath& containerPrimPath,
        const std::vector<StepModel::PartNode>& instances,
        const std::vector<SdfPath>& paths, 
        const LabelMap<SdfPath>& prototypePaths,
        double linearScale
    );

    static void writePrototypeOverridesInAssemblyStage(
        UsdStageRefPtr assemblyStage,
        LabelMap<SdfPath>& prototypePaths
    );

    static void writeCadPart(
        UsdStageRefPtr prototypesStage,
        const UsdPrim& prototypesPrimOnContainerStage,
        const SdfPath& containerPrimPath,
        const SdfPath& cadPartPath
    );

    static void writePrototypeXformsInPrototypesStage(
        UsdStageRefPtr prototypesStage,
        const std::vector<std::pair<TDF_Label, TopoDS_Shape>>& defs,
        const SdfPath& prototypesPath,
        const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
        const SdfPath& containerPrimPath,
        const std::string& variantSetName,
        const std::string& variantName,
        const LabelMap<std::string>& definitionNames,
        LabelMap<SdfPath>& prototypePaths,
        bool makeFreshStage
    );

    static void writePrototypeGeometries(
        UsdStageRefPtr stage,
        const std::vector<ProtoGeomJob>& jobs,
        const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
        const SdfPath& containerPrimPath,
        const std::string& variantSetName,
        const std::string& variantName
    );

    static bool writePrototypeXform(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        int defIdx
    );

    static bool writePrototypeGeometry(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        const TessResult& r,
        const TessParams& params,
        int defIdx
    ); 

    static void writeGeometry(
        const std::vector<std::pair<TDF_Label, TopoDS_Shape>>& defs,
        const LabelMap<SdfPath>& prototypePaths,
        const SdfPath& prototypesPath,
        const SdfPath& prototypesInContainerPath,
        UsdStageRefPtr targetStage,
        const std::string& logLabel,
        const TessParams& containerParams,
        const std::vector<TessResult>& tessResults,
        const std::map<SdfPath, TessParams>& paramsBank,
        const std::unordered_set<SdfPath, SdfPath::Hash>& prototypeFilter = {}
    );

    static std::optional<SdfReference> getPrototypesDefaultParams(const UsdPrim& containerPrim);

    static std::map<SdfPath, TessParams> resolveParams(
        const UsdPrim& containerPrim,
        const TessParams& defaultParams
    );

    static TessParams getTessParams(
        UsdPrim prim,
        const TessParams& defaultParams = {}
    );
};