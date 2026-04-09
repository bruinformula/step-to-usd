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
    VtArray<int> curveContinuity;

    // Sketch curves
    VtArray<GfVec3f> sketchPoints;
    VtArray<int> sketchCounts;

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

struct TessParams {
    // Meshing
    float meshLinearDeflection = 1.0f;       // Linear deflection as fraction of bounding-box diagonal
    float meshAngularDeflection = 0.5f;      // Angular deflection in radians
    double meshMinSize = 0.0;                // Minimum triangle edge length as fraction of bounding-box diagonal
    double selfIntersectionThreshold = 1e-3; // Threshold for detecting self-intersecting triangles
    int maxNumberRemeshPasses = 1;           // Maximum remesh passes

    // Timeout in milliseconds
    uint64_t fixTimeout = 3000;            
    uint64_t meshTimeout = 3000;           
    uint64_t remeshTimeout = 3000;   
    
    //
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
    float wireframeDeflection = 0.01f;     
    CurveMode wireframeMode = { CurveType::Linear, CurveSampling::Underlying };
    bool wireframeCombineCurves = true;    

    // Sketch
    float sketchDeflection = 0.005f;       
    CurveMode sketchMode = { CurveType::Linear, CurveSampling::Underlying };
    bool sketchCombineCurves = true;       

    // Other 
    float renderPurposeThreshold = std::numeric_limits<float>::infinity();
    bool enableSurfaceSubsets = false;     
    bool enableUVs = true;
    bool enableSurfaceID = false;
    bool enableIsBoundaryVertex = false;
    double unitScale = 1.0;                // Internal exporter scale: source model units -> target USD units.
    // in the units of the model along the diagonal. 
    // if proto is smaller it gets marked as a render only asset
};

struct UsdStepExporter {
    
    static void populateUsd(
        const StepModel& model, 
        UsdStageRefPtr containerStage,
        UsdPrim& containerPrim,
        const std::unordered_set<SdfPath, SdfPath::Hash> selectedPaths
    );

private:

    struct UVPatch {
        std::vector<GfVec2f> uvs; // one per face-vertex, in raw param space
        float uMin, uMax, vMin, vMax;
    };

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

    static GfMatrix4d trsfToGfMatrix(const gp_Trsf& t, double linearScale = 1.0);

    static std::string sanitizeUsdName(const std::string_view& name, int idx);

    static VtArray<GfVec2f> packUVAtlas(std::vector<UVPatch>& patches);

    static bool validateVariants(
        UsdStageRefPtr containerStage,
        const SdfPath& containerPrimPath,
        const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths
    );

    static bool isPrototypeActiveInFilter(
        const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
        const SdfPath& containerPrimPath,
        const std::string& variantSetName,
        const std::string& variantName,
        const SdfPath& prototypePath
    );

    static std::optional<SdfReference> getPrototypesDefaultParams(const UsdPrim& containerPrim);
    
    static UsdStageRefPtr initUsdStage(
        const fs::path& newStagePath, 
        bool clearExisting
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

    static std::vector<SdfPath> computeNodePaths(
        const std::vector<StepModel::PartNode>& partNodes,
        const SdfPath& assemblyPath
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
        const UsdPrim& containerPrim,
        LabelMap<SdfPath>& prototypePaths
    );

    static void writeCadPart(
        UsdStageRefPtr prototypesStage,
        const UsdPrim& containerPrim,
        const SdfPath cadPartPath
    );

    static void writePrototypeXformsInPrototypesStage(
        UsdStageRefPtr prototypesStage,
        const UsdPrim& containerPrim,
        const std::vector<std::pair<TDF_Label, TopoDS_Shape>>& defs,
        const SdfPath& prototypesPath,
        const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
        const SdfPath& containerPrimPath,
        const std::string& variantSetName,
        const std::string& variantName,
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
};

// Usd Utils
template <typename T>
void updateIfAuthored(const UsdAttribute& attr, T* value);

template <>
void updateIfAuthored<TessParams::CurveType>(const UsdAttribute& attr, TessParams::CurveType* value);

template <>
void updateIfAuthored<TessParams::CurveSampling>(const UsdAttribute& attr, TessParams::CurveSampling* value);

extern template void updateIfAuthored<float>(const UsdAttribute&, float*);
extern template void updateIfAuthored<double>(const UsdAttribute&, double*);
extern template void updateIfAuthored<int>(const UsdAttribute&, int*);
extern template void updateIfAuthored<uint64_t>(const UsdAttribute&, uint64_t*);
extern template void updateIfAuthored<bool>(const UsdAttribute&, bool*);

std::map<SdfPath, TessParams> resolveParams(
    const UsdPrim& containerPrim,
    const TessParams& defaultParams
);

std::unordered_set<SdfPath, SdfPath::Hash> getVariantsOnPrim(
    const UsdPrim& prim
);