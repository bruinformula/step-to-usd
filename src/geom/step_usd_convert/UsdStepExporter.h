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

enum class CurveType {
    None,                // ain't got nothing on me
    Linear,              // polyline using the tessellated mesh boundary vertices directly
    CatmullRom          // cubic Catmull-Rom using the tessellated mesh boundary vertices directly
};

enum class CurveSampling {
    Underlying, // polyline using the tessellated mesh boundary vertices
    Resampled   // resampled from the underlying curve geometry
};

struct CurveMode {
    CurveType type;
    CurveSampling sampling;
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
    VtArray<int> curveContinuity;

    // Sketch curves
    VtArray<GfVec3f> sketchPoints;
    VtArray<int> sketchCounts;

    // Render Purpose
    bool renderOnly; 
};

struct TessParams {
    float renderPurposeThreshold = std::numeric_limits<float>::infinity();
    // in the units of the model along the diagonal. 
    // if proto is smaller it gets marked as a render only asset

    float meshLinearDeflection = 0.05f; // as a fraction of the diagonal of the bounding box
    float meshAngularDeflection = 0.35f;
    double meshMinSize = 0.1;

    float wireframeDeflection = 1.0f;
    CurveMode wireframeMode = { CurveType::Linear, CurveSampling::Underlying };

    float sketchDeflection = 0.5f;
    CurveMode sketchMode = { CurveType::Linear, CurveSampling::Underlying };

    double selfIntersectionThreshold = 1e-3;
    int maxNumberRemeshPasses = 3;

    // Timeout in milliseconds
    uint64_t fixTimeout = 3000;
    uint64_t meshTimeout = 3000;
    uint64_t remeshTimeout = 3000;
};

struct UsdStepExporter {

    enum class LoggingMode {
        NONE,
        VERBOSE
    };
    
    static void populateUsd(
        const StepModel& model, 
        UsdStageRefPtr rootStage,
        UsdPrim& rootPrim,
        const std::unordered_set<SdfPath, SdfPath::Hash> selectedPaths,
        LoggingMode verbose = LoggingMode::NONE
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
    };

    struct TessellationJob {
        const PrototypeContainer* proto;
        int defIndex;
        SdfPath prototypePath;
        TessParams params;
        TessResult result;
        bool parallel = false;
    };

    struct ProtoGeomJob {
        SdfPath protoPath;
        TessResult result;
        TessParams params;
    };

    static GfMatrix4d trsfToGfMatrix(const gp_Trsf& t);

    static std::string sanitizeUsdName(const std::string_view& name, int idx);

    static VtArray<GfVec2f> packUVAtlas(std::vector<UVPatch>& patches);

    static bool isPrototypeActiveInFilter(
        const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
        const SdfPath& rootPrimPath,
        const std::string& variantSetName,
        const std::string& variantName,
        const SdfPath& prototypePath
    );

    static std::optional<SdfReference> getPrototypesDefaultParams(const UsdPrim& rootPrim);
    
    static UsdStageRefPtr initUsdStage(
        const fs::path& newStagePath, 
        const SdfPath& rootPrimPath,
        bool clearExisting
    );

    static bool tesselatePart(
        TessResult& result, 
        const TopoDS_Shape& defShape, 
        const TessParams& params,
        bool parallel = false
    );

    static void tessellateGeometry(
        std::vector<TessellationJob>& tessJobs,
        const std::vector<std::pair<TDF_Label, TopoDS_Shape>>& defs,
        const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
        const SdfPath& rootPrimPath
    );

    static std::vector<SdfPath> computeNodePaths(
        const std::vector<StepModel::PartNode>& partNodes,
        const SdfPath& assemblyPath
    );

    static void writeAssemblyXforms(
        UsdStageRefPtr stage, 
        const SdfPath& rootPrimPath,
        const std::vector<StepModel::PartNode>& instances,
        const std::vector<SdfPath>& paths, 
        const LabelMap<SdfPath>& prototypePaths
    );

    static void writePrototypeOverridesInAssemblyStage(
        UsdStageRefPtr assemblyStage,
        const UsdPrim& rootPrim,
        LabelMap<SdfPath>& prototypePaths
    );

    static void writeCadPart(
        UsdStageRefPtr prototypesStage,
        const UsdPrim& rootPrim,
        const SdfPath cadPartPath
    );

    static void writePrototypeXformsInPrototypesStage(
        UsdStageRefPtr prototypesStage,
        const UsdPrim& rootPrim,
        const std::vector<std::pair<TDF_Label, TopoDS_Shape>>& defs,
        const SdfPath& prototypesPath,
        const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
        const SdfPath& rootPrimPath,
        const std::string& variantSetName,
        const std::string& variantName,
        LabelMap<SdfPath>& prototypePaths,
        bool makeFreshStage
    );

    static void writePrototypeGeometries(
        UsdStageRefPtr stage,
        const std::vector<ProtoGeomJob>& jobs,
        const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
        const SdfPath& rootPrimPath,
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
        const SdfPath& prototypesInRootPath,
        UsdStageRefPtr targetStage,
        const std::string& logLabel,
        const TessParams& rootParams,
        const std::vector<TessResult>& tessResults,
        const std::map<SdfPath, TessParams>& paramsBank,
        const std::unordered_set<SdfPath, SdfPath::Hash>& prototypeFilter = {}
    );
};

template <typename T>
void updateIfAuthored(const UsdAttribute& attr, T* value);

template <>
void updateIfAuthored<CurveType>(const UsdAttribute& attr, CurveType* value);

template <>
void updateIfAuthored<CurveSampling>(const UsdAttribute& attr, CurveSampling* value);

extern template void updateIfAuthored<float>(const UsdAttribute&, float*);
extern template void updateIfAuthored<double>(const UsdAttribute&, double*);
extern template void updateIfAuthored<int>(const UsdAttribute&, int*);
extern template void updateIfAuthored<uint64_t>(const UsdAttribute&, uint64_t*);

std::map<SdfPath, TessParams> resolveParams(
    const UsdPrim& rootPrim,
    const TessParams& defaultParams
);

static TessParams getTessParams(UsdPrim prim, const TessParams& defaultParams = {});