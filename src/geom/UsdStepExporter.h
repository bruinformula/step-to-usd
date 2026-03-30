#pragma once

#include <limits>
#include <vector>

#pragma push_macro("Handle") // pxr, CGAL, and occt all define Handle as a macro
#undef Handle

#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/common.h>
#include <pxr/usd/usd/timeCode.h>

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
};

struct UVPatch {
    std::vector<GfVec2f> uvs; // one per face-vertex, in raw param space
    float uMin, uMax, vMin, vMax;
};

struct UsdStepExporter {

    static UsdStageRefPtr initUsdStage(
        const fs::path& newStagePath, 
        const SdfPath& rootPrimPath,
        bool writeCadPart = false
    );

    static void populateUsdPlain(
        const StepModel& model, 
        const fs::path& newStagePath, 
        const TessParams& params
    );
    
    #ifdef AUTOLIB_BUILD_STEP_USD_SCHEMA
    static void populateUsd(
        const StepModel& model, 
        UsdStageRefPtr rootStage,
        UsdPrim& rootPrim // on the stage with the stronger opinions
    );

    static void populateUsdVariant(
        const StepModel& model, 
        UsdStageRefPtr rootStage,
        UsdPrim& rootPrim, // on the stage with the stronger opinions
        const std::map<std::string, std::vector<std::string>>& variantSetNameToVariantNames
    );
    #endif

private:

    static GfMatrix4d trsfToGfMatrix(const gp_Trsf& t);

    static std::string sanitizeUsdName(const std::string_view& name, int idx);

    static VtArray<GfVec2f> packUVAtlas(std::vector<UVPatch>& patches);

    static bool tesselatePart(
        TessResult& result, 
        const TopoDS_Shape& defShape, 
        const TessParams& params
    );

    static std::vector<SdfPath> computeNodePaths(
        const std::vector<StepModel::PartNode>& partNodes,
        const SdfPath& assemblyPath
    );

    static void writeAssemblyXforms(
        const std::vector<StepModel::PartNode>& instances,
        UsdStageRefPtr stage, 
        const std::vector<SdfPath>& paths, 
        const LabelMap<SdfPath>& prototypePaths
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

};