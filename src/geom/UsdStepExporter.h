#pragma once

#include <filesystem>
#include <limits>
#include <string>
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
    VtArray<int> curveCounts;
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

template <typename T>
bool getInheritedAttribute(const UsdPrim& prim, const TfToken& attrName, T* value, const UsdTimeCode& time = UsdTimeCode::Default());

template <typename T>
void updateIfAuthored(const UsdAttribute& attr, T* value);

struct UsdStepExporter {

    struct VariantParams {
        TessParams tessParams;
        std::string variantName;
        std::string variantSetName;
        std::filesystem::path outpath;  
        std::filesystem::path refpath;
        bool overwrite = false;
    };

    static void populateUsd(const StepModel& model, UsdStageRefPtr stage, const TessParams& params);
    static void populateVariantUsd(const StepModel& model, UsdStageRefPtr stage, const std::vector<VariantParams>& variantParams);

private:
    static std::vector<SdfPath> computeInstancePaths(const std::vector<StepModel::PartInstance>& instances);

    static void writeInstanceXforms(
        const std::vector<StepModel::PartInstance>& instances,
        UsdStageRefPtr stage, 
        const std::vector<SdfPath>& paths, 
        const LabelMap<SdfPath>& prototypePaths
    );

    static bool writePrototypeGeometry(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        const TessResult& r,
        const CurveMode& wireframeMode,
        const CurveMode& sketchMode,
        int defIdx
    ); 
};