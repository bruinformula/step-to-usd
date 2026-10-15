#pragma once

#include "tokens.h"
#include <pxr/base/tf/token.h>
#include <pxr/pxr.h>
#include <pxr/usd/sdf/tokens.h>

#include <stdint.h>

#include "CadUSD/Tessellation/TessellationRoutineInterface.h"

#include "cadTessellationAPI.h"

PXR_NAMESPACE_USING_DIRECTIVE

namespace CadUSD {

struct SketchParams {
    double deflection = 0.01;
    TfToken type = AutolibTokens->linear;

    bool combineCurves = true;
    bool embedSurfaceNormals = true;
    uint64_t pointLimit = 65535;
    
    void populateParams(const AutolibCadTessellationAPI& api);
};

struct SketchPlaneParams {
    double linearDeflection = 0.01;
    double angularDeflection = 0.5;
    double minSize = 0.0;
    double combineTolerance = 1e-5;

    double fixPrecision = 1e-7;
    double fixTolerance = 1e-7;

    uint64_t fixTimeout = 3000;
    uint64_t meshTimeout = 3000;
    
    void populateParams(const AutolibCadTessellationAPI& api);
};

// Handles the Sketch and the SketchPlane
struct SketchTessellationContext;

struct SketchTessellationRoutine : public TessellationRoutineInterface {
    SketchParams sketch;
    SketchPlaneParams sketchPlane;
    double unitScale = 1;

    bool writePartMember(UsdStageRefPtr stage,
                         SdfPath& partPath) const override;

    bool tessellate(const TopoDS_Shape& defShape,
                    const TessParams& params,
                    const SdfPath& protoPath) override;

    bool definePrim(UsdStageRefPtr stage,
                    const SdfPath& protoPath,
                    const TessParams& params) const override;

    bool writePrim(UsdStageRefPtr stage,
                   const SdfPath& protoPath,
                   const TessParams& params) const override;

    void clearPrim(UsdStageRefPtr stage,
                   const SdfPath& protoPath) const override;

    size_t size() const override;

  private:
    // Sketch curves
    VtArray<GfVec3f> sketchPoints;
    VtArray<int> sketchCounts;
    VtArray<float> sketchArcValues;
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

    void initializeFreeEdges(const TopoDS_Shape& defShape,
                             SketchTessellationContext& ctx) const;

    void tessellateSketchPlane(const TopoDS_Shape& defShape,
                               const SdfPath& protoPath,
                               SketchTessellationContext& ctx);

    void tessellateSketch(const TopoDS_Shape& defShape,
                          const SdfPath& protoPath,
                          SketchTessellationContext& ctx);

    void applyUnitScale(const TessParams& params);

    bool hasValidGeometry();

    bool defineSketchPrim(UsdStageRefPtr stage,
                          const SdfPath& protoPath,
                          const TessParams& params) const;

    bool writeSketchPrim(UsdStageRefPtr stage,
                         const SdfPath& protoPath,
                         const TessParams& params) const;

    bool defineSketchPlanePrim(UsdStageRefPtr stage,
                               const SdfPath& protoPath,
                               const TessParams& params) const;

    bool writeSketchPlanePrim(UsdStageRefPtr stage,
                              const SdfPath& protoPath,
                              const TessParams& params) const;
};


} // namespace CadUSD