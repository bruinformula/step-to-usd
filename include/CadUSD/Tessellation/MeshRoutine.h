#pragma once

#include <stdint.h>

#include <pxr/base/tf/token.h>
#include <pxr/pxr.h>
#include <pxr/usd/sdf/tokens.h>

#include "tokens.h"

#include "CadUSD/Tessellation/TessellationRoutineInterface.h"

#include "cadTessellationAPI.h"

PXR_NAMESPACE_USING_DIRECTIVE

namespace CadUSD {

struct MeshParams {
    double linearDeflection = 1.0;
    double angularDeflection = 0.5;
    double minSize = 0.0;

    double selfIntersectionThreshold = 1e-3;
    int maxRemeshPasses = 1;

    double fixPrecision = 1e-7;
    double fixTolerance = 1e-7;
    bool enableRepairPass = false;

    uint64_t fixTimeout = 3000;
    uint64_t meshTimeout = 3000;
    uint64_t remeshTimeout = 3000;

    bool enableSurfaceSubsets = false;
    bool enableUVs = true;
    bool enableSurfaceID = false;
    bool enableIsBoundaryVertex = false;

    void populateParams(const AutolibCadTessellationAPI& api);
};

struct WireframeParams {
    double deflection = 0.01;
    TfToken type = AutolibTokens->linear;
    TfToken sampling = AutolibTokens->underlying;

    bool combineCurves = true;
    bool embedSurfaceNormals = true;
    uint64_t pointLimit = 65535;
    
    void populateParams(const AutolibCadTessellationAPI& api);
};

// Handles the Mesh and the Wireframe
struct UVPatch;
struct MeshTessellationContext;

struct MeshTessellationRoutine : public TessellationRoutineInterface {
    MeshParams mesh;
    WireframeParams wireframe;
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

    static constexpr int kBoundaryEdgeSlots = 6;

  private:
    VtArray<GfVec3f> points;
    VtArray<GfVec3f> normals;
    VtArray<int> faceVertexCounts;
    VtArray<int> faceVertexIndices;

    VtArray<GfVec2f> perSurfaceUVs;
    VtArray<bool> isBoundaryVertex;
    VtArray<GfVec3f> boundaryTangents;

    VtArray<float> boundaryEdgeDistances[kBoundaryEdgeSlots];

    struct SurfaceIDBounds {
        int startIdx;
        int endIdx;
        int surfaceID;
    };

    std::vector<SurfaceIDBounds> surfaceIDBounds; // start, end index for a particular face id

    // Wireframe curves
    VtArray<GfVec3f> wireframePoints;
    VtArray<int> wireframeCounts;
    VtArray<float> wireframeArcValues;
    VtArray<GfVec3f> wireframeSurfaceNormals;
    VtArray<int> wireframeContinuity;

    void meshShape(const TopoDS_Shape& defShape);

    void emitResampledWireframeCurve(const TopoDS_Edge& edge,
                                     const NCollection_List<TopoDS_Shape>& adjFaces,
                                     int continuity,
                                     const TessParams& params);

    void buildEdgeWalk(const TopoDS_Shape& defShape,
                       const TessParams& params,
                       MeshTessellationContext& ctx);

    void countTrianglesAndNodes(const TopoDS_Shape& defShape,
                                int& totalTris,
                                int& totalNodes);

    void weldFaceNodes(const occt::handle<Poly_Triangulation>& tri,
                       const gp_Trsf& trsf,
                       MeshTessellationContext& ctx,
                       float bboxOut[6] // xmin, xmax, ymin, ymax, zmin, zmax
    );

    void emitFaceTriangles(const occt::handle<Poly_Triangulation>& tri,
                           const gp_Trsf& trsf,
                           bool reversed,
                           bool hasUV,
                           const occt::handle<Geom_Surface>& geomSurface,
                           const gp_Vec& faceTangentU,
                           const gp_Vec& faceTangentV,
                           const gp_Pnt& faceCentroid,
                           MeshTessellationContext& ctx,
                           UVPatch& patch);

    void tessellateFaces(const TopoDS_Shape& defShape,
                         MeshTessellationContext& ctx);

    void finalizeBoundaryData(MeshTessellationContext& ctx);

    void emitDeferredWireframeCurves(const TessParams& params,
                                     const MeshTessellationContext& ctx,
                                     const VtArray<GfVec3f>& pointNormals);

    void compactUnusedPoints();

    void applyUnitScale(const TessParams& params);

    bool hasValidGeometry();

    bool defineMeshPrim(UsdStageRefPtr stage,
                        const SdfPath& protoPath,
                        const TessParams& params) const;

    bool writeMeshPrim(UsdStageRefPtr stage,
                       const SdfPath& protoPath,
                       const TessParams& params) const;

    bool defineWireframePrim(UsdStageRefPtr stage,
                             const SdfPath& protoPath,
                             const TessParams& params) const;

    bool writeWireframePrim(UsdStageRefPtr stage,
                            const SdfPath& protoPath,
                            const TessParams& params) const;
};

} // namespace CadUSD