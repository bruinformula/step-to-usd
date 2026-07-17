#pragma once

#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>

#include "StepUSD/StepUsdPipeline.h"
#include "StepUSD/PrototypeMember/PrototypeMember.h"

PXR_NAMESPACE_USING_DIRECTIVE

class Poly_Triangulation;
class Geom_Surface;

struct TessellationRoutineInterface {
    virtual ~TessellationRoutineInterface() = default;

    virtual bool tessellate(
        const TopoDS_Shape& defShape, 
        const TessParams& params,
        const SdfPath& protoPath,
        TessResult& result
    ) = 0;
};

// Handles the Mesh and the Wireframe
struct UVPatch;
struct MeshTessellationContext;

struct MeshTessellationRoutine : public TessellationRoutineInterface {

    SdfPath protoPath;
    TessParams params;
    
    MeshPrim meshRoutine;
    WireframePrim wireframeRoutine;

    bool tessellate(
        const TopoDS_Shape& defShape, 
        const TessParams& params,
        const SdfPath& protoPath,
        TessResult& result
    ) override;

private:

    void meshShape(
        const TopoDS_Shape& defShape,
        TessResult& result
    );

    void emitResampledWireframeCurve(
        const TopoDS_Edge& edge,
        const NCollection_List<TopoDS_Shape>& adjFaces,
        int continuity,
        const TessParams& params,
        TessResult& result
    );

    void buildEdgeWalk(
        const TopoDS_Shape& defShape,
        const TessParams& params,
        MeshTessellationContext& ctx,
        TessResult& result
    );

    void countTrianglesAndNodes(const TopoDS_Shape& defShape, int& totalTris, int& totalNodes);

    void weldFaceNodes(
        const occt::handle<Poly_Triangulation>& tri,
        const gp_Trsf& trsf,
        MeshTessellationContext& ctx,
        TessResult& result,
        float bboxOut[6] // xmin, xmax, ymin, ymax, zmin, zmax
    );

    void emitFaceTriangles(
        const occt::handle<Poly_Triangulation>& tri,
        const gp_Trsf& trsf,
        bool reversed,
        bool hasUV,
        const occt::handle<Geom_Surface>& geomSurface,
        const gp_Vec& faceTangentU,
        const gp_Vec& faceTangentV,
        const gp_Pnt& faceCentroid,
        MeshTessellationContext& ctx,
        TessResult& result,
        UVPatch& patch
    );

    void tessellateFaces(
        const TopoDS_Shape& defShape,
        MeshTessellationContext& ctx,
        TessResult& result
    );

    void finalizeBoundaryData(
        MeshTessellationContext& ctx, 
        TessResult& result
    );

    void emitDeferredWireframeCurves(
        const TessParams& params,
        const MeshTessellationContext& ctx,
        const VtArray<GfVec3f>& pointNormals,
        TessResult& result
    );

    void compactUnusedPoints(TessResult& result);

    void applyUnitScale(const TessParams& params, TessResult& result);

    bool hasValidGeometry(const TessResult& result);
};

// Handles the Sketch and the SketchPlane
struct SketchTessellationContext;

struct SketchTessellationRoutine : public TessellationRoutineInterface {


    SdfPath protoPath;
    TessParams params;

    SketchPrim sketchRoutine;
    SketchPlanePrim sketchPlaneRoutine;
    
    bool tessellate(
        const TopoDS_Shape& defShape, 
        const TessParams& params,
        const SdfPath& protoPath,
        TessResult& result
    ) override;

private:

    void initializeFreeEdges(
        const TopoDS_Shape& defShape,
        SketchTessellationContext& ctx
    ) const;

    void tessellateSketchPlane(
        const TopoDS_Shape& defShape, 
        SketchTessellationContext& ctx,
        TessResult& result
    );

    void tessellateSketch(
        const TopoDS_Shape& defShape, 
        SketchTessellationContext& ctx,
        TessResult& result
    );

    void applyUnitScale(const TessParams& params, TessResult& result);

    bool hasValidGeometry(const TessResult& result);

};