#pragma once

#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/pxr.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/base/work/loops.h>
#include <pxr/base/work/workTBB/loops_impl.h>

#include <pxr/base/vt/array.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/gf/vec2f.h>
#include <pxr/usd/sdf/path.h>

#pragma pop_macro("Handle")

PXR_NAMESPACE_USING_DIRECTIVE

class Poly_Triangulation;
class Geom_Surface;

namespace occt = opencascade;

struct TessParams {
    // Meshing
    double meshLinearDeflection = 1.0f;       // Linear deflection as fraction of bounding-box diagonal
    double meshAngularDeflection = 0.5f;      // Angular deflection in radians
    double meshMinSize = 0.0;                // Minimum triangle edge length as fraction of bounding-box diagonal
    double meshSelfIntersectionThreshold = 1e-3; // Threshold for detecting self-intersecting triangles
    int meshMaxNumberRemeshPasses = 1;           // Maximum remesh passes
    double meshFixPrecision = 1e-7;          // Tolerance for shape fixing
    double meshFixTolerance = 1e-7;       // Tolerance for meshing operations
    bool meshEnableRepairPass = true;    // Enables a repair pass before meshing 

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
    uint64_t wireframePointLimit = 65535; 

    // Sketch
    double sketchDeflection = 0.005f;       
    CurveMode sketchMode = { CurveType::Linear, CurveSampling::Underlying };
    bool sketchCombineCurves = true;
    bool sketchEmbedSurfaceNormals = true;
    uint64_t sketchPointLimit = 65535; 

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

struct TessellationRoutineInterface {
    virtual ~TessellationRoutineInterface() = default;

    virtual bool tessellate(
        const TopoDS_Shape& defShape, 
        const TessParams& params,
        const SdfPath& protoPath
    ) = 0;

    virtual bool definePrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        const TessParams& params
    ) const = 0;

    virtual bool writePrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        const TessParams& params
    ) const = 0;

    virtual void clearPrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath
    ) const = 0;

    virtual size_t size() const = 0;
};

// Handles the Mesh and the Wireframe
struct UVPatch;
struct MeshTessellationContext;

struct MeshTessellationRoutine : public TessellationRoutineInterface {
    TessParams params;

    bool tessellate(
        const TopoDS_Shape& defShape, 
        const TessParams& params,
        const SdfPath& protoPath
    ) override;

    bool definePrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        const TessParams& params
    ) const override;

    bool writePrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        const TessParams& params
    ) const override;

    void clearPrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath
    ) const override;

    size_t size() const override;

private:

    VtArray<GfVec3f> points;
    VtArray<GfVec3f> normals;
    VtArray<int> faceVertexCounts;
    VtArray<int> faceVertexIndices;

    VtArray<GfVec2f> perSurfaceUVs;
    VtArray<bool> isBoundaryVertex;
    VtArray<GfVec3f> boundaryTangents;

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

    void meshShape(
        const TopoDS_Shape& defShape
    );

    void emitResampledWireframeCurve(
        const TopoDS_Edge& edge,
        const NCollection_List<TopoDS_Shape>& adjFaces,
        int continuity,
        const TessParams& params
    );

    void buildEdgeWalk(
        const TopoDS_Shape& defShape,
        const TessParams& params,
        MeshTessellationContext& ctx
    );

    void countTrianglesAndNodes(const TopoDS_Shape& defShape, int& totalTris, int& totalNodes);

    void weldFaceNodes(
        const occt::handle<Poly_Triangulation>& tri,
        const gp_Trsf& trsf,
        MeshTessellationContext& ctx,
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
        UVPatch& patch
    );

    void tessellateFaces(
        const TopoDS_Shape& defShape,
        MeshTessellationContext& ctx
    );

    void finalizeBoundaryData(
        MeshTessellationContext& ctx
    );

    void emitDeferredWireframeCurves(
        const TessParams& params,
        const MeshTessellationContext& ctx,
        const VtArray<GfVec3f>& pointNormals
    );

    void compactUnusedPoints();

    void applyUnitScale(const TessParams& params);

    bool hasValidGeometry();

    bool defineMeshPrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        const TessParams& params
    ) const;

    bool writeMeshPrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        const TessParams& params
    ) const;

    bool defineWireframePrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        const TessParams& params
    ) const;

    bool writeWireframePrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        const TessParams& params
    ) const;
};

// Handles the Sketch and the SketchPlane
struct SketchTessellationContext;

struct SketchTessellationRoutine : public TessellationRoutineInterface {
    TessParams params;
    
    bool tessellate(
        const TopoDS_Shape& defShape, 
        const TessParams& params,
        const SdfPath& protoPath
    ) override;

    bool definePrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        const TessParams& params
    ) const override;

    bool writePrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        const TessParams& params
    ) const override;

    void clearPrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath
    ) const override;

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

    void initializeFreeEdges(
        const TopoDS_Shape& defShape,
        SketchTessellationContext& ctx
    ) const;

    void tessellateSketchPlane(
        const TopoDS_Shape& defShape,
        const SdfPath& protoPath,
        SketchTessellationContext& ctx
    );

    void tessellateSketch(
        const TopoDS_Shape& defShape, 
        const SdfPath& protoPath,
        SketchTessellationContext& ctx
    );

    void applyUnitScale(const TessParams& params);

    bool hasValidGeometry();

    bool defineSketchPrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        const TessParams& params
    ) const;

    bool writeSketchPrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        const TessParams& params
    ) const;

    bool defineSketchPlanePrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        const TessParams& params
    ) const;

    bool writeSketchPlanePrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        const TessParams& params
    ) const;

};

struct TessellationRoutine : public TessellationRoutineInterface {
    bool tessellate(
        const TopoDS_Shape& defShape, 
        const TessParams& params,
        const SdfPath& protoPath
    ) override;

    bool definePrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        const TessParams& params
    ) const override;

    bool writePrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        const TessParams& params
    ) const override;
    
    void clearPrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath
    ) const override;

    size_t size() const override;

private:
    bool renderOnly = false;

    SketchTessellationRoutine sketchRoutine;
    MeshTessellationRoutine meshRoutine;

};