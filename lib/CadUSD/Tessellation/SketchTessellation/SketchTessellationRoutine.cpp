#include <chrono>
#include <utility>
#include <algorithm>
#include <string>
#include <vector>

#include <GProp_PEquation.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <BOPAlgo_BuilderFace.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TDF_Label.hxx>
#include <TopLoc_Location.hxx>
#include <gp_Trsf.hxx>
#include <ShapeFix_Shape.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <Poly_Triangulation.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <GeomLProp_SLProps.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <BRepBndLib.hxx>
#include <TopExp.hxx>
#include <IMeshTools_Parameters.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_QuasiUniformDeflection.hxx>
#include <BRepExtrema_SelfIntersection.hxx>
#include <BRepTools.hxx>
#include <BRepExtrema_MapOfIntegerPackedMapOfInteger.hxx>
#include <Bnd_Box.hxx>
#include <BOPAlgo_Splitter.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRep_Builder.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <GeomAbs_Shape.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_IndexedMap.hxx>
#include <NCollection_List.hxx>
#include <Poly_PolygonOnTriangulation.hxx>
#include <Poly_Triangle.hxx>
#include <ShapeAnalysis_FreeBounds.hxx>
#include <Standard_Failure.hxx>
#include <Standard_Handle.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopTools_HSequenceOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Message_ProgressRange.hxx>
#include <BRepOffsetAPI_MakeFilling.hxx>
#include <Geom_Plane.hxx>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/pxr.h>
#include <pxr/base/work/loops.h>
#include <pxr/base/work/workTBB/loops_impl.h>

#include <pxr/base/vt/array.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/gf/vec2f.h>
#include <pxr/usd/sdf/path.h>

#pragma pop_macro("Handle")

#include "CadUSD/Logger.h"
#include "CadUSD/Tessellation/TessellationRoutine.h"
#include "CadUSD/Tessellation/TessellationUtils.h"
#include "CadUSD/Tessellation/DeadlineProgressIndicator.h"

PXR_NAMESPACE_USING_DIRECTIVE

struct SketchTessellationContext { 
    std::vector<TopoDS_Edge> freeEdges;
};

void SketchTessellationRoutine::initializeFreeEdges(
    const TopoDS_Shape& defShape,
    SketchTessellationContext& ctx
) const {

    // sketches in STEP 242 are registered as free edges 
    // in the defintion shape and are not guaranteed to be connected to any faces, 
    // so we have to do a separate edge walk to find them and sample 

    NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher> edgeToFaces;
    TopExp::MapShapesAndAncestors(defShape, TopAbs_EDGE, TopAbs_FACE, edgeToFaces);

    ctx.freeEdges.reserve(edgeToFaces.Extent());
    for (TopExp_Explorer edgeExp(defShape, TopAbs_EDGE); edgeExp.More(); edgeExp.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(edgeExp.Current());
        if (BRep_Tool::Degenerated(edge)) continue;

        int edgeIdx = edgeToFaces.FindIndex(edge);
        bool isFreeEdge = (edgeIdx == 0) || (edgeToFaces.FindFromIndex(edgeIdx).Extent() == 0);
        if (isFreeEdge) {
            ctx.freeEdges.push_back(edge);
        }
    }
}

void SketchTessellationRoutine::tessellateSketchPlane(
    const TopoDS_Shape& defShape, 
    const SdfPath& protoPath,
    SketchTessellationContext& ctx
) {
    // Build sketch planes from free edges using OCCT topology:
    // split all free edges at intersections, connect split edges into wires,
    // convert closed wires into planar faces, then triangulate those faces.

    std::string protoName = protoPath.GetName();
    
    // TODO: Support Muliplanar sketches
    
    // Assemble free edges into a compound and connect them into wires
    TopoDS_Compound freeEdgeCompound;
    BRep_Builder builder;
    builder.MakeCompound(freeEdgeCompound);
    for (const TopoDS_Edge& edge : ctx.freeEdges) {
        builder.Add(freeEdgeCompound, edge);
    }

    TopoDS_Shape splitShape = freeEdgeCompound;
    {
        BOPAlgo_Splitter splitter;
        for (const TopoDS_Edge& edge : ctx.freeEdges) {
            splitter.AddArgument(edge);
        }
        
        splitter.Perform();
        if (!splitter.HasErrors()) { 
            splitShape = splitter.Shape();
        } else {
            std::ostringstream e;
            splitter.DumpErrors(e);
            LOG_DEBUG("  -> Sketch plane splitter reported error:" + e.str());
        }
    }

    opencascade::handle<TopTools_HSequenceOfShape> edgeSeq = new TopTools_HSequenceOfShape();
    {
        TopTools_IndexedMapOfShape splitEdgeMap;
        TopExp::MapShapes(splitShape, TopAbs_EDGE, splitEdgeMap);
        LOG_DEBUG("  -> Sketch plane split edge count=" + std::to_string(splitEdgeMap.Extent()));
        for (int ei = 1; ei <= splitEdgeMap.Extent(); ++ei) {
            const TopoDS_Edge& e = TopoDS::Edge(splitEdgeMap.FindKey(ei));
            if (!BRep_Tool::Degenerated(e))
                edgeSeq->Append(e);
        }
    }
    LOG_DEBUG("  -> Sketch plane usable split edges=" + std::to_string(edgeSeq->Length()));

    double diagonal = computeBoundingBoxDiagonal(freeEdgeCompound);

    opencascade::handle<TopTools_HSequenceOfShape> wireSeq =  new TopTools_HSequenceOfShape();
    double edgeTolerance = std::max(
        params.sketchPlaneCombineTolerance,
        diagonal * params.sketchPlaneCombineTolerance
    );
    ShapeAnalysis_FreeBounds::ConnectEdgesToWires(edgeSeq, edgeTolerance, true, wireSeq);

    if (wireSeq->Length() >= edgeSeq->Length()) {
        opencascade::handle<TopTools_HSequenceOfShape> fallbackWireSeq = new TopTools_HSequenceOfShape();
        const double fallbackTol = std::max(edgeTolerance, static_cast<double>(params.sketchDeflection));
        ShapeAnalysis_FreeBounds::ConnectEdgesToWires(edgeSeq, fallbackTol, false, fallbackWireSeq);
        if (fallbackWireSeq->Length() < wireSeq->Length()) {
            wireSeq = fallbackWireSeq;
            edgeTolerance = fallbackTol;
            LOG_DEBUG("  -> Sketch plane wire connect fallback used (shared=false)");
        }
    }
    LOG_DEBUG("  -> Sketch plane wire candidates=" + std::to_string(wireSeq->Length()) + " (tol=" + std::to_string(edgeTolerance) + ")");

    // Build a planar face from every closed wire.
    auto isWireGeometricallyClosed = [&](const TopoDS_Wire& wire, double tol) -> bool {
        TopoDS_Vertex vFirst, vLast;
        TopExp::Vertices(wire, vFirst, vLast);
        if (vFirst.IsNull() || vLast.IsNull()) 
            return false;
        if (vFirst.IsSame(vLast))               
            return true;
        return BRep_Tool::Pnt(vFirst).Distance(BRep_Tool::Pnt(vLast)) <= tol;
    };

    int closedWireCount = 0;
    int geomClosedWireCount = 0;
    int makeFaceFailedCount = 0;
    int builtFaceCount = 0;

    // Collect all successfully built faces for the union step.
    TopoDS_Compound faceCompound;
    builder.MakeCompound(faceCompound);

    for (int wi = 1; wi <= wireSeq->Length(); ++wi) {
        const TopoDS_Wire wire = TopoDS::Wire(wireSeq->Value(wi));

        const bool topoClosed = BRep_Tool::IsClosed(wire);
        const bool geomClosed = isWireGeometricallyClosed(wire, edgeTolerance);
        if (geomClosed) ++geomClosedWireCount;
        if (!(topoClosed || geomClosed)) continue;
        ++closedWireCount;

        BRepBuilderAPI_MakeFace makeFace(wire, true);
        if (!makeFace.IsDone()) {
            ++makeFaceFailedCount;
            continue;
        }
        const TopoDS_Face f = makeFace.Face();
        if (f.IsNull()) { ++makeFaceFailedCount; continue; }

    }

    LOG_DEBUG("  -> Sketch plane closed wires=" + std::to_string(closedWireCount) +
            ", geomClosed=" + std::to_string(geomClosedWireCount) +
            ", makeFaceFailed=" + std::to_string(makeFaceFailedCount) +
            ", builtFaces=" + std::to_string(builtFaceCount));

    // Union all faces so overlapping / nested regions
    // collapse into one non-overlapping shell.
    // We iterate over the compound and fuse each face into
    // a running accumulator. 

    TopoDS_Shape unifiedShape = faceCompound; // safe fallback

    // std::string generatedShapeSavePath = protoName + ".brep";
    // BRepTools::Write(unifiedShape, generatedShapeSavePath.c_str());

    if (builtFaceCount > 1) {
        try {
            TopExp_Explorer faceIt(faceCompound, TopAbs_FACE);

            TopoDS_Shape running = faceIt.Current(); // seed with first face
            faceIt.Next();

            for (; faceIt.More(); faceIt.Next()) {
                BRepAlgoAPI_Fuse fuse(running, faceIt.Current());
                fuse.Build();
                
                if (fuse.IsDone() && !fuse.HasErrors()) {
                    running = fuse.Shape();
                } else {
                    std::ostringstream e;
                    fuse.DumpErrors(e);
                    LOG_DEBUG("  -> Sketch plane: one fuse step failed with error " + e.str() +", face skipped");
                }
            }

            unifiedShape = running;
            LOG_DEBUG("  -> Sketch plane union complete");
        } catch (const Standard_Failure& e) {
            LOG_DEBUG(std::string("  -> Sketch plane fuse failed, using raw compound: ") + e.GetMessageString());
            unifiedShape = faceCompound;
        }
    }

    ShapeFix_Shape fixer(defShape);
    fixer.SetPrecision(params.sketchPlaneFixPrecision);
    fixer.SetMaxTolerance(params.sketchPlaneFixTolerance);
    
    opencascade::handle<DeadlineProgressIndicator> fixProgress;
    {
        std::string label = "ShapeFix_Shape for part " + protoName + ": ";
        fixProgress = new DeadlineProgressIndicator(std::chrono::milliseconds(params.sketchPlaneFixTimeout), label);
    }
    Message_ProgressRange fixRange = fixProgress->Start();
    fixer.Perform(fixRange);

    // Mesh the unified shape and emit SketchPlaneBounds
    IMeshTools_Parameters sketchPlaneMeshParams;
    sketchPlaneMeshParams.Deflection = diagonal * params.sketchPlaneLinearDeflection;
    sketchPlaneMeshParams.Angle = params.sketchPlaneAngularDeflection;
    sketchPlaneMeshParams.MinSize = params.sketchPlaneMinSize;
    
    BRepMesh_IncrementalMesh planeMesher(unifiedShape, sketchPlaneMeshParams);
    opencascade::handle<DeadlineProgressIndicator> meshProgress;
    {
        std::string label = "BRepMesh_IncrementalMesh for part " + protoName + ": ";
        meshProgress = new DeadlineProgressIndicator(std::chrono::milliseconds(params.sketchPlaneMeshTimeout), label);
    }
    Message_ProgressRange meshRange = meshProgress->Start();
    planeMesher.Perform(meshRange);

    int emittedPlaneCount = 0;
    int emittedPlaneTriangles = 0;
    int emptyTriangulationCount = 0;

    for (TopExp_Explorer faceExp(unifiedShape, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
        const TopoDS_Face& sketchFace = TopoDS::Face(faceExp.Current());

        TopLoc_Location sketchLoc;
        occt::handle<Poly_Triangulation> tri = BRep_Tool::Triangulation(sketchFace, sketchLoc);

        if (tri.IsNull() || tri->NbTriangles() == 0 || tri->NbNodes() == 0) {
            ++emptyTriangulationCount;
            continue;
        }

        ++emittedPlaneCount;
        emittedPlaneTriangles += tri->NbTriangles();

        const gp_Trsf sketchTrsf = sketchLoc.Transformation();
        const int basePoint = static_cast<int>(sketchPlanePoints.size());
        const int faceCountStart = static_cast<int>(sketchPlaneFaceVertexCounts.size());
        const int faceIndexStart = static_cast<int>(sketchPlaneFaceVertexIndices.size());
        const int normalStart = static_cast<int>(sketchPlaneNormals.size());

        for (int ni = 1; ni <= tri->NbNodes(); ++ni) {
            gp_Pnt p = tri->Node(ni).Transformed(sketchTrsf);
            sketchPlanePoints.push_back(GfVec3f(
                static_cast<float>(p.X()),
                static_cast<float>(p.Y()),
                static_cast<float>(p.Z())
            ));
        }

        const bool reversed = (sketchFace.Orientation() == TopAbs_REVERSED);
        for (int ti = 1; ti <= tri->NbTriangles(); ++ti) {
            int n1 = 0, n2 = 0, n3 = 0;
            tri->Triangle(ti).Get(n1, n2, n3);
            if (reversed) std::swap(n2, n3);

            const int i1 = basePoint + (n1 - 1);
            const int i2 = basePoint + (n2 - 1);
            const int i3 = basePoint + (n3 - 1);

            sketchPlaneFaceVertexCounts.push_back(3);
            sketchPlaneFaceVertexIndices.push_back(i1);
            sketchPlaneFaceVertexIndices.push_back(i2);
            sketchPlaneFaceVertexIndices.push_back(i3);

            const GfVec3f& p1 = sketchPlanePoints[i1];
            const GfVec3f& p2 = sketchPlanePoints[i2];
            const GfVec3f& p3 = sketchPlanePoints[i3];
            GfVec3f normal = GfCross(p2 - p1, p3 - p1);
            if (normal.GetLength() > 1e-10f) {
                normal.Normalize();
            } else {
                normal = GfVec3f(0.0f, 0.0f, 1.0f);
            }
            sketchPlaneNormals.push_back(normal);
            sketchPlaneNormals.push_back(normal);
            sketchPlaneNormals.push_back(normal);
        }

        const int pointCount = static_cast<int>(sketchPlanePoints.size()) - basePoint;
        const int faceCountCount = static_cast<int>(sketchPlaneFaceVertexCounts.size())  - faceCountStart;
        const int faceIndexCount = static_cast<int>(sketchPlaneFaceVertexIndices.size()) - faceIndexStart;
        const int normalCount = static_cast<int>(sketchPlaneNormals.size()) - normalStart;

        if (pointCount > 0 && faceCountCount > 0 &&
            faceIndexCount > 0 && normalCount == faceIndexCount) {
            sketchPlaneBounds.push_back({
                basePoint,      pointCount,
                faceCountStart, faceCountCount,
                faceIndexStart, faceIndexCount,
                normalStart,    normalCount
            });
        }
    }

    LOG_DEBUG(
        "  -> Sketch plane summary: closedWires=" + std::to_string(closedWireCount) +
        ", geomClosedWires=" + std::to_string(geomClosedWireCount) +
        ", makeFaceFailed=" + std::to_string(makeFaceFailedCount) +
        ", emptyTriangulations=" + std::to_string(emptyTriangulationCount) +
        ", emittedPlanes=" + std::to_string(emittedPlaneCount) +
        ", emittedTriangles=" + std::to_string(emittedPlaneTriangles)
    );
    LOG_DEBUG(
        "  -> Sketch plane output buffers: points=" +
        std::to_string(sketchPlanePoints.size()) +
        ", faceCounts=" +
        std::to_string(sketchPlaneFaceVertexCounts.size()) +
        ", faceIndices=" +
        std::to_string(sketchPlaneFaceVertexIndices.size())
    );

}

void SketchTessellationRoutine::tessellateSketch(
    const TopoDS_Shape& defShape, 
    const SdfPath& protoPath,
    SketchTessellationContext& ctx
) {
    if (params.sketchMode.type == TessParams::CurveType::None)
        return;

    for (const TopoDS_Edge& edge : ctx.freeEdges) {
        BRepAdaptor_Curve adaptor(edge);

        GCPnts_QuasiUniformDeflection sampler(
            adaptor, params.sketchDeflection,
            adaptor.FirstParameter(), adaptor.LastParameter()
        );
        if (!sampler.IsDone() || sampler.NbPoints() < 2) continue;

        int n = sampler.NbPoints();

        std::vector<float> arcValues = computeArcValues(sampler);

        if (params.sketchMode.type == TessParams::CurveType::Cubic) {
            // Phantom start — duplicate first point for Catmull-Rom
            gp_Pnt p0 = sampler.Value(1);
            sketchPoints.push_back(GfVec3f(p0.X(), p0.Y(), p0.Z()));
            sketchArcValues.push_back(0.0f);
            for (int si = 1; si <= n; ++si) {
                gp_Pnt p = sampler.Value(si);
                sketchPoints.push_back(GfVec3f(p.X(), p.Y(), p.Z()));
                sketchArcValues.push_back(arcValues[si - 1]);
            }
            // Phantom end — duplicate last point
            gp_Pnt pN = sampler.Value(n);
            sketchPoints.push_back(GfVec3f(pN.X(), pN.Y(), pN.Z()));
            sketchArcValues.push_back(1.0f);
            sketchCounts.push_back(n + 2);
        } else {
            // Linear and ResampledLinear both produce polylines
            for (int si = 1; si <= n; ++si) {
                gp_Pnt p = sampler.Value(si);
                sketchPoints.push_back(GfVec3f(p.X(), p.Y(), p.Z()));
                sketchArcValues.push_back(arcValues[si - 1]);
            }
            sketchCounts.push_back(n);
        }
    }
}

void SketchTessellationRoutine::applyUnitScale(const TessParams& params) {
    if (params.unitScale == 1.0) return;

    const float s = static_cast<float>(params.unitScale);
    auto scalePoints = [s](VtArray<GfVec3f>& points) {
        for (GfVec3f& p : points) {
            p *= s;
        }
    };
    scalePoints(sketchPoints);
    scalePoints(sketchPlanePoints);
}

// A definition is valid if it has mesh geometry OR sketch curves.
// Pure edge compounds (e.g. AP242 PMI annotation shapes) have no faces
// but do carry sketch curves, so only reject if both are absent.
bool SketchTessellationRoutine::hasValidGeometry() {
    return !(sketchCounts.empty() &&
             sketchPlaneFaceVertexIndices.empty());
}

bool SketchTessellationRoutine::tessellate(
    const TopoDS_Shape& defShape, 
    const TessParams& params,
    const SdfPath& protoPath
) {
    try {
        SketchTessellationContext ctx;
        initializeFreeEdges(defShape, ctx);
        tessellateSketchPlane(defShape, protoPath, ctx);
        tessellateSketch(defShape, protoPath, ctx);

        applyUnitScale(params);

        if (!hasValidGeometry()) {
            LOG_DEBUG("def produced no sketch curves in Shape");
            return false;
        }

    } catch (const Standard_Failure& e) {
        LOG_DEBUG(std::string("  -> Sketch plane reconstruction OCCT failure: ") + e.GetMessageString());
    } catch (const std::exception& e) {
        LOG_DEBUG(std::string("  -> Sketch plane reconstruction std failure: ") + e.what());
    } catch (...) {
        LOG_DEBUG("  -> Sketch plane reconstruction unknown failure");
    }

    return true;
}

bool SketchTessellationRoutine::definePrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessParams& params
) const {
    bool hasSketch = !sketchPoints.empty() && !sketchCounts.empty();
    bool hasSketchPlanes = !sketchPlaneBounds.empty();

    bool sketchDefined = true;
    if (hasSketch) {
        sketchDefined = defineSketchPrim(stage, protoPath, params);
    }

    bool sketchPlaneDefined = true;
    if (hasSketchPlanes) {
        sketchPlaneDefined = defineSketchPlanePrim(stage, protoPath, params);
    }

    return sketchDefined && sketchPlaneDefined;
}

bool SketchTessellationRoutine::writePrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessParams& params
) const {
    bool hasSketch = !sketchPoints.empty() && !sketchCounts.empty();
    bool hasSketchPlanes = !sketchPlaneBounds.empty();

    bool sketchWritten = true;
    if (hasSketch) {
        sketchWritten = writeSketchPrim(stage, protoPath, params);
    }

    bool sketchPlaneWritten = true;
    if (hasSketchPlanes) {
        sketchPlaneWritten = writeSketchPlanePrim(stage, protoPath, params);
    }

    return sketchWritten && sketchPlaneWritten;
}

void SketchTessellationRoutine::clearPrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath
) const {
    stage->RemovePrim(protoPath.AppendChild(TfToken("Sketch")));
    stage->RemovePrim(protoPath.AppendChild(TfToken("SketchPlane")));
}

size_t SketchTessellationRoutine::size() const {
    size_t sketchSize = sketchPoints.size();
    size_t sketchPlaneSize = sketchPlanePoints.size();
    return sketchSize + sketchPlaneSize;
}