#include <stddef.h>
#include <iostream>
#include <chrono>
#include <utility>
#include <algorithm>
#include <cmath>
#include <functional>
#include <initializer_list>
#include <unordered_map>
#include <unordered_set>
#include <atomic>
#include <limits>
#include <string>
#include <vector>

#include <opencascade/TDF_Label.hxx>
#include <opencascade/TopLoc_Location.hxx>
#include <opencascade/gp_Trsf.hxx>
#include <opencascade/ShapeFix_Shape.hxx>
#include <opencascade/BRepMesh_IncrementalMesh.hxx>
#include <opencascade/BRep_Tool.hxx>
#include <opencascade/TopExp_Explorer.hxx>
#include <opencascade/TopoDS.hxx>
#include <opencascade/Poly_Triangulation.hxx>
#include <opencascade/BRepAdaptor_Surface.hxx>
#include <opencascade/GeomLProp_SLProps.hxx>
#include <opencascade/BRepBndLib.hxx>
#include <opencascade/TopExp.hxx>
#include <opencascade/IMeshTools_Parameters.hxx>
#include <opencascade/BRepAdaptor_Curve.hxx>
#include <opencascade/GCPnts_QuasiUniformDeflection.hxx>
#include <opencascade/BRepExtrema_SelfIntersection.hxx>
#include <opencascade/BRepTools.hxx>
#include <opencascade/BRepExtrema_MapOfIntegerPackedMapOfInteger.hxx>
#include <opencascade/Bnd_Box.hxx>
#include <opencascade/GeomAbs_Shape.hxx>
#include <opencascade/GeomAdaptor_Surface.hxx>
#include <opencascade/NCollection_IndexedDataMap.hxx>
#include <opencascade/NCollection_IndexedMap.hxx>
#include <opencascade/NCollection_List.hxx>
#include <opencascade/Poly_PolygonOnTriangulation.hxx>
#include <opencascade/Poly_Triangle.hxx>
#include <opencascade/Standard_Failure.hxx>
#include <opencascade/Standard_Handle.hxx>
#include <opencascade/TopAbs_Orientation.hxx>
#include <opencascade/TopAbs_ShapeEnum.hxx>
#include <opencascade/TopTools_ShapeMapHasher.hxx>
#include <opencascade/TopoDS_Edge.hxx>
#include <opencascade/TopoDS_Face.hxx>
#include <opencascade/TopoDS_Shape.hxx>
#include <opencascade/gp_Pnt.hxx>
#include <opencascade/gp_Pnt2d.hxx>
#include <opencascade/gp_Vec.hxx>
#include <opencascade/Message_ProgressIndicator.hxx>
#include <opencascade/Message_ProgressRange.hxx>

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

#include "UsdStepExporter.h"
#include "Logger.h"

class Geom_Surface;

PXR_NAMESPACE_USING_DIRECTIVE

class DeadlineProgressIndicator : public Message_ProgressIndicator {
public:
    using Clock = std::chrono::steady_clock;
    using Duration = std::chrono::milliseconds;

    explicit DeadlineProgressIndicator(Duration timeout)
        : _deadline(Clock::now() + timeout)
        , _timedOut(false)
    {}

    // OCCT polls this at internal checkpoints
    Standard_Boolean UserBreak() override {
        if (Clock::now() >= _deadline) {
            _timedOut = true;
            return true;
        }
        return false;
    }

    void Show(const Message_ProgressScope&, const Standard_Boolean) override {/*do nothing*/}

    bool timedOut() const { return _timedOut; }

private:
    std::chrono::time_point<Clock> _deadline;
    bool _timedOut;
};

bool UsdStepExporter::tessellatePart(
    TessResult& result, 
    const TopoDS_Shape& defShape, 
    const TessParams& params,
    bool mesherInParallel
) {
    
    using Clock = std::chrono::high_resolution_clock;
    using Seconds = std::chrono::duration<double>;

    auto tessellateStart = Clock::now();

    LOG_DEBUG("  -> tessellatePart: ShapeFix_Shape (Repair pass)");
    ShapeFix_Shape fixer(defShape);
    fixer.SetPrecision(1e-4);
    fixer.SetMaxTolerance(0.1);
    
    opencascade::handle<DeadlineProgressIndicator> fixProgress = new DeadlineProgressIndicator(std::chrono::milliseconds(params.fixTimeout));
    Message_ProgressRange fixRange = fixProgress->Start();
    fixer.Perform(fixRange);

    if (fixProgress->timedOut()) {
        LOG_DEBUG("  -> ShapeFix_Shape timed out, proceeding with partial repair");
        // workingShape is still usable
        // OCCT leaves it in a partially-fixed state
    }
    TopoDS_Shape workingShape = fixer.Shape();

    LOG_DEBUG("  -> tessellatePart: BRepTools::Clean");
    BRepTools::Clean(workingShape); // remove previously created tessellations for this part 

    Bnd_Box bbox;
    LOG_DEBUG("  -> tessellatePart: BRepBndLib::Add");
    BRepBndLib::Add(workingShape, bbox);
    double xmin, ymin, zmin, xmax, ymax, zmax;
    LOG_DEBUG("  -> tessellatePart: bbox.Get");
    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);

    double diagonal = std::sqrt(
        std::pow(xmax - xmin, 2) + 
        std::pow(ymax - ymin, 2) + 
        std::pow(zmax - zmin, 2)
    );

    result.renderOnly = params.renderPurposeThreshold != std::numeric_limits<float>::infinity() && diagonal < params.renderPurposeThreshold;

    IMeshTools_Parameters meshParams;
    meshParams.InParallel = mesherInParallel; 
    meshParams.Deflection = static_cast<float>(diagonal * params.meshLinearDeflection);
    meshParams.Angle = params.meshAngularDeflection; // in radians
    meshParams.MinSize = meshParams.Deflection * params.meshMinSize;
    
    LOG_DEBUG("  -> tessellatePart: BRepMesh_IncrementalMesh");
    BRepMesh_IncrementalMesh mesher(workingShape, meshParams);

    LOG_DEBUG("  -> tessellatePart: mesher.Perform()");
    opencascade::handle<DeadlineProgressIndicator> meshProgress = new DeadlineProgressIndicator(std::chrono::milliseconds(params.meshTimeout));
    Message_ProgressRange meshRange = meshProgress->Start();
    mesher.Perform(meshRange);

    if (meshProgress->timedOut()) {
        LOG_DEBUG("  -> BRepMesh_IncrementalMesh timed out"); // Some faces will have null triangulations
    }

    int maxPasses = params.maxNumberRemeshPasses;
    IMeshTools_Parameters repairParams = meshParams;

    // repeat check for self-intersections 
    // if fail, refine the mesh until there are none 
    // or we hit the max pass count.

    // important note this is on the whole shape 
    // not just the intersected shapes, 
    // so the edge walk later works

    LOG_DEBUG("  -> tessellatePart: Starting remesh passes (" + std::to_string(maxPasses) + ")");
    for (int pass = 0; pass < maxPasses; ++pass) {
        LOG_DEBUG("  Running self-intersection check (pass " + std::to_string(pass) + ")");
        BRepExtrema_SelfIntersection checker(workingShape, params.selfIntersectionThreshold);
        LOG_DEBUG("  -> checker.Perform()");
        checker.Perform();

        if (!checker.IsDone()) {
            LOG_DEBUG("  -> checker not done, breaking");
            break;
        }

        LOG_DEBUG("  -> checker getting OverlapElements");
        const BRepExtrema_MapOfIntegerPackedMapOfInteger& overlaps = checker.OverlapElements();

        if (overlaps.IsEmpty()) {
            LOG_DEBUG("  No interesections found.");
            break;
        }
        
        LOG_DEBUG("  Found overlaps. Remeshing with finer parameters.");

        repairParams.Deflection *= 0.5;
        repairParams.Angle *= 0.5;

        LOG_DEBUG("  -> BRepTools::Clean (repair)");
        BRepTools::Clean(workingShape); 
        LOG_DEBUG("  -> BRepMesh_IncrementalMesh (repair)");
        BRepMesh_IncrementalMesh remesher(workingShape, repairParams);
        
        opencascade::handle<DeadlineProgressIndicator> remeshProgress = new DeadlineProgressIndicator(std::chrono::milliseconds(params.remeshTimeout));
        Message_ProgressRange remeshRange = remeshProgress->Start();
        remesher.Perform(remeshRange);
        if (remeshProgress->timedOut()) {
            LOG_DEBUG("  -> BRepMesh_IncrementalMesh (repair) timed out");
            break;
        }
    }
    

    auto meshEnd = Clock::now();
    LOG_DEBUG("  Mesh time: " + std::to_string(Seconds(meshEnd - tessellateStart).count()) + " s");
    
    LOG_DEBUG("  -> tessellatePart: Edge walk preparation");
    // normals will be faceVarying: result.normals.size() == result.faceVertexIndices.size()

    // positions remain welded via topology
    // Poly_Triangululation is a tessellated representation of 
    // the Topo_DS_whatever. For a given face, we need to store 
    // it alongside the index into the source face array
    // so its:
    // Face A at idx 2 shares a triangle with Face C at idx 5 
    using TriNodeKey = std::pair<const Poly_Triangulation*, int>;
    struct PairHash {
        size_t operator()(const TriNodeKey& k) const {
            return std::hash<const void*>{}(k.first) ^ (std::hash<int>{}(k.second) << 16);
        }
    };

    using Edge = std::pair<int,int>;
    struct EdgeHash {
        size_t operator()(const Edge& e) const {
            auto [a, b] = e.first < e.second 
                ? std::make_pair(e.first, e.second) 
                : std::make_pair(e.second, e.first);
            return std::hash<int>{}(a) ^ (std::hash<int>{}(b) << 16);
        }
    };

    std::unordered_map<TriNodeKey, int, PairHash> nodeToCanonical;
    std::unordered_set<TriNodeKey, PairHash> boundaryKeys;
    std::unordered_set<int> boundaryNodes;

    std::unordered_map<TriNodeKey, TriNodeKey, PairHash> nodeAlias;

    auto resolveAlias = [&](TriNodeKey key) -> TriNodeKey {
        int limit = 32; // guard against degenerate cycles
        while (nodeAlias.count(key) && --limit > 0)
            key = nodeAlias[key];
        return key;
    };

    // edge walk to unify boundary nodes
    NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> faceMap;
    TopExp::MapShapes(workingShape, TopAbs_FACE, faceMap);

    NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher> edgeToFaces;
    TopExp::MapShapesAndAncestors(workingShape, TopAbs_EDGE, TopAbs_FACE, edgeToFaces);

    // Per-edge data deferred for Linear mode. 
    struct DeferredLinearCurve {
        std::vector<TriNodeKey> keys;
        int continuity;
    };
    
    std::vector<DeferredLinearCurve> deferredLinearCurves;

    for (TopExp_Explorer edgeExp(workingShape, TopAbs_EDGE); edgeExp.More(); edgeExp.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(edgeExp.Current());
        if (BRep_Tool::Degenerated(edge)) continue;

        int edgeIdx = edgeToFaces.FindIndex(edge);
        if (edgeIdx == 0) continue;

        const NCollection_List<TopoDS_Shape>& adjFaces = edgeToFaces.FindFromIndex(edgeIdx);

        struct FacePoly {
            TopoDS_Face face;
            occt::handle<Poly_Triangulation> tri;
            occt::handle<Poly_PolygonOnTriangulation> poly;
            TopLoc_Location loc;
            int surfaceIndex;
        };

        std::vector<FacePoly> facePolys;

        for (NCollection_List<TopoDS_Shape>::Iterator iter(adjFaces); iter.More(); iter.Next()) {
            const TopoDS_Face& face = TopoDS::Face(iter.Value());
            TopLoc_Location loc;
            occt::handle<Poly_Triangulation> tri = BRep_Tool::Triangulation(face, loc);
            if (tri.IsNull()) continue;

            TopLoc_Location edgeLoc;
            occt::handle<Poly_PolygonOnTriangulation> poly = BRep_Tool::PolygonOnTriangulation(edge, tri, edgeLoc);
            if (poly.IsNull()) continue;

            int surfaceIndex = faceMap.FindIndex(face);
            facePolys.push_back({face, tri, poly, loc, surfaceIndex});
        }

        if (facePolys.empty()) continue;

        bool isSurfaceBoundary = false;
        for (size_t fi = 1; fi < facePolys.size(); fi++) {
            if (facePolys[fi].surfaceIndex != facePolys[0].surfaceIndex) {
                isSurfaceBoundary = true;
                break;
            }
        }

        int continuity = 0;
        if (facePolys.size() >= 2) {
            GeomAbs_Shape continuityType = BRep_Tool::Continuity(
                edge,
                facePolys[0].face,
                facePolys[1].face
            );
            switch (continuityType) {
                case GeomAbs_C0: continuity = 1; break;
                case GeomAbs_G1: continuity = 2; break;
                case GeomAbs_C1: continuity = 3; break;
                case GeomAbs_G2: continuity = 4; break;
                case GeomAbs_C2: continuity = 5; break;
                case GeomAbs_CN: continuity = 6; break;
                default: break;
            }
        }

        const FacePoly& canonical = facePolys[0];
        int numNodes = canonical.poly->NbNodes();

        // Ensure we don't exceed the minimum node count across all adjacent faces
        for (size_t fi = 1; fi < facePolys.size(); fi++) {
            numNodes = std::min(numNodes, facePolys[fi].poly->NbNodes());
        }

        std::vector<TriNodeKey> edgeCanonicalKeys;
        if (isSurfaceBoundary && params.wireframeMode.sampling == CurveSampling::Underlying)
            edgeCanonicalKeys.reserve(numNodes);

        try {
            for (int k = 1; k <= numNodes; k++) {
                int canonicalNode = canonical.poly->Node(k);
                TriNodeKey canonKey = {canonical.tri.get(), canonicalNode};
                TriNodeKey resolvedCanon = resolveAlias(canonKey);

                // Output indices are assigned later in the face loop.
                boundaryKeys.insert(resolvedCanon);

                for (size_t fi = 1; fi < facePolys.size(); fi++) {
                    int otherNode = facePolys[fi].poly->Node(k);
                    TriNodeKey otherKey = {facePolys[fi].tri.get(), otherNode};
                    TriNodeKey resolvedOther = resolveAlias(otherKey);
                    boundaryKeys.insert(resolvedOther);

                    // Alias the other face's node to the canonical representative
                    // so that the face loop emits a single shared vertex for both.
                    if (resolvedOther != resolvedCanon) {
                        nodeAlias[resolvedOther] = resolvedCanon;
                    }
                }

                if (isSurfaceBoundary && params.wireframeMode.sampling == CurveSampling::Underlying)
                    edgeCanonicalKeys.push_back(canonKey);
            }
        } catch (const Standard_Failure& e) {
            std::cerr << "OCCT Exception in face iteration: " << e.GetMessageString() << "\n";
        } catch (const std::exception& e) {
            std::cerr << "Standard Exception in face iteration: " << e.what() << "\n";
        } catch (...) {
            std::cerr << "Unknown exception in face iteration\n";
        }

        if (isSurfaceBoundary && params.wireframeMode.type != CurveType::None) {
            if (params.wireframeMode.sampling == CurveSampling::Underlying) {
                if (edgeCanonicalKeys.size() >= 2)
                    deferredLinearCurves.push_back({std::move(edgeCanonicalKeys), continuity});
            } else {
                BRepAdaptor_Curve adaptor(edge);
                GCPnts_QuasiUniformDeflection sampler(adaptor, params.wireframeDeflection, adaptor.FirstParameter(), adaptor.LastParameter());

                if (sampler.IsDone() && sampler.NbPoints() >= 2) {
                    int n = sampler.NbPoints();

                    if (params.wireframeMode.type == CurveType::Cubic) {
                        // Add phantom start for Catmull-Rom interpolation
                        gp_Pnt p0 = sampler.Value(1);
                        result.curvePoints.push_back(GfVec3f(p0.X(), p0.Y(), p0.Z()));

                        for (int i = 1; i <= n; ++i) {
                            gp_Pnt p = sampler.Value(i);
                            result.curvePoints.push_back(GfVec3f(p.X(), p.Y(), p.Z()));
                        }

                        // Add phantom end
                        gp_Pnt pN = sampler.Value(n);
                        result.curvePoints.push_back(GfVec3f(pN.X(), pN.Y(), pN.Z()));

                        result.wireframeCounts.push_back(n + 2);
                    } else {
                        // ResampledLinear
                        for (int i = 1; i <= n; ++i) {
                            gp_Pnt p = sampler.Value(i);
                            result.curvePoints.push_back(GfVec3f(p.X(), p.Y(), p.Z()));
                        }
                        result.wireframeCounts.push_back(n);
                    }

                    result.curveContinuity.push_back(continuity);
                }
            }
        }
    }

    auto edgeWalkEnd = Clock::now();
    LOG_DEBUG("  Edge-walk time: " + std::to_string(Seconds(edgeWalkEnd - meshEnd).count()) + " s");

    // sketches in Step 242 are registered as free edges 
    // in the defintion shape and are not guaranteed to be connected to any faces, 
    // so we have to do a separate edge walk to find them and sample 
    if (params.sketchMode.type != CurveType::None) {
        for (TopExp_Explorer edgeExp(workingShape, TopAbs_EDGE); edgeExp.More(); edgeExp.Next()) {
            const TopoDS_Edge& edge = TopoDS::Edge(edgeExp.Current());
            if (BRep_Tool::Degenerated(edge)) continue;

            int edgeIdx = edgeToFaces.FindIndex(edge);
            bool isFreeEdge = (edgeIdx == 0) || (edgeToFaces.FindFromIndex(edgeIdx).Extent() == 0);
            if (!isFreeEdge) continue;

            BRepAdaptor_Curve adaptor(edge);

            double deflection;
            
            if (params.sketchMode.type == CurveType::Linear) {
                deflection = params.wireframeDeflection;
            } else {
                deflection = params.sketchDeflection;
            }

            GCPnts_QuasiUniformDeflection sampler(
                adaptor, deflection,
                adaptor.FirstParameter(), adaptor.LastParameter()
            );
            if (!sampler.IsDone() || sampler.NbPoints() < 2) continue;

            int n = sampler.NbPoints();

            if (params.sketchMode.type == CurveType::Cubic) {
                // Phantom start — duplicate first point for Catmull-Rom
                gp_Pnt p0 = sampler.Value(1);
                result.sketchPoints.push_back(GfVec3f(p0.X(), p0.Y(), p0.Z()));
                for (int si = 1; si <= n; ++si) {
                    gp_Pnt p = sampler.Value(si);
                    result.sketchPoints.push_back(GfVec3f(p.X(), p.Y(), p.Z()));
                }
                // Phantom end — duplicate last point
                gp_Pnt pN = sampler.Value(n);
                result.sketchPoints.push_back(GfVec3f(pN.X(), pN.Y(), pN.Z()));
                result.sketchCounts.push_back(n + 2);
            } else {
                // Linear and ResampledLinear both produce polylines
                for (int si = 1; si <= n; ++si) {
                    gp_Pnt p = sampler.Value(si);
                    result.sketchPoints.push_back(GfVec3f(p.X(), p.Y(), p.Z()));
                }
                result.sketchCounts.push_back(n);
            }
        }
    }

    int totalTris = 0, totalNodes = 0;
    for (TopExp_Explorer faceExp(workingShape, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
        const TopoDS_Face& face = TopoDS::Face(faceExp.Current());
        TopLoc_Location loc;
        auto tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull()) continue;
        totalTris += tri->NbTriangles();
        totalNodes += tri->NbNodes();
    }

    result.faceVertexCounts.reserve(totalTris);
    result.faceVertexIndices.reserve(totalTris * 3);
    result.normals.reserve(totalTris * 3);
    result.surfaceIDs.reserve(totalTris);
    std::vector<UVPatch> uvPatches;
    uvPatches.reserve(faceMap.Extent());
    result.surfaceIDBounds.reserve(faceMap.Extent());

    int surfaceBoundIdx = 0; // used to track 'global' idx for geom subsets 

    // weld positions, emit faceVarying normals.
    for (TopExp_Explorer faceExp(workingShape, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
        const TopoDS_Face& face = TopoDS::Face(faceExp.Current());
        int surfaceIndex = faceMap.FindIndex(face);
        TopLoc_Location loc;
        occt::handle<Poly_Triangulation> tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull()) continue;

        TessResult::SurfaceIDBounds surfaceBounds = { surfaceBoundIdx, surfaceBoundIdx + tri->NbTriangles(), surfaceIndex };
        result.surfaceIDBounds.push_back(surfaceBounds);
        surfaceBoundIdx += tri->NbTriangles();

        gp_Trsf trsf = loc.Transformation();

        for (int j = 1; j <= tri->NbNodes(); j++) {
            TriNodeKey key = resolveAlias({tri.get(), j});
            if (nodeToCanonical.count(key)) continue;

            gp_Pnt p = tri->Node(j).Transformed(trsf);
            int idx = static_cast<int>(result.points.size());
            result.points.push_back(GfVec3f(
                static_cast<float>(p.X()),
                static_cast<float>(p.Y()),
                static_cast<float>(p.Z())
            ));
            nodeToCanonical[key] = idx;

            if (boundaryKeys.count(key))
                boundaryNodes.insert(idx);
        }

        bool reversed = (face.Orientation() == TopAbs_REVERSED);

        BRepAdaptor_Surface adapter(face);
        float uMin = adapter.FirstUParameter();
        float uMax = adapter.LastUParameter();
        float vMin = adapter.FirstVParameter();
        float vMax = adapter.LastVParameter();

        bool hasUV = tri->HasUVNodes();

        GeomAdaptor_Surface adapterSurface = adapter.Surface();
        occt::handle<Geom_Surface> geomSurface = adapterSurface.Surface();

        UVPatch patch;
        patch.uMin = uMin; patch.uMax = uMax;
        patch.vMin = vMin; patch.vMax = vMax;

        for (int j = 1; j <= tri->NbTriangles(); j++) {
            int n1, n2, n3;
            tri->Triangle(j).Get(n1, n2, n3);
            if (reversed) std::swap(n2, n3);

            int i1 = nodeToCanonical[resolveAlias({tri.get(), n1})];
            int i2 = nodeToCanonical[resolveAlias({tri.get(), n2})];
            int i3 = nodeToCanonical[resolveAlias({tri.get(), n3})];

            result.faceVertexCounts.push_back(3);
            result.faceVertexIndices.push_back(i1);
            result.faceVertexIndices.push_back(i2);
            result.faceVertexIndices.push_back(i3);
            result.surfaceIDs.push_back(surfaceIndex);

            for (int localIdx : {n1, n2, n3}) {
                GfVec3f normal(0.0f, 0.0f, 1.0f);
                float u = 0.0f, v = 0.0f;

                if (hasUV) {
                    gp_Pnt2d uv = tri->UVNode(localIdx);
                    u = std::clamp(static_cast<float>(uv.X()), uMin, uMax);
                    v = std::clamp(static_cast<float>(uv.Y()), vMin, vMax);
                }

                GeomLProp_SLProps props(geomSurface, u, v, 1, 1e-6);

                constexpr bool useParametricNormals = true;

                if (useParametricNormals && props.IsNormalDefined()) {
                    gp_Vec n = props.Normal();
                    GfVec3f raw(
                        static_cast<float>(n.X()),
                        static_cast<float>(n.Y()),
                        static_cast<float>(n.Z())
                    );
                    float len = raw.GetLength();
                    if (len > 1e-10f) normal = raw / len;
                } else {
                    gp_Pnt p1 = tri->Node(n1).Transformed(trsf);
                    gp_Pnt p2 = tri->Node(n2).Transformed(trsf);
                    gp_Pnt p3 = tri->Node(n3).Transformed(trsf);
                    gp_Vec e1(p1, p2), e2(p1, p3);
                    gp_Vec faceNormal = e1.Crossed(e2);
                    if (faceNormal.Magnitude() > 1e-10)
                        faceNormal.Normalize();
                    normal = GfVec3f(
                        static_cast<float>(faceNormal.X()),
                        static_cast<float>(faceNormal.Y()),
                        static_cast<float>(faceNormal.Z())
                    );
                }

                if (reversed) normal = -normal;

                result.normals.push_back(normal);
                patch.uvs.push_back(GfVec2f(u, v)); // raw param coords, packed later
            }
        }
        uvPatches.push_back(std::move(patch));
    }

    result.perSurfaceUVs = packUVAtlas(uvPatches);

    result.isBoundaryVertex.resize(result.points.size(), false);
    for (int idx : boundaryNodes)
        result.isBoundaryVertex[idx] = true;

    for (const DeferredLinearCurve& deferred : deferredLinearCurves) {
        std::vector<int> resolved;
        resolved.reserve(deferred.keys.size());
        for (const TriNodeKey& key : deferred.keys) {
            auto it = nodeToCanonical.find(resolveAlias(key));
            if (it != nodeToCanonical.end())
                resolved.push_back(it->second);
        }
        if (resolved.size() >= 2) {
            if (params.wireframeMode.type == CurveType::Cubic) {
                // Phantom start — duplicate first point
                result.curvePoints.push_back(result.points[resolved.front()]);
            }
            for (int idx : resolved)
                result.curvePoints.push_back(result.points[idx]);
            if (params.wireframeMode.type == CurveType::Cubic) {
                // Phantom end — duplicate last point
                result.curvePoints.push_back(result.points[resolved.back()]);
                result.wireframeCounts.push_back(static_cast<int>(resolved.size()) + 2);
            } else {
                result.wireframeCounts.push_back(static_cast<int>(resolved.size()));
            }
            result.curveContinuity.push_back(deferred.continuity);
        }
    }

    if (result.faceVertexIndices.empty()) {
        LOG_DEBUG("  -> tessellatePart: faceVertexIndices is empty, clearing points and boundary flags to avoid confusion downstream");
        result.points.clear();
        result.isBoundaryVertex.clear();
    } else if (!result.points.empty()) {
        LOG_DEBUG("  -> tessellatePart: Welding vertices and reindexing faces");
        std::vector<int> oldToNew(result.points.size(), -1);
        VtArray<GfVec3f> newPoints;
        newPoints.reserve(result.points.size());
        VtArray<bool> newIsBoundary;
        if (!result.isBoundaryVertex.empty()) {
            newIsBoundary.reserve(result.points.size());
        }

        for (int index : result.faceVertexIndices) {
            if (oldToNew[index] == -1) {
                oldToNew[index] = newPoints.size();
                newPoints.push_back(result.points[index]);
                if (!result.isBoundaryVertex.empty()) {
                    newIsBoundary.push_back(result.isBoundaryVertex[index]);
                }
            }
        }

        for (int& index : result.faceVertexIndices) {
            index = oldToNew[index];
        }

        result.points = std::move(newPoints);
        if (!result.isBoundaryVertex.empty()) {
            result.isBoundaryVertex = std::move(newIsBoundary);
        }
    }

    auto faceProcessEnd = Clock::now();
    LOG_DEBUG("  Face processing time: " + std::to_string(Seconds(faceProcessEnd - edgeWalkEnd).count()) + " s");

    auto tessellateEnd = Clock::now();
    LOG_DEBUG("  Total tessellatePart time: " + std::to_string(Seconds(tessellateEnd - tessellateStart).count()) + " s");

    // A definition is valid if it has mesh geometry OR sketch curves.
    // Pure edge compounds (e.g. AP242 PMI annotation shapes) have no faces
    // but do carry sketch curves, so only reject if both are absent.
    if (result.points.empty() && result.sketchCounts.empty() && result.wireframeCounts.empty()) {
        LOG_DEBUG("def produced no geometry, wireframeCounts, sketch curves in Shape");
        return false;
    }

    return true;
}

void UsdStepExporter::tessellateGeometry(
    std::vector<TessellationJob>& tessJobs,
    const std::vector<std::pair<TDF_Label, TopoDS_Shape>>& defs,
    const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
    const SdfPath& containerPrimPath
) {
    std::vector<int> defIndices(defs.size());
    for (int i = 0; i < defs.size(); ++i) defIndices[i] = i;

    // Pre-calculate complexity in the form of
    // face counts for better load balancing
    std::vector<int> shapeComplexity(defs.size(), 0);
    for (int i = 0; i < defs.size(); ++i) {
        int complexity = 0;
        for (TopExp_Explorer faceExp(defs[i].second, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
            complexity++;
        }
        shapeComplexity[i] = complexity;
    }

    // Sort descending by complexity so heaviest jobs start first
    std::sort(defIndices.begin(), defIndices.end(), [&shapeComplexity](int a, int b) {
        return shapeComplexity[a] > shapeComplexity[b];
    });

    {
        LOG_SCOPED_TIMER("Parallel Tessellation of " + std::to_string(defIndices.size()) + " definition indices.");
        std::atomic<int> completedJobs{0};
        const int totalJobs = static_cast<int>(tessJobs.size());
        
        WorkParallelForEach(defIndices.begin(), defIndices.end(), [&](int idx) {
            LOG_DEBUG("Starting processing for definition index: " + std::to_string(idx) + " / " + std::to_string(defIndices.size()));
            for (TessellationJob& job : tessJobs) {
                if (job.defIndex != idx) continue;

                bool bTessellate = isPrototypeActiveInFilter(selectedPaths, containerPrimPath, job.proto->variantSetName, job.proto->variantName, job.prototypePath);
                
                if (bTessellate) {
                    if (job.runMesherInParallel) {
                        LOG_DEBUG("Job for " + job.prototypePath.GetString() + " is set to run mesher in mesherInParallel model");
                    }
                    LOG_DEBUG("Tessellating part: " + job.prototypePath.GetString() + " (def index " + std::to_string(idx) + ")");
                    try {
                        tessellatePart(job.result, defs[idx].second, job.params, job.runMesherInParallel);
                        int currentCount = ++completedJobs;
                        LOG_DEBUG("Finished tessellating: " + job.prototypePath.GetString() + " | Faces: " + std::to_string(job.result.faceVertexCounts.size()) + " (" + std::to_string(currentCount) + "/" + std::to_string(totalJobs) + " jobs completed globally)");
                        LOG_PROGRESS(currentCount, totalJobs, "Tessellating Geometry");
                    } catch (const Standard_Failure& e) {
                        LOG_PROGRESS_DONE();
                        LOG_ERR("OCC exception on " + job.prototypePath.GetString() + "(def index " + std::to_string(idx) + ")" + ": " + e.GetMessageString());
                        int currentCount = ++completedJobs;
                        LOG_PROGRESS(currentCount, totalJobs, "Tessellating Geometry");
                    } catch (const std::exception& e) {
                        LOG_PROGRESS_DONE();
                        LOG_ERR("std exception on " + job.prototypePath.GetString() + "(def index " + std::to_string(idx) + ")" + ": " + e.what());
                        int currentCount = ++completedJobs;
                        LOG_PROGRESS(currentCount, totalJobs, "Tessellating Geometry");
                    } catch (...) {
                        LOG_PROGRESS_DONE();
                        LOG_ERR("Unknown exception on " + job.prototypePath.GetString() + "(def index " + std::to_string(idx) + ")");
                        int currentCount = ++completedJobs;
                        LOG_PROGRESS(currentCount, totalJobs, "Tessellating Geometry");
                    }
                } else {
                    LOG_DEBUG("Skipping tessellation for inactive part: " + job.prototypePath.GetString());
                    int currentCount = ++completedJobs;
                    LOG_PROGRESS(currentCount, totalJobs, "Tessellating Geometry");
                }
            }
            LOG_DEBUG("Thread finished with definition index: " + std::to_string(idx) + " / " + std::to_string(defIndices.size()));
        });
        
        LOG_PROGRESS_DONE();
    }    
}