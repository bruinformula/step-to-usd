
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

#include <TDF_Label.hxx>
#include <TopLoc_Location.hxx>
#include <gp_Trsf.hxx>
#include <ShapeFix_Shape.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
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

#include "StepUSD/StepUsdPipeline.h"
#include "StepUSD/Logger.h"

class Geom_Surface;

PXR_NAMESPACE_USING_DIRECTIVE

class DeadlineProgressIndicator : public Message_ProgressIndicator {
public:
    using Clock = std::chrono::steady_clock;
    using Duration = std::chrono::milliseconds;

    explicit DeadlineProgressIndicator(Duration timeout, std::string partLabel) : 
        _deadline(Clock::now() + timeout),
        _timedOut(false),
        label(partLabel) 
    {
    }

    ~DeadlineProgressIndicator() {
        if (_timedOut) {
            LOG_WARN(label + "DeadlineProgressIndicator destroyed after timeout");
        } else {
            // LOG_DEBUG(label + "DeadlineProgressIndicator destroyed (completed within deadline)");
        }
    }

    // OCCT polls this at internal checkpoints
    Standard_Boolean UserBreak() override {
        if (Clock::now() >= _deadline) {
            if (!_timedOut) {  // Only log once on first timeout detection
                LOG_WARN(label + "operation timed out");
            }
            _timedOut = true;
            return true;
        }
        // LOG_DEBUG(label + "UserBreak polled, still within deadline");
        return false;
    }

    void Show(const Message_ProgressScope& scope, const bool isForced) override {
        // LOG_DEBUG(label + "Show called (forced=" + std::string(isForced ? "true" : "false") + ")");
    }

    bool timedOut() const { return _timedOut; }

private:
    std::chrono::time_point<Clock> _deadline;
    bool _timedOut;
    std::string label;
};

// UVs
struct UVPatch {
    std::vector<GfVec2f> uvs; // one per face-vertex, in raw param space
    float uMin, uMax, vMin, vMax;
    float worldW, worldH; // world-space extents of this face's nodes
};

static VtArray<GfVec2f> packUVAtlas(std::vector<UVPatch>& patches) {
    
    int n = (int)patches.size();

    // Each patch's needs to have normalized UVs to local [0,1]
    std::vector<float> tileWidths(n), tileHeights(n);
    for (int i = 0; i < n; i++) {
        float uRange = std::max(patches[i].uMax - patches[i].uMin, 1e-10f);
        float vRange = std::max(patches[i].vMax - patches[i].vMin, 1e-10f);
        for (auto& uv : patches[i].uvs) {
            uv[0] = (uv[0] - patches[i].uMin) / uRange;
            uv[1] = (uv[1] - patches[i].vMin) / vRange;
        }
        // Size tiles by world-space extent
        tileWidths[i]  = patches[i].worldW;
        tileHeights[i] = patches[i].worldH;
    }

    // Scale so patches roughly tile a unit square
    float invSqrtN = 1.0f / std::sqrt((float)std::max(n, 1));
    for (int i = 0; i < n; i++) { 
        tileWidths[i] *= invSqrtN; 
        tileHeights[i] *= invSqrtN; 
    }

    // sorting
    std::vector<int> order(n);
    std::iota(order.begin(), order.end(), 0); // fills an array 0,1,2,3...
    std::sort(
        order.begin(), 
        order.end(), 
        [&](int a, int b) { 
            return tileHeights[a] > tileHeights[b]; 
        }
    );

    constexpr float padding = 0.001f;
    std::vector<GfVec4f> placements(n); // (x, y, w, h)
    float shelfX = 0, shelfY = 0, shelfH = 0;
    float atlasW = 0, atlasH = 0;

    for (int idx : order) {
        if (shelfX + tileWidths[idx] > 1.0f + 1e-5f) {
            shelfY += shelfH + padding;
            shelfX = 0;
            shelfH = 0;
        }
        placements[idx] = GfVec4f(shelfX, shelfY, tileWidths[idx], tileHeights[idx]);
        shelfX += tileWidths[idx] + padding;
        shelfH = std::max(shelfH, tileHeights[idx]);
        atlasW = std::max(atlasW, shelfX);
        atlasH = std::max(atlasH, shelfY + shelfH);
    }

    atlasW = std::max(atlasW, 1e-10f);
    atlasH = std::max(atlasH, 1e-10f);

    int totalFaceVerts = 0;
    for (auto& p : patches) totalFaceVerts += (int)p.uvs.size();

    VtArray<GfVec2f> result(totalFaceVerts);
    int offset = 0;
    for (int i = 0; i < n; i++) {
        float px = placements[i][0] / atlasW;
        float py = placements[i][1] / atlasH;
        float pw = placements[i][2] / atlasW;
        float ph = placements[i][3] / atlasH;
        for (const auto& uv : patches[i].uvs)
            result[offset++] = GfVec2f(px + uv[0] * pw, py + uv[1] * ph);
    }
    return result;
}
/*
bool StepUsdPipeline::tessellatePart(
    TessResult& result, 
    const TopoDS_Shape& defShape, 
    const TessParams& params,
    const SdfPath& protoPath,
    bool mesherInParallel
) {
    
    using Clock = std::chrono::high_resolution_clock;
    using Seconds = std::chrono::duration<double>;

    auto tessellateStart = Clock::now();






    auto tessellateEnd = Clock::now();
    LOG_DEBUG("  Total tessellatePart time: " + std::to_string(Seconds(tessellateEnd - tessellateStart).count()) + " s");

    return true;
}
*/


bool StepUsdPipeline::tessellatePart(
    TessResult& result, 
    const TopoDS_Shape& defShape, 
    const TessParams& params,
    const SdfPath& protoPath,
    bool mesherInParallel
) {
    
    using Clock = std::chrono::high_resolution_clock;
    using Seconds = std::chrono::duration<double>;

    auto tessellateStart = Clock::now();

    std::string protoName = protoPath.GetAsString();

    LOG_DEBUG("  -> tessellatePart: ShapeFix_Shape (Repair pass)");
    ShapeFix_Shape fixer(defShape);
    fixer.SetPrecision(params.meshFixPrecision);
    fixer.SetMaxTolerance(params.meshFixTolerance);
    
    opencascade::handle<DeadlineProgressIndicator> fixProgress;
    {
        std::string label = "ShapeFix_Shape for part " + protoName + ": ";
        fixProgress = new DeadlineProgressIndicator(std::chrono::milliseconds(params.meshFixTimeout), label);
    } 
    Message_ProgressRange fixRange = fixProgress->Start();
    fixer.Perform(fixRange);

    if (fixProgress->timedOut()) {
        LOG_DEBUG("  -> ShapeFix_Shape timed out, proceeding with partial repair");
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

    result.renderOnly = params.renderPurposeThreshold != std::numeric_limits<double>::infinity() && diagonal < params.renderPurposeThreshold;

    IMeshTools_Parameters meshParams;
    meshParams.InParallel = mesherInParallel; 
    meshParams.Deflection = diagonal * params.meshLinearDeflection;
    meshParams.Angle = params.meshAngularDeflection; // in radians
    meshParams.MinSize = meshParams.Deflection * params.meshMinSize;
    
    LOG_DEBUG("  -> tessellatePart: BRepMesh_IncrementalMesh");
    BRepMesh_IncrementalMesh mesher(workingShape, meshParams);

    LOG_DEBUG("  -> tessellatePart: mesher.Perform()");
    opencascade::handle<DeadlineProgressIndicator> meshProgress;
    {
        std::string label = "BRepMesh_IncrementalMesh for part " + protoName + ": ";
        meshProgress = new DeadlineProgressIndicator(std::chrono::milliseconds(params.meshMeshTimeout), label);
    }
    Message_ProgressRange meshRange = meshProgress->Start();
    mesher.Perform(meshRange);

    if (meshProgress->timedOut()) {
        LOG_DEBUG("  -> BRepMesh_IncrementalMesh timed out"); // Some faces will have null triangulations
    }

    int maxPasses = params.meshMaxNumberRemeshPasses;
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
        BRepExtrema_SelfIntersection checker(workingShape, params.meshSelfIntersectionThreshold);
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
        
        opencascade::handle<DeadlineProgressIndicator> remeshProgress;
        {
            std::string label = "BRepMesh_IncrementalMesh for part " + protoName + ": ";
            remeshProgress = new DeadlineProgressIndicator(std::chrono::milliseconds(params.meshRemeshTimeout), label);
        }
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

    auto computeArcValues = [](const GCPnts_QuasiUniformDeflection& sampler) -> std::vector<float> {
        int n = sampler.NbPoints();
        std::vector<float> arc(n, 0.0f);
        if (n < 2) return arc;

        float total = 0.0f;
        for (int i = 2; i <= n; ++i)
            total += static_cast<float>(sampler.Value(i - 1).Distance(sampler.Value(i)));

        float cum = 0.0f;
        for (int i = 2; i <= n; ++i) {
            cum += static_cast<float>(sampler.Value(i - 1).Distance(sampler.Value(i)));
            arc[i - 1] = (total > 1e-10f) ? cum / total : 1.0f;
        }
        arc[n - 1] = 1.0f; // guard against float drift
        return arc;
    };

    // Per-edge data deferred for Linear mode. 
    struct DeferredCurve {
        std::vector<TriNodeKey> keys;
        int continuity;
    };

    std::vector<DeferredCurve> deferredCurves;

    std::unordered_map<TriNodeKey, GfVec3f, PairHash> tangentAccum;
    std::unordered_map<TriNodeKey, int, PairHash> tangentCount;

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
        GeomAbs_Shape continuityType;
        if (facePolys.size() >= 2) {
            continuityType = BRep_Tool::Continuity(
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
        if (isSurfaceBoundary && params.wireframeMode.sampling == TessParams::CurveSampling::Underlying)
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

                if (isSurfaceBoundary && params.wireframeMode.sampling == TessParams::CurveSampling::Underlying)
                    edgeCanonicalKeys.push_back(canonKey);
            }
        } catch (const Standard_Failure& e) {
            std::cerr << "OCCT Exception in face iteration: " << e.GetMessageString() << "\n";
        } catch (const std::exception& e) {
            std::cerr << "Standard Exception in face iteration: " << e.what() << "\n";
        } catch (...) {
            std::cerr << "Unknown exception in face iteration\n";
        }

        if (canonical.poly->HasParameters()) {
            BRepAdaptor_Curve curveAdaptor(edge);
            const opencascade::handle<TColStd_HArray1OfReal>& edgeParams = canonical.poly->Parameters();

            for (int k = 1; k <= numNodes; ++k) {
                gp_Pnt pt;
                gp_Vec dv;
                try {
                    curveAdaptor.D1(edgeParams->Array1()(k), pt, dv);
                } catch (const Standard_Failure&) {
                    continue;
                }

                const double mag = dv.Magnitude();
                if (mag < 1e-10) continue;

                const GfVec3f tan(
                    static_cast<float>(dv.X() / mag),
                    static_cast<float>(dv.Y() / mag),
                    static_cast<float>(dv.Z() / mag)
                );

                // Use the same resolved canonical key so junction vertices
                // accumulate contributions from every edge that shares them.
                int canonicalNode = canonical.poly->Node(k);
                TriNodeKey canonKey = resolveAlias({canonical.tri.get(), canonicalNode});
                tangentAccum[canonKey] += tan;
                tangentCount [canonKey]++;
            }
        }

        auto sampleNearestSurfaceNormal = [&](const TopoDS_Face& face, const gp_Pnt& p) -> GfVec3f {
            TopLoc_Location loc;
            occt::handle<Geom_Surface> surf = BRep_Tool::Surface(face, loc);

            if (surf.IsNull()) return {0.0f, 0.0f, 0.0f}; 

            gp_Pnt localP = p.Transformed(loc.Inverted());

            // Project the local point onto the surface to find nearest UV coordinates
            GeomAPI_ProjectPointOnSurf projector(localP, surf);

            if (projector.NbPoints() > 0) {
                double u, v;
                projector.LowerDistanceParameters(u, v);

                // Evaluate surface properties at the projected point uv 
                GeomLProp_SLProps props(surf, u, v, 1, Precision::Confusion());

                if (props.IsNormalDefined()) {
                    gp_Dir normal = props.Normal();
                    gp_Pnt point = props.Value();

                    if (face.Orientation() == TopAbs_REVERSED) normal.Reverse(); 

                    normal.Transform(loc.Transformation());

                    return GfVec3f(
                        static_cast<float>(normal.X()),
                        static_cast<float>(normal.Y()),
                        static_cast<float>(normal.Z())
                    );
                }
            }

            return {0.0f, 0.0f, 0.0f}; 
        };

        if (isSurfaceBoundary && params.wireframeMode.type != TessParams::CurveType::None) {
            if (params.wireframeMode.sampling == TessParams::CurveSampling::Underlying) {
                if (edgeCanonicalKeys.size() >= 2)
                    deferredCurves.push_back({std::move(edgeCanonicalKeys), continuity});
            } else {
                BRepAdaptor_Curve adaptor(edge);
                GCPnts_QuasiUniformDeflection sampler(adaptor, params.wireframeDeflection, adaptor.FirstParameter(), adaptor.LastParameter());

                VtArray<GfVec3f> wireframeSurfaceNormals;

                if (params.wireframeEmbedSurfaceNormals) {
                    int n = sampler.NbPoints();
                    // initialize to zero
                    wireframeSurfaceNormals.assign(n, GfVec3f(0.f, 0.f, 0.f));

                    for (NCollection_List<TopoDS_Shape>::Iterator iter(adjFaces); iter.More(); iter.Next()) {
                        const TopoDS_Face& face = TopoDS::Face(iter.Value());
                        for (int si = 1; si <= n; ++si) {
                            gp_Pnt p = sampler.Value(si);
                            wireframeSurfaceNormals[si - 1] += sampleNearestSurfaceNormal(face, p);
                        }
                    }

                    // normalize each accumulated sum
                    for (GfVec3f& n : wireframeSurfaceNormals) {
                        float len = n.GetLength();
                        if (len > 1e-6f) n /= len;
                        else n = GfVec3f(0.f, 0.f, 1.f);
                    }
                }

                LOG_DEBUG("  -> Wireframe sampling: " + std::to_string(sampler.NbPoints()) + " points");

                if (params.wireframeEmbedSurfaceNormals) {
                    LOG_DEBUG("  -> Wireframe surface normals sampled: " + std::to_string(wireframeSurfaceNormals.size()));
                }

                if (sampler.IsDone() && sampler.NbPoints() >= 2) {
                    int n = sampler.NbPoints();
                    std::vector<float> arcValues = computeArcValues(sampler);

                    if (params.wireframeMode.type == TessParams::CurveType::Cubic) {
                        // Add phantom start for Catmull-Rom interpolation
                        gp_Pnt p0 = sampler.Value(1);
                        result.wireframeArcValues.push_back(0.0f);
                        
                        result.wireframePoints.push_back(GfVec3f(p0.X(), p0.Y(), p0.Z()));
                        if (params.wireframeEmbedSurfaceNormals) result.wireframeSurfaceNormals.push_back(wireframeSurfaceNormals[0]);

                        for (int i = 1; i <= n; ++i) {
                            gp_Pnt p = sampler.Value(i);
                            result.wireframePoints.push_back(GfVec3f(p.X(), p.Y(), p.Z()));
                            result.wireframeArcValues.push_back(arcValues[i - 1]);
                            if (params.wireframeEmbedSurfaceNormals) result.wireframeSurfaceNormals.push_back(wireframeSurfaceNormals[i - 1]);
                        }

                        // Add phantom end
                        gp_Pnt pN = sampler.Value(n);
                        result.wireframePoints.push_back(GfVec3f(pN.X(), pN.Y(), pN.Z()));
                        result.wireframeArcValues.push_back(1.0f);
                        if (params.wireframeEmbedSurfaceNormals) result.wireframeSurfaceNormals.push_back(wireframeSurfaceNormals[n - 1]);

                        result.wireframeCounts.push_back(n + 2);
                    } else {
                        // ResampledLinear
                        for (int i = 1; i <= n; ++i) {
                            gp_Pnt p = sampler.Value(i);
                            result.wireframePoints.push_back(GfVec3f(p.X(), p.Y(), p.Z()));
                            result.wireframeArcValues.push_back(arcValues[i - 1]);
                            if (params.wireframeEmbedSurfaceNormals) result.wireframeSurfaceNormals.push_back(wireframeSurfaceNormals[i - 1]);
                        }
                        result.wireframeCounts.push_back(n);
                    }

                    result.wireframeContinuity.push_back(continuity);
                }
            }
        }
    }

    auto edgeWalkEnd = Clock::now();
    LOG_DEBUG("  Edge-walk time: " + std::to_string(Seconds(edgeWalkEnd - meshEnd).count()) + " s");

    std::vector<TopoDS_Edge> freeEdges;
    freeEdges.reserve(edgeToFaces.Extent());
    for (TopExp_Explorer edgeExp(workingShape, TopAbs_EDGE); edgeExp.More(); edgeExp.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(edgeExp.Current());
        if (BRep_Tool::Degenerated(edge)) continue;

        int edgeIdx = edgeToFaces.FindIndex(edge);
        bool isFreeEdge = (edgeIdx == 0) || (edgeToFaces.FindFromIndex(edgeIdx).Extent() == 0);
        if (isFreeEdge) {
            freeEdges.push_back(edge);
        }
    }
    LOG_DEBUG("  -> Sketch plane input: freeEdges=" + std::to_string(freeEdges.size()));

    // sketches in Step 242 are registered as free edges 
    // in the defintion shape and are not guaranteed to be connected to any faces, 
    // so we have to do a separate edge walk to find them and sample 
    if (params.sketchMode.type != TessParams::CurveType::None) {
        for (const TopoDS_Edge& edge : freeEdges) {

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
                result.sketchPoints.push_back(GfVec3f(p0.X(), p0.Y(), p0.Z()));
                result.sketchArcValues.push_back(0.0f);
                for (int si = 1; si <= n; ++si) {
                    gp_Pnt p = sampler.Value(si);
                    result.sketchPoints.push_back(GfVec3f(p.X(), p.Y(), p.Z()));
                    result.sketchArcValues.push_back(arcValues[si - 1]);
                }
                // Phantom end — duplicate last point
                gp_Pnt pN = sampler.Value(n);
                result.sketchPoints.push_back(GfVec3f(pN.X(), pN.Y(), pN.Z()));
                result.sketchArcValues.push_back(1.0f);
                result.sketchCounts.push_back(n + 2);
            } else {
                // Linear and ResampledLinear both produce polylines
                for (int si = 1; si <= n; ++si) {
                    gp_Pnt p = sampler.Value(si);
                    result.sketchPoints.push_back(GfVec3f(p.X(), p.Y(), p.Z()));
                    result.sketchArcValues.push_back(arcValues[si - 1]);
                }
                result.sketchCounts.push_back(n);
            }
        }
    }

    // Build sketch planes from free edges using OCCT topology:
    // split all free edges at intersections, connect split edges into wires,
    // convert closed wires into planar faces, then triangulate those faces.
    

    // TODO: Support Muliplanar sketches
    if (!freeEdges.empty()) {
        try {
            // Assemble free edges into a compound and connect them into wires
            TopoDS_Compound freeEdgeCompound;
            BRep_Builder builder;
            builder.MakeCompound(freeEdgeCompound);
            for (const TopoDS_Edge& edge : freeEdges) {
                builder.Add(freeEdgeCompound, edge);
            }

            TopoDS_Shape splitShape = freeEdgeCompound;
            {
                BOPAlgo_Splitter splitter;
                for (const TopoDS_Edge& edge : freeEdges) {
                    splitter.AddArgument(edge);
                }
                
                splitter.Perform();
                if (!splitter.HasErrors()) { 
                    splitShape = splitter.Shape();
                } else {
                    std::ostringstream e;
                    splitter.DumpErrors(e);
                    LOG_DEBUG("  -> Sketch plane splitter reported error:" + e.str() + ". using unsplit free edges");
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
                if (vFirst.IsNull() || vLast.IsNull()) return false;
                if (vFirst.IsSame(vLast))               return true;
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
                const bool geomClosed = isWireGeometricallyClosed(wire, edgeTolerance * 10.0);
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

                builder.Add(faceCompound, f);
                ++builtFaceCount;
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
                const int basePoint = static_cast<int>(result.sketchPlanePoints.size());
                const int faceCountStart = static_cast<int>(result.sketchPlaneFaceVertexCounts.size());
                const int faceIndexStart = static_cast<int>(result.sketchPlaneFaceVertexIndices.size());
                const int normalStart = static_cast<int>(result.sketchPlaneNormals.size());

                for (int ni = 1; ni <= tri->NbNodes(); ++ni) {
                    gp_Pnt p = tri->Node(ni).Transformed(sketchTrsf);
                    result.sketchPlanePoints.push_back(GfVec3f(
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

                    result.sketchPlaneFaceVertexCounts.push_back(3);
                    result.sketchPlaneFaceVertexIndices.push_back(i1);
                    result.sketchPlaneFaceVertexIndices.push_back(i2);
                    result.sketchPlaneFaceVertexIndices.push_back(i3);

                    const GfVec3f& p1 = result.sketchPlanePoints[i1];
                    const GfVec3f& p2 = result.sketchPlanePoints[i2];
                    const GfVec3f& p3 = result.sketchPlanePoints[i3];
                    GfVec3f normal = GfCross(p2 - p1, p3 - p1);
                    if (normal.GetLength() > 1e-10f) {
                        normal.Normalize();
                    } else {
                        normal = GfVec3f(0.0f, 0.0f, 1.0f);
                    }
                    result.sketchPlaneNormals.push_back(normal);
                    result.sketchPlaneNormals.push_back(normal);
                    result.sketchPlaneNormals.push_back(normal);
                }

                const int pointCount = static_cast<int>(result.sketchPlanePoints.size()) - basePoint;
                const int faceCountCount = static_cast<int>(result.sketchPlaneFaceVertexCounts.size())  - faceCountStart;
                const int faceIndexCount = static_cast<int>(result.sketchPlaneFaceVertexIndices.size()) - faceIndexStart;
                const int normalCount = static_cast<int>(result.sketchPlaneNormals.size()) - normalStart;

                if (pointCount > 0 && faceCountCount > 0 &&
                    faceIndexCount > 0 && normalCount == faceIndexCount) {
                    result.sketchPlaneBounds.push_back({
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
                std::to_string(result.sketchPlanePoints.size()) +
                ", faceCounts=" +
                std::to_string(result.sketchPlaneFaceVertexCounts.size()) +
                ", faceIndices=" +
                std::to_string(result.sketchPlaneFaceVertexIndices.size())
            );

        } catch (const Standard_Failure& e) {
            LOG_DEBUG(std::string("  -> Sketch plane reconstruction OCCT failure: ") +
                    e.GetMessageString());
        } catch (const std::exception& e) {
            LOG_DEBUG(std::string("  -> Sketch plane reconstruction std failure: ") + e.what());
        } catch (...) {
            LOG_DEBUG("  -> Sketch plane reconstruction unknown failure");
        }
    }
    
    // Get a total of the number of triangles and nodes 
    // across all faces to reserve output buffer sizes up front.
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
    std::vector<UVPatch> uvPatches;
    uvPatches.reserve(faceMap.Extent());
    result.surfaceIDBounds.reserve(faceMap.Extent());

    int surfaceBoundIdx = 0; // used to track 'global' idx for geom subsets 

    // Accumulated per-vertex normals for wireframe use.
    // Sized and zeroed after the face loop once result.points.size() is known.
    std::vector<GfVec3f> pointNormalAccum;
    std::vector<int> pointNormalCount;

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
        float wxMin= std::numeric_limits<float>::max(), wxMax=-std::numeric_limits<float>::max();
        float wyMin= std::numeric_limits<float>::max(), wyMax=-std::numeric_limits<float>::max();
        float wzMin= std::numeric_limits<float>::max(), wzMax=-std::numeric_limits<float>::max();

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

            wxMin = std::min(wxMin, (float)p.X()); wxMax = std::max(wxMax, (float)p.X());
            wyMin = std::min(wyMin, (float)p.Y()); wyMax = std::max(wyMax, (float)p.Y());
            wzMin = std::min(wzMin, (float)p.Z()); wzMax = std::max(wzMax, (float)p.Z());

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

        // Tangent frame for hasUV=false fallback
        gp_Vec faceTangentU(1,0,0), faceTangentV(0,1,0), faceNormalVec(0,0,1);
        {
            GeomLProp_SLProps props(geomSurface,
                (uMin+uMax)*0.5, (vMin+vMax)*0.5, 1, 1e-6);
            if (props.IsNormalDefined()) {
                faceNormalVec = gp_Vec(props.Normal());
                faceTangentU  = gp_Vec(1,0,0);
                if (std::abs(faceNormalVec.Dot(faceTangentU)) > 0.9)
                    faceTangentU = gp_Vec(0,1,0);
                faceTangentV = faceNormalVec.Crossed(faceTangentU).Normalized();
                faceTangentU = faceTangentV.Crossed(faceNormalVec).Normalized();
            }
        }
        gp_Pnt faceCentroid(0,0,0);
        for (int j = 1; j <= tri->NbNodes(); j++) {
            gp_Pnt p = tri->Node(j).Transformed(trsf);
            faceCentroid.SetX(faceCentroid.X() + p.X());
            faceCentroid.SetY(faceCentroid.Y() + p.Y());
            faceCentroid.SetZ(faceCentroid.Z() + p.Z());
        }
        faceCentroid.SetX(faceCentroid.X() / tri->NbNodes());
        faceCentroid.SetY(faceCentroid.Y() / tri->NbNodes());
        faceCentroid.SetZ(faceCentroid.Z() / tri->NbNodes());

        UVPatch patch;
        patch.uMin =  std::numeric_limits<float>::max();
        patch.uMax = -std::numeric_limits<float>::max();
        patch.vMin =  std::numeric_limits<float>::max();
        patch.vMax = -std::numeric_limits<float>::max();
        float dims[3] = { wxMax-wxMin, wyMax-wyMin, wzMax-wzMin };
        std::sort(dims, dims+3, std::greater<float>());
        patch.worldW = std::max(dims[0], 1e-10f);
        patch.worldH = std::max(dims[1], 1e-10f);

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

            for (int localIdx : {n1, n2, n3}) {
                GfVec3f normal(0.0f, 0.0f, 1.0f);
                float u = 0.0f, v = 0.0f;

                if (hasUV) {
                    gp_Pnt2d uv = tri->UVNode(localIdx);
                    u = static_cast<float>(uv.X());
                    v = static_cast<float>(uv.Y());
                } else {
                    int canonIdx = nodeToCanonical[resolveAlias({tri.get(), localIdx})];
                    const GfVec3f& wp = result.points[canonIdx];
                    gp_Vec offset(
                        wp[0] - faceCentroid.X(),
                        wp[1] - faceCentroid.Y(),
                        wp[2] - faceCentroid.Z()
                    );
                    u = static_cast<float>(offset.Dot(faceTangentU));
                    v = static_cast<float>(offset.Dot(faceTangentV));
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
                patch.uvs.push_back(GfVec2f(u, v));
                patch.uMin = std::min(patch.uMin, u);
                patch.uMax = std::max(patch.uMax, u);
                patch.vMin = std::min(patch.vMin, v);
                patch.vMax = std::max(patch.vMax, v);      

                int canonIdx = nodeToCanonical[resolveAlias({tri.get(), localIdx})];
                if (canonIdx >= (int)pointNormalAccum.size()) {
                    pointNormalAccum.resize(canonIdx + 1, GfVec3f(0.f, 0.f, 0.f));
                    pointNormalCount.resize(canonIdx + 1, 0);
                }
                pointNormalAccum[canonIdx] += normal;
                pointNormalCount[canonIdx]++;
            }
        }
        uvPatches.push_back(std::move(patch));
    }
    result.perSurfaceUVs = packUVAtlas(uvPatches);

    result.isBoundaryVertex.resize(result.points.size(), false);
    for (int idx : boundaryNodes)
        result.isBoundaryVertex[idx] = true;

    result.boundaryTangents.assign(result.points.size(), GfVec3f(0.0f, 0.f, 0.f));
    for (auto& [key, accum] : tangentAccum) {
        auto it = nodeToCanonical.find(resolveAlias(key));
        if (it == nodeToCanonical.end()) continue;

        const int idx = it->second;
        if (idx < 0 || idx >= (int)result.points.size()) continue;

        const float len = accum.GetLength();
        if (len > 1e-10f) {
            result.boundaryTangents[idx] = accum / len;
        } else {
            result.boundaryTangents[idx] = GfVec3f(0.0f, 0.0f, 0.0f);
        }
    }

    VtArray<GfVec3f> pointNormals(pointNormalAccum.size(), GfVec3f(0.0f, 0.0f, 1.0f));
    if (params.wireframeEmbedSurfaceNormals && params.wireframeMode.sampling == TessParams::CurveSampling::Underlying) {
        for (int i = 0; i < (int)pointNormalAccum.size(); ++i) {
            if (pointNormalCount[i] > 0) {
                GfVec3f avg = pointNormalAccum[i]; // sum, not yet divided
                float len = avg.GetLength();
                pointNormals[i] = (len > 1e-6f) ? avg / len : GfVec3f(0.f, 0.f, 1.f);
            }
        }
    }

    for (const DeferredCurve& deferred : deferredCurves) {
        std::vector<int> resolved;
        resolved.reserve(deferred.keys.size());
        for (const TriNodeKey& key : deferred.keys) {
            auto it = nodeToCanonical.find(resolveAlias(key));
            if (it != nodeToCanonical.end())
                resolved.push_back(it->second);
        }
        if (resolved.size() >= 2) {
            std::vector<float> arcValues(resolved.size(), 0.0f);
            {
                float total = 0.0f;
                for (size_t i = 1; i < resolved.size(); ++i)
                    total += (result.points[resolved[i]] - result.points[resolved[i-1]]).GetLength();
                float cum = 0.0f;
                for (size_t i = 1; i < resolved.size(); ++i) {
                    cum += (result.points[resolved[i]] - result.points[resolved[i-1]]).GetLength();
                    arcValues[i] = (total > 1e-10f) ? cum / total : 1.0f;
                }
                arcValues.back() = 1.0f;
            }

            if (params.wireframeMode.type == TessParams::CurveType::Cubic) {
                // Phantom start — duplicate first point
                result.wireframePoints.push_back(result.points[resolved.front()]);
                result.wireframeArcValues.push_back(0.0f);
                if (params.wireframeEmbedSurfaceNormals) 
                    result.wireframeSurfaceNormals.push_back(pointNormals[resolved.front()]);
            }
            for (size_t i = 0; i < resolved.size(); ++i) {
                result.wireframePoints.push_back(result.points[resolved[i]]);
                result.wireframeArcValues.push_back(arcValues[i]);
                if (params.wireframeEmbedSurfaceNormals)
                    result.wireframeSurfaceNormals.push_back(pointNormals[resolved[i]]);
            }
            if (params.wireframeMode.type == TessParams::CurveType::Cubic) {
                // Phantom end — duplicate last point
                result.wireframePoints.push_back(result.points[resolved.back()]);
                result.wireframeArcValues.push_back(1.0f);
                result.wireframeCounts.push_back(static_cast<int>(resolved.size()) + 2);
                if (params.wireframeEmbedSurfaceNormals) 
                    result.wireframeSurfaceNormals.push_back(pointNormals[resolved.back()]);
            } else {
                result.wireframeCounts.push_back(static_cast<int>(resolved.size()));
            }
            result.wireframeContinuity.push_back(deferred.continuity);
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
        VtArray<GfVec3f> newBoundaryTangents;
        if (!result.isBoundaryVertex.empty()) {
            newIsBoundary.reserve(result.points.size());
        }

        if (!result.boundaryTangents.empty()) {
            newBoundaryTangents.reserve(result.points.size());
        }

        for (int index : result.faceVertexIndices) {
            if (oldToNew[index] == -1) {
                oldToNew[index] = newPoints.size();
                newPoints.push_back(result.points[index]);
                if (!result.isBoundaryVertex.empty()) {
                    newIsBoundary.push_back(result.isBoundaryVertex[index]);
                }
                if (!result.boundaryTangents.empty()) {
                    newBoundaryTangents.push_back(result.boundaryTangents[index]);
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
        if (!result.boundaryTangents.empty()) {
            result.boundaryTangents = std::move(newBoundaryTangents);
        }
    }

    auto faceProcessEnd = Clock::now();
    LOG_DEBUG("  Face processing time: " + std::to_string(Seconds(faceProcessEnd - edgeWalkEnd).count()) + " s");

    auto tessellateEnd = Clock::now();
    LOG_DEBUG("  Total tessellatePart time: " + std::to_string(Seconds(tessellateEnd - tessellateStart).count()) + " s");

    if (params.unitScale != 1.0) {
        const float s = static_cast<float>(params.unitScale);
        auto scalePoints = [s](VtArray<GfVec3f>& points) {
            for (GfVec3f& p : points) {
                p *= s;
            }
        };
        scalePoints(result.points);
        scalePoints(result.wireframePoints);
        scalePoints(result.sketchPoints);
        scalePoints(result.sketchPlanePoints);
    }

    // A definition is valid if it has mesh geometry OR sketch curves.
    // Pure edge compounds (e.g. AP242 PMI annotation shapes) have no faces
    // but do carry sketch curves, so only reject if both are absent.
    if (result.points.empty() &&
        result.sketchCounts.empty() &&
        result.wireframeCounts.empty() &&
        result.sketchPlaneFaceVertexIndices.empty()) {
        LOG_DEBUG("def produced no geometry, wireframeCounts, sketch curves in Shape");
        return false;
    }

    return true;
}


struct ShapeKey {
    const void* tshape;

    bool operator==(const ShapeKey& other) const {
        return tshape == other.tshape;
    }
};

struct ShapeKeyHash {
    size_t operator()(const ShapeKey& k) const {
        return std::hash<const void*>{}(k.tshape);
    }
};
static bool tessParamsEqual(const TessParams& a, const TessParams& b) {
    return std::memcmp(&a, &b, sizeof(TessParams)) == 0;
}

struct ParamSubgroup {
    TessParams params;
    std::vector<TessellationJob*> jobs;
    bool runMesherInParallel;
};

void StepUsdPipeline::tessellateGeometry(
    std::vector<TessellationJob>& tessJobs,
    const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths
) {
    std::unordered_map<ShapeKey, std::vector<TessellationJob*>, ShapeKeyHash> jobsByShape;
    for (TessellationJob& job : tessJobs) {
        const auto& defs = job.proto->model->getDefinitionShapes();
        const TopoDS_Shape& shape = defs[job.defIndex].second;
        jobsByShape[ShapeKey{shape.TShape().get()}].push_back(&job);
    }

    // Pre-calculate complexity (face count) per unique shape for load balancing.
    std::vector<ShapeKey> shapeKeys;
    shapeKeys.reserve(jobsByShape.size());
    std::unordered_map<ShapeKey, int, ShapeKeyHash> shapeComplexity;
    shapeComplexity.reserve(jobsByShape.size());

    for (const auto& [key, jobs] : jobsByShape) {
        shapeKeys.push_back(key);
        const TessellationJob* rep = jobs.front();
        const auto& defs = rep->proto->model->getDefinitionShapes();
        const TopoDS_Shape& shape = defs[rep->defIndex].second;
        int complexity = 0;
        for (TopExp_Explorer faceExp(shape, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
            complexity++;
        }
        shapeComplexity[key] = complexity;
    }

    // Sort descending by complexity so heaviest jobs start first
    std::sort(shapeKeys.begin(), shapeKeys.end(), [&shapeComplexity](const ShapeKey& a, const ShapeKey& b) {
        return shapeComplexity.at(a) > shapeComplexity.at(b);
    });

    {
        LOG_SCOPED_TIMER("Parallel Tessellation of " + std::to_string(shapeKeys.size()) + " unique shapes.");
        std::atomic<int> completedJobs{0};
        const int totalJobs = static_cast<int>(tessJobs.size());

        WorkParallelForEach(shapeKeys.begin(), shapeKeys.end(), [&](const ShapeKey& key) {
            const std::vector<TessellationJob*>& jobs = jobsByShape.at(key);

            const TessellationJob* rep = jobs.front();
            const auto& defs = rep->proto->model->getDefinitionShapes();
            const TopoDS_Shape& shape = defs[rep->defIndex].second;

            std::vector<ParamSubgroup> subgroups;

            for (TessellationJob* jobPtr : jobs) {
                TessellationJob& job = *jobPtr;

                bool bTessellate = isPrototypeActiveInFilter(
                    selectedPaths, job.prototypePath, job.proto->variantSetName, job.proto->variantName);

                if (!bTessellate) {
                    int currentCount = ++completedJobs;
                    LOG_PROGRESS(currentCount, totalJobs, "Tessellating Geometry");
                    continue;
                }

                bool merged = false;
                for (ParamSubgroup& sg : subgroups) {
                    if (tessParamsEqual(sg.params, job.params)) {
                        sg.jobs.push_back(jobPtr);
                        sg.runMesherInParallel = sg.runMesherInParallel || job.runMesherInParallel;
                        merged = true;
                        break;
                    }
                }
                if (!merged) {
                    subgroups.push_back(ParamSubgroup{job.params, {jobPtr}, job.runMesherInParallel});
                }
            }

            for (ParamSubgroup& sg : subgroups) {
                TessellationJob& primary = *sg.jobs.front();

                if (sg.runMesherInParallel) {
                    LOG_DEBUG("Job for " + primary.prototypePath.GetString() + " is set to run mesher in mesherInParallel mode");
                }
                LOG_DEBUG("Tessellating part: " + primary.prototypePath.GetString() +
                          " (def index " + std::to_string(primary.defIndex) +
                          ", shared by " + std::to_string(sg.jobs.size()) + " prototype path(s))");

                try {
                    tessellatePart(primary.result, shape, sg.params, primary.prototypePath, sg.runMesherInParallel);

                    // Fan the computed result out to every other job that needs
                    // this exact (shape, params) pair.
                    for (size_t i = 1; i < sg.jobs.size(); ++i) {
                        sg.jobs[i]->result = primary.result;
                    }

                    for (TessellationJob* jobPtr : sg.jobs) {
                        int currentCount = ++completedJobs;
                        LOG_DEBUG("Finished tessellating: " + jobPtr->prototypePath.GetString() +
                                  " | Faces: " + std::to_string(jobPtr->result.faceVertexCounts.size()) +
                                  " (" + std::to_string(currentCount) + "/" + std::to_string(totalJobs) + " jobs completed globally)");
                        LOG_PROGRESS(currentCount, totalJobs, "Tessellating Geometry");
                    }
                } catch (const Standard_Failure& e) {
                    LOG_PROGRESS_DONE();
                    for (TessellationJob* jobPtr : sg.jobs) {
                        LOG_ERR("OCC exception on " + jobPtr->prototypePath.GetString() +
                                " (def index " + std::to_string(jobPtr->defIndex) + "): " + e.GetMessageString());
                        int currentCount = ++completedJobs;
                        LOG_PROGRESS(currentCount, totalJobs, "Tessellating Geometry");
                    }
                } catch (const std::exception& e) {
                    LOG_PROGRESS_DONE();
                    for (TessellationJob* jobPtr : sg.jobs) {
                        LOG_ERR("std exception on " + jobPtr->prototypePath.GetString() +
                                " (def index " + std::to_string(jobPtr->defIndex) + "): " + e.what());
                        int currentCount = ++completedJobs;
                        LOG_PROGRESS(currentCount, totalJobs, "Tessellating Geometry");
                    }
                } catch (...) {
                    LOG_PROGRESS_DONE();
                    for (TessellationJob* jobPtr : sg.jobs) {
                        LOG_ERR("Unknown exception on " + jobPtr->prototypePath.GetString() +
                                " (def index " + std::to_string(jobPtr->defIndex) + ")");
                        int currentCount = ++completedJobs;
                        LOG_PROGRESS(currentCount, totalJobs, "Tessellating Geometry");
                    }
                }
            }
        });

        LOG_PROGRESS_DONE();
    }
}