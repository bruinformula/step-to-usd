#include <chrono>
#include <utility>
#include <algorithm>
#include <cmath>
#include <functional>
#include <initializer_list>
#include <unordered_map>
#include <unordered_set>
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

#include "CadUSD/Logger.h"
#include "CadUSD/Tessellation/TessellationRoutine.h"
#include "CadUSD/Tessellation/TessellationUtils.h"

class Geom_Surface;

PXR_NAMESPACE_USING_DIRECTIVE

using Clock = std::chrono::high_resolution_clock;
using Seconds = std::chrono::duration<double>;

static GfVec3f sampleNearestSurfaceNormal(const TopoDS_Face& face, const gp_Pnt& p) {
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

struct TriNodeKey {
    const Poly_Triangulation* first;
    int second;

    struct Hash {
        size_t operator()(const TriNodeKey& k) const {
            return std::hash<const void*>{}(k.first) ^ (std::hash<int>{}(k.second) << 16);
        }
    };
    bool operator==(const TriNodeKey& k) const {
        return this->first == k.first && this->second == k.second;
    }

    bool operator!=(const TriNodeKey& k) const {
        return !(*this == k);
    }
};


struct DeferredCurve {
    std::vector<TriNodeKey> keys;
    int continuity;
};

struct TangentAccum {
    GfVec3f sum{0.f, 0.f, 0.f};
    int count = 0;
};

// Everything that gets built up in one phase and consumed in a later one,
// bundled so helper functions don't need a dozen out-parameters apiece.
struct MeshTessellationContext {
    NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> faceMap;
    NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher> edgeToFaces;

    std::unordered_map<TriNodeKey, int, TriNodeKey::Hash> nodeToCanonical;
    std::unordered_set<TriNodeKey, TriNodeKey::Hash> boundaryKeys;
    std::unordered_set<int> boundaryNodes;
    std::unordered_map<TriNodeKey, TriNodeKey, TriNodeKey::Hash> nodeAlias;

    std::vector<DeferredCurve> deferredCurves;
    
    std::unordered_map<TriNodeKey, TangentAccum, TriNodeKey::Hash> tangentAccum;

    std::vector<UVPatch> uvPatches;
    std::vector<GfVec3f> pointNormalAccum;
    std::vector<int> pointNormalCount;
};

// Follows alias chains built during the edge walk so every node on a shared
// edge resolves to the same canonical (triangulation*, node) key.
TriNodeKey resolveAlias(
    const std::unordered_map<TriNodeKey, TriNodeKey, TriNodeKey::Hash>& nodeAlias,
    TriNodeKey key
) {
    int limit = 32; // guard against degenerate cycles
    auto it = nodeAlias.find(key);
    while (it != nodeAlias.end() && --limit > 0) {
        key = it->second;
        it = nodeAlias.find(key);
    }
    return key;
}

// Samples a resampled (non-"Underlying") boundary curve and appends it to
// wireframe*. Lifted out of the edge-walk loop unchanged.
void MeshTessellationRoutine::emitResampledWireframeCurve(
    const TopoDS_Edge& edge,
    const NCollection_List<TopoDS_Shape>& adjFaces,
    int continuity,
    const TessParams& params
) {
    BRepAdaptor_Curve adaptor(edge);
    GCPnts_QuasiUniformDeflection sampler(adaptor, params.wireframeDeflection, adaptor.FirstParameter(), adaptor.LastParameter());

    VtArray<GfVec3f> wireframeSurfaceNormals;
    
    if (params.wireframeEmbedSurfaceNormals) {
        int n = sampler.NbPoints();
        wireframeSurfaceNormals.assign(n, GfVec3f(0.f, 0.f, 0.f));

        for (NCollection_List<TopoDS_Shape>::Iterator iter(adjFaces); iter.More(); iter.Next()) {
            const TopoDS_Face& face = TopoDS::Face(iter.Value());
            for (int si = 1; si <= n; ++si) {
                gp_Pnt p = sampler.Value(si);
                wireframeSurfaceNormals[si - 1] += sampleNearestSurfaceNormal(face, p);
            }
        }

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

    if (!sampler.IsDone() || sampler.NbPoints() < 2) return;

    int n = sampler.NbPoints();
    std::vector<float> arcValues = computeArcValues(sampler);

    if (params.wireframeMode.type == TessParams::CurveType::Cubic) {
        // Add phantom start for Catmull-Rom interpolation
        gp_Pnt p0 = sampler.Value(1);
        wireframeArcValues.push_back(0.0f);

        wireframePoints.push_back(GfVec3f(p0.X(), p0.Y(), p0.Z()));
        if (params.wireframeEmbedSurfaceNormals) wireframeSurfaceNormals.push_back(wireframeSurfaceNormals[0]);

        for (int i = 1; i <= n; ++i) {
            gp_Pnt p = sampler.Value(i);
            wireframePoints.push_back(GfVec3f(p.X(), p.Y(), p.Z()));
            wireframeArcValues.push_back(arcValues[i - 1]);
            if (params.wireframeEmbedSurfaceNormals) wireframeSurfaceNormals.push_back(wireframeSurfaceNormals[i - 1]);
        }

        // Add phantom end
        gp_Pnt pN = sampler.Value(n);
        wireframePoints.push_back(GfVec3f(pN.X(), pN.Y(), pN.Z()));
        wireframeArcValues.push_back(1.0f);
        if (params.wireframeEmbedSurfaceNormals) wireframeSurfaceNormals.push_back(wireframeSurfaceNormals[n - 1]);

        wireframeCounts.push_back(n + 2);
    } else {
        // ResampledLinear
        for (int i = 1; i <= n; ++i) {
            gp_Pnt p = sampler.Value(i);
            wireframePoints.push_back(GfVec3f(p.X(), p.Y(), p.Z()));
            wireframeArcValues.push_back(arcValues[i - 1]);
            if (params.wireframeEmbedSurfaceNormals) wireframeSurfaceNormals.push_back(wireframeSurfaceNormals[i - 1]);
        }
        wireframeCounts.push_back(n);
    }

    wireframeContinuity.push_back(continuity);
}

// Walk every edge once, unify triangulation nodes shared by
// adjacent faces, accumulate boundary tangents,
// and either emit resampled wireframe curves immediately or defer
// "Underlying" sampling until after welding gives us final indices.
void MeshTessellationRoutine::buildEdgeWalk(
    const TopoDS_Shape& defShape,
    const TessParams& params,
    MeshTessellationContext& ctx
) {
    TopExp::MapShapes(defShape, TopAbs_FACE, ctx.faceMap);
    TopExp::MapShapesAndAncestors(defShape, TopAbs_EDGE, TopAbs_FACE, ctx.edgeToFaces);

    for (TopExp_Explorer edgeExp(defShape, TopAbs_EDGE); edgeExp.More(); edgeExp.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(edgeExp.Current());
        if (BRep_Tool::Degenerated(edge)) continue;

        int edgeIdx = ctx.edgeToFaces.FindIndex(edge);
        if (edgeIdx == 0) continue;

        const NCollection_List<TopoDS_Shape>& adjFaces = ctx.edgeToFaces.FindFromIndex(edgeIdx);

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

            int surfaceIndex = ctx.faceMap.FindIndex(face);
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
        if (isSurfaceBoundary && params.wireframeMode.sampling == TessParams::CurveSampling::Underlying)
            edgeCanonicalKeys.reserve(numNodes);

        for (int k = 1; k <= numNodes; k++) {
            int canonicalNode = canonical.poly->Node(k);
            TriNodeKey canonKey = {canonical.tri.get(), canonicalNode};
            TriNodeKey resolvedCanon = resolveAlias(ctx.nodeAlias, canonKey);

            // Output indices are assigned later in the face loop.
            ctx.boundaryKeys.insert(resolvedCanon);

            for (size_t fi = 1; fi < facePolys.size(); fi++) {
                int otherNode = facePolys[fi].poly->Node(k);
                TriNodeKey otherKey = {facePolys[fi].tri.get(), otherNode};
                TriNodeKey resolvedOther = resolveAlias(ctx.nodeAlias, otherKey);
                ctx.boundaryKeys.insert(resolvedOther);

                // Alias the other face's node to the canonical representative
                // so that the face loop emits a single shared vertex for both.
                if (resolvedOther != resolvedCanon) {
                    ctx.nodeAlias[resolvedOther] = resolvedCanon;
                }
            }

            if (isSurfaceBoundary && params.wireframeMode.sampling == TessParams::CurveSampling::Underlying)
                edgeCanonicalKeys.push_back(canonKey);
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
                TriNodeKey canonKey = resolveAlias(ctx.nodeAlias, {canonical.tri.get(), canonicalNode});
                TangentAccum& t = ctx.tangentAccum[canonKey];
                t.sum += tan;
                t.count++;
            }
        }

        if (isSurfaceBoundary && params.wireframeMode.type != TessParams::CurveType::None) {
            if (params.wireframeMode.sampling == TessParams::CurveSampling::Underlying) {
                if (edgeCanonicalKeys.size() >= 2)
                    ctx.deferredCurves.push_back({std::move(edgeCanonicalKeys), continuity});
            } else {
                emitResampledWireframeCurve(edge, adjFaces, continuity, params);
            }
        }
    }
}

// Pre-pass so output buffers can be reserve()'d up front instead of
// growing incrementally during the face loop.
void MeshTessellationRoutine::countTrianglesAndNodes(const TopoDS_Shape& defShape, int& totalTris, int& totalNodes) {
    totalTris = 0;
    totalNodes = 0;
    for (TopExp_Explorer faceExp(defShape, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
        const TopoDS_Face& face = TopoDS::Face(faceExp.Current());
        TopLoc_Location loc;
        auto tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull()) continue;
        totalTris += tri->NbTriangles();
        totalNodes += tri->NbNodes();
    }
}

// Welds one face's triangulation nodes into points (skipping nodes
// already emitted by a face visited earlier, via ctx.nodeToCanonical), and
// reports the world-space bounding box of the nodes it touched.
void MeshTessellationRoutine::weldFaceNodes(
    const occt::handle<Poly_Triangulation>& tri,
    const gp_Trsf& trsf,
    MeshTessellationContext& ctx,
    float bboxOut[6] // xmin, xmax, ymin, ymax, zmin, zmax
) {
    float wxMin= std::numeric_limits<float>::max(), wxMax=-std::numeric_limits<float>::max();
    float wyMin= std::numeric_limits<float>::max(), wyMax=-std::numeric_limits<float>::max();
    float wzMin= std::numeric_limits<float>::max(), wzMax=-std::numeric_limits<float>::max();

    for (int j = 1; j <= tri->NbNodes(); j++) {
        TriNodeKey key = resolveAlias(ctx.nodeAlias, {tri.get(), j});
        if (ctx.nodeToCanonical.count(key)) continue;

        gp_Pnt p = tri->Node(j).Transformed(trsf);
        int idx = static_cast<int>(points.size());
        points.push_back(GfVec3f(
            static_cast<float>(p.X()),
            static_cast<float>(p.Y()),
            static_cast<float>(p.Z())
        ));
        ctx.nodeToCanonical[key] = idx;

        wxMin = std::min(wxMin, (float)p.X()); wxMax = std::max(wxMax, (float)p.X());
        wyMin = std::min(wyMin, (float)p.Y()); wyMax = std::max(wyMax, (float)p.Y());
        wzMin = std::min(wzMin, (float)p.Z()); wzMax = std::max(wzMax, (float)p.Z());

        if (ctx.boundaryKeys.count(key))
            ctx.boundaryNodes.insert(idx);
    }

    bboxOut[0] = wxMin; bboxOut[1] = wxMax;
    bboxOut[2] = wyMin; bboxOut[3] = wyMax;
    bboxOut[4] = wzMin; bboxOut[5] = wzMax;
}

// Emits faceVarying normals/UVs and vertex indices for every triangle of one
// face, and accumulates per-point normal sums for later averaging.
void MeshTessellationRoutine::emitFaceTriangles(
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
) {
    for (int j = 1; j <= tri->NbTriangles(); j++) {
        int n1, n2, n3;
        tri->Triangle(j).Get(n1, n2, n3);
        if (reversed) std::swap(n2, n3);

        int i1 = ctx.nodeToCanonical[resolveAlias(ctx.nodeAlias, {tri.get(), n1})];
        int i2 = ctx.nodeToCanonical[resolveAlias(ctx.nodeAlias, {tri.get(), n2})];
        int i3 = ctx.nodeToCanonical[resolveAlias(ctx.nodeAlias, {tri.get(), n3})];

        faceVertexCounts.push_back(3);
        faceVertexIndices.push_back(i1);
        faceVertexIndices.push_back(i2);
        faceVertexIndices.push_back(i3);

        for (int localIdx : {n1, n2, n3}) {
            GfVec3f normal(0.0f, 0.0f, 1.0f);
            float u = 0.0f, v = 0.0f;

            if (hasUV) {
                gp_Pnt2d uv = tri->UVNode(localIdx);
                u = static_cast<float>(uv.X());
                v = static_cast<float>(uv.Y());
            } else {
                int canonIdx = ctx.nodeToCanonical[resolveAlias(ctx.nodeAlias, {tri.get(), localIdx})];
                const GfVec3f& wp = points[canonIdx];
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

            normals.push_back(normal);
            patch.uvs.push_back(GfVec2f(u, v));
            patch.uMin = std::min(patch.uMin, u);
            patch.uMax = std::max(patch.uMax, u);
            patch.vMin = std::min(patch.vMin, v);
            patch.vMax = std::max(patch.vMax, v);

            int canonIdx = ctx.nodeToCanonical[resolveAlias(ctx.nodeAlias, {tri.get(), localIdx})];
            if (canonIdx >= (int)ctx.pointNormalAccum.size()) {
                ctx.pointNormalAccum.resize(canonIdx + 1, GfVec3f(0.f, 0.f, 0.f));
                ctx.pointNormalCount.resize(canonIdx + 1, 0);
            }
            ctx.pointNormalAccum[canonIdx] += normal;
            ctx.pointNormalCount[canonIdx]++;
        }
    }
}

// Weld nodes, build the local UV patch,
// figure out the fallback tangent frame for faces with no native UVs, then
// hand off to emitFaceTriangles for the actual per-triangle work.
void MeshTessellationRoutine::tessellateFaces(
    const TopoDS_Shape& defShape,
    MeshTessellationContext& ctx
) {
    int surfaceBoundIdx = 0; // 'global' running triangle offset for geom subsets

    for (TopExp_Explorer faceExp(defShape, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
        const TopoDS_Face& face = TopoDS::Face(faceExp.Current());
        int surfaceIndex = ctx.faceMap.FindIndex(face);
        TopLoc_Location loc;
        occt::handle<Poly_Triangulation> tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull()) continue;

        SurfaceIDBounds surfaceBounds = { surfaceBoundIdx, surfaceBoundIdx + tri->NbTriangles(), surfaceIndex };
        surfaceIDBounds.push_back(surfaceBounds);
        surfaceBoundIdx += tri->NbTriangles();

        gp_Trsf trsf = loc.Transformation();

        float bbox[6];
        weldFaceNodes(tri, trsf, ctx, bbox);

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
        float dims[3] = { bbox[1]-bbox[0], bbox[3]-bbox[2], bbox[5]-bbox[4] };
        std::sort(dims, dims+3, std::greater<float>());
        patch.worldW = std::max(dims[0], 1e-10f);
        patch.worldH = std::max(dims[1], 1e-10f);

        emitFaceTriangles(tri, trsf, reversed, hasUV, geomSurface, faceTangentU, faceTangentV, faceCentroid, ctx, patch);

        ctx.uvPatches.push_back(std::move(patch));
    }
}

// Turn the boundary node set / tangent accumulator collected during
// the edge walk into per-output-point arrays.
void MeshTessellationRoutine::finalizeBoundaryData(MeshTessellationContext& ctx) {
    isBoundaryVertex.resize(points.size(), false);
    for (int idx : ctx.boundaryNodes)
        isBoundaryVertex[idx] = true;

    boundaryTangents.assign(points.size(), GfVec3f(0.0f, 0.f, 0.f));
    for (auto& [key, tangent] : ctx.tangentAccum) {
        auto it = ctx.nodeToCanonical.find(resolveAlias(ctx.nodeAlias, key));
        if (it == ctx.nodeToCanonical.end()) continue;

        const int idx = it->second;
        if (idx < 0 || idx >= (int)points.size()) continue;

        const float len = tangent.sum.GetLength();
        if (len > 1e-10f) {
            boundaryTangents[idx] = tangent.sum / len;
        } else {
            boundaryTangents[idx] = GfVec3f(0.0f, 0.0f, 0.0f);
        }
    }
}

// Average the per-point normal sums accumulated in emitFaceTriangles.
// Only needed when wireframe curves sample the underlying triangulation
// (Underlying) and are asked to carry surface normals.
VtArray<GfVec3f> computeEmbeddedPointNormals(const TessParams& params, const MeshTessellationContext& ctx) {
    VtArray<GfVec3f> pointNormals(ctx.pointNormalAccum.size(), GfVec3f(0.0f, 0.0f, 1.0f));
    if (params.wireframeEmbedSurfaceNormals && params.wireframeMode.sampling == TessParams::CurveSampling::Underlying) {
        for (int i = 0; i < (int)ctx.pointNormalAccum.size(); ++i) {
            if (ctx.pointNormalCount[i] > 0) {
                GfVec3f avg = ctx.pointNormalAccum[i]; // sum, not yet divided
                float len = avg.GetLength();
                pointNormals[i] = (len > 1e-6f) ? avg / len : GfVec3f(0.f, 0.f, 1.f);
            }
        }
    }
    return pointNormals;
}

// For wireframe curves that sample the underlying triangulation
// (deferred during the edge walk because welded output indices weren't
// known yet), resolve their node keys to final point indices and emit them.
void MeshTessellationRoutine::emitDeferredWireframeCurves(
    const TessParams& params,
    const MeshTessellationContext& ctx,
    const VtArray<GfVec3f>& pointNormals
) {
    for (const DeferredCurve& deferred : ctx.deferredCurves) {
        std::vector<int> resolved;
        resolved.reserve(deferred.keys.size());
        for (const TriNodeKey& key : deferred.keys) {
            auto it = ctx.nodeToCanonical.find(resolveAlias(ctx.nodeAlias, key));
            if (it != ctx.nodeToCanonical.end())
                resolved.push_back(it->second);
        }
        if (resolved.size() < 2) continue;

        std::vector<float> arcValues(resolved.size(), 0.0f);
        {
            float total = 0.0f;
            for (size_t i = 1; i < resolved.size(); ++i)
                total += (points[resolved[i]] - points[resolved[i-1]]).GetLength();
            float cum = 0.0f;
            for (size_t i = 1; i < resolved.size(); ++i) {
                cum += (points[resolved[i]] - points[resolved[i-1]]).GetLength();
                arcValues[i] = (total > 1e-10f) ? cum / total : 1.0f;
            }
            arcValues.back() = 1.0f;
        }

        if (params.wireframeMode.type == TessParams::CurveType::Cubic) {
            // Phantom start — duplicate first point
            wireframePoints.push_back(points[resolved.front()]);
            wireframeArcValues.push_back(0.0f);
            if (params.wireframeEmbedSurfaceNormals)
                wireframeSurfaceNormals.push_back(pointNormals[resolved.front()]);
        }
        for (size_t i = 0; i < resolved.size(); ++i) {
            wireframePoints.push_back(points[resolved[i]]);
            wireframeArcValues.push_back(arcValues[i]);
            if (params.wireframeEmbedSurfaceNormals)
                wireframeSurfaceNormals.push_back(pointNormals[resolved[i]]);
        }
        if (params.wireframeMode.type == TessParams::CurveType::Cubic) {
            // Phantom end — duplicate last point
            wireframePoints.push_back(points[resolved.back()]);
            wireframeArcValues.push_back(1.0f);
            wireframeCounts.push_back(static_cast<int>(resolved.size()) + 2);
            if (params.wireframeEmbedSurfaceNormals)
                wireframeSurfaceNormals.push_back(pointNormals[resolved.back()]);
        } else {
            wireframeCounts.push_back(static_cast<int>(resolved.size()));
        }
        wireframeContinuity.push_back(deferred.continuity);
    }
}

// points was built incrementally and may contain points that
// no triangle ended up referencing (e.g. from faces that failed to weld
// cleanly). Drop them and reindex faceVertexIndices to match.
void MeshTessellationRoutine::compactUnusedPoints() {
    if (faceVertexIndices.empty()) {
        LOG_DEBUG("  -> tessellatePart: faceVertexIndices is empty, clearing points and boundary flags to avoid confusion downstream");
        points.clear();
        isBoundaryVertex.clear();
        return;
    }

    if (points.empty()) return;

    LOG_DEBUG("  -> tessellatePart: Welding vertices and reindexing faces");
    std::vector<int> oldToNew(points.size(), -1);
    VtArray<GfVec3f> newPoints;
    newPoints.reserve(points.size());
    VtArray<bool> newIsBoundary;
    VtArray<GfVec3f> newBoundaryTangents;
    if (!isBoundaryVertex.empty()) {
        newIsBoundary.reserve(points.size());
    }

    if (!boundaryTangents.empty()) {
        newBoundaryTangents.reserve(points.size());
    }

    for (int index : faceVertexIndices) {
        if (oldToNew[index] == -1) {
            oldToNew[index] = newPoints.size();
            newPoints.push_back(points[index]);
            if (!isBoundaryVertex.empty()) {
                newIsBoundary.push_back(isBoundaryVertex[index]);
            }
            if (!boundaryTangents.empty()) {
                newBoundaryTangents.push_back(boundaryTangents[index]);
            }
        }
    }

    for (int& index : faceVertexIndices) {
        index = oldToNew[index];
    }

    points = std::move(newPoints);
    if (!isBoundaryVertex.empty()) {
        isBoundaryVertex = std::move(newIsBoundary);
    }
    if (!boundaryTangents.empty()) {
        boundaryTangents = std::move(newBoundaryTangents);
    }
}

void MeshTessellationRoutine::applyUnitScale(const TessParams& params) {
    if (params.unitScale == 1.0) return;

    const float s = static_cast<float>(params.unitScale);
    auto scalePoints = [s](VtArray<GfVec3f>& points) {
        for (GfVec3f& p : points) {
            p *= s;
        }
    };
    scalePoints(points);
    scalePoints(wireframePoints);
}

// A definition is valid if it has mesh geometry OR sketch curves.
// Pure edge compounds (e.g. AP242 PMI annotation shapes) have no faces
// but do carry sketch curves, so only reject if both are absent.
bool MeshTessellationRoutine::hasValidGeometry() {
    return !(points.empty() &&
             wireframeCounts.empty());
}

bool MeshTessellationRoutine::tessellate(
    const TopoDS_Shape& defShape, 
    const TessParams& params,
    const SdfPath& protoPath
) {
    auto tessellateStart = Clock::now();

    LOG_DEBUG("  -> tessellatePart: Edge walk preparation");

    MeshTessellationContext ctx;
    buildEdgeWalk(defShape, params, ctx);

    auto edgeWalkEnd = Clock::now();
    LOG_DEBUG("  Edge-walk time: " + std::to_string(Seconds(edgeWalkEnd - tessellateStart).count()) + " s");

    // Get a total of the number of triangles and nodes across all faces to
    // reserve output buffer sizes up front.
    int totalTris = 0, totalNodes = 0;
    countTrianglesAndNodes(defShape, totalTris, totalNodes);

    faceVertexCounts.reserve(totalTris);
    faceVertexIndices.reserve(totalTris * 3);
    normals.reserve(totalTris * 3);
    ctx.uvPatches.reserve(ctx.faceMap.Extent());
    surfaceIDBounds.reserve(ctx.faceMap.Extent());

    // weld positions, emit faceVarying normals.
    tessellateFaces(defShape, ctx);
    perSurfaceUVs = packUVAtlas(ctx.uvPatches);

    finalizeBoundaryData(ctx);

    VtArray<GfVec3f> pointNormals = computeEmbeddedPointNormals(params, ctx);
    emitDeferredWireframeCurves(params, ctx, pointNormals);

    compactUnusedPoints();

    auto faceProcessEnd = Clock::now();
    LOG_DEBUG("  Face processing time: " + std::to_string(Seconds(faceProcessEnd - edgeWalkEnd).count()) + " s");

    auto tessellateEnd = Clock::now();
    LOG_DEBUG("  Total tessellatePart time: " + std::to_string(Seconds(tessellateEnd - tessellateStart).count()) + " s");

    applyUnitScale(params);

    if (!hasValidGeometry()) {
        LOG_DEBUG("def produced no surface or wireframe geometry in Shape");
        return false;
    }

    return true;
}

bool MeshTessellationRoutine::definePrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessParams& params
) const {
    bool hasPoints = !points.empty() && !faceVertexIndices.empty() && !faceVertexCounts.empty();
    bool meshDefined = true;
    if (hasPoints) {
        meshDefined = defineMeshPrim(stage, protoPath, params);
    }
    
    bool hasWireframe = !wireframePoints.empty() && !wireframeCounts.empty();
    bool wireframeDefined = true;
    if (hasWireframe) {
        wireframeDefined = defineWireframePrim(stage, protoPath, params);
    }

    return meshDefined && wireframeDefined;
}

bool MeshTessellationRoutine::writePrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessParams& params
) const {
    bool meshWritten = true;
    if (!points.empty() && !faceVertexIndices.empty() && !faceVertexCounts.empty()) {
        meshWritten = writeMeshPrim(stage, protoPath, params);
    }

    bool wireframeWritten = true;
    if (!wireframePoints.empty() && !wireframeCounts.empty()) {
        wireframeWritten = writeWireframePrim(stage, protoPath, params);
    }

    return meshWritten && wireframeWritten;
}