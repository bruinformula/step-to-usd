
#include <algorithm>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include <BRepAdaptor_Surface.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <IMeshTools_Parameters.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_IndexedMap.hxx>
#include <NCollection_List.hxx>
#include <Poly_PolygonOnTriangulation.hxx>
#include <Poly_Triangulation.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/pxr.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/vt/array.h>
#include <pxr/usd/sdf/path.h>

#pragma pop_macro("Handle")

#include "CadUSD/Logger.h"
#include "CadUSD/Tessellation/TessellationRoutine.h"
#include "CadUSD/Tessellation/TessellationUtils.h"

#include "Quadriflow/hierarchy.hpp"
#include "Quadriflow/optimizer.hpp"
#include "Quadriflow/parametrizer.hpp"

PXR_NAMESPACE_USING_DIRECTIVE

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

struct MeshWeldContext {
    NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> faceMap;
    NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher> edgeToFaces;

    std::unordered_map<TriNodeKey, int, TriNodeKey::Hash> nodeToCanonical;
    std::unordered_set<TriNodeKey, TriNodeKey::Hash> boundaryKeys;
    std::unordered_set<int> boundaryNodes;
    std::unordered_map<TriNodeKey, TriNodeKey, TriNodeKey::Hash> nodeAlias;
};

// Follows alias chains built during the edge walk so every node on a shared
// edge resolves to the same canonical (triangulation*, node) key.
static TriNodeKey resolveAlias(
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

// Walk every edge once, unify triangulation nodes shared by adjacent faces.
static void buildEdgeWalk(
    const TopoDS_Shape& defShape,
    MeshWeldContext& ctx
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

            occt::handle<Poly_PolygonOnTriangulation> poly = BRep_Tool::PolygonOnTriangulation(edge, tri, loc);
            if (poly.IsNull()) continue;

            int surfaceIndex = ctx.faceMap.FindIndex(face);
            facePolys.push_back({face, tri, poly, loc, surfaceIndex});
        }

        if (facePolys.empty()) continue;

        const FacePoly& canonical = facePolys[0];
        int numNodes = canonical.poly->NbNodes();

        for (size_t fi = 1; fi < facePolys.size(); fi++) {
            numNodes = std::min(numNodes, facePolys[fi].poly->NbNodes());
        }

        for (int k = 1; k <= numNodes; k++) {
            int canonicalNode = canonical.poly->Node(k);
            TriNodeKey canonKey = {canonical.tri.get(), canonicalNode};
            TriNodeKey resolvedCanon = resolveAlias(ctx.nodeAlias, canonKey);

            ctx.boundaryKeys.insert(resolvedCanon);

            for (size_t fi = 1; fi < facePolys.size(); fi++) {
                int otherNode = facePolys[fi].poly->Node(k);
                TriNodeKey otherKey = {facePolys[fi].tri.get(), otherNode};
                TriNodeKey resolvedOther = resolveAlias(ctx.nodeAlias, otherKey);
                ctx.boundaryKeys.insert(resolvedOther);

                if (resolvedOther != resolvedCanon) {
                    ctx.nodeAlias[resolvedOther] = resolvedCanon;
                }
            }
        }
    }
}

// Welds one face's triangulation nodes into outPoints (skipping nodes
// already emitted by a face visited earlier, via ctx.nodeToCanonical), and
// fills faceNodeToGlobal[localNode] -> welded global index for that face.
static void weldFaceNodes(
    const occt::handle<Poly_Triangulation>& tri,
    const gp_Trsf& trsf,
    MeshWeldContext& ctx,
    std::vector<Eigen::Vector3d>& outPoints,
    std::vector<int>& faceNodeToGlobal // sized NbNodes()+1 by the caller
) {
    for (int j = 1; j <= tri->NbNodes(); j++) {
        TriNodeKey key = resolveAlias(ctx.nodeAlias, {tri.get(), j});
        if (ctx.nodeToCanonical.count(key)) {
            faceNodeToGlobal[j] = ctx.nodeToCanonical[key];
            continue;
        }

        gp_Pnt p = tri->Node(j).Transformed(trsf);
        int idx = static_cast<int>(outPoints.size());
        outPoints.emplace_back(p.X(), p.Y(), p.Z());
        ctx.nodeToCanonical[key] = idx;
        faceNodeToGlobal[j] = idx;

        if (ctx.boundaryKeys.count(key))
            ctx.boundaryNodes.insert(idx);
    }
}

bool InstantMeshesTessellationRoutine::tessellate(
    const TopoDS_Shape& defShape,
    const TessParams& params,
    const SdfPath& protoPath
) {
    std::string protoName = protoPath.GetAsString();

    // Run the part through BRepMesh so every face has a triangulation to
    // pull nodes/triangles from.
    double diagonal = computeBoundingBoxDiagonal(defShape);

    IMeshTools_Parameters meshParams;
    meshParams.InParallel = false;
    meshParams.Deflection = diagonal * params.meshLinearDeflection;
    meshParams.Angle = params.meshAngularDeflection;
    meshParams.MinSize = meshParams.Deflection * params.meshMinSize;

    BRepMesh_IncrementalMesh meshGen;
    meshGen.SetShape(defShape);
    meshGen.ChangeParameters() = meshParams;
    meshGen.Perform();

    MeshWeldContext ctx;
    buildEdgeWalk(defShape, ctx);

    std::vector<Eigen::Vector3d> rawPoints;
    std::vector<Eigen::Vector3d> rawNormals;
    std::vector<Eigen::Vector3d> rawTangents;
    std::vector<double> rawTangentWeights;
    std::vector<Eigen::Vector3i> rawFaces;

    for (TopExp_Explorer faceExp(defShape, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
        const TopoDS_Face& face = TopoDS::Face(faceExp.Current());

        TopLoc_Location loc;
        occt::handle<Poly_Triangulation> tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull() || tri->NbTriangles() == 0 || tri->NbNodes() == 0) continue;

        const gp_Trsf trsf = loc.Transformation();
        const bool reversed = (face.Orientation() == TopAbs_REVERSED);
        const bool hasUV = tri->HasUVNodes();

        BRepAdaptor_Surface adapter(face);

        const int firstNewIdx = (int)rawPoints.size();

        std::vector<int> faceNodeToGlobal(tri->NbNodes() + 1, -1);
        weldFaceNodes(tri, trsf, ctx, rawPoints, faceNodeToGlobal);

        // rawPoints grew by exactly the newly-welded vertices this face
        // introduced (indices [firstNewIdx, rawPoints.size())); sample
        // dp/du for those only -- reused/aliased vertices already have a
        // tangent from whichever face welded them first.
        for (int j = 1; j <= tri->NbNodes(); j++) {
            int global = faceNodeToGlobal[j];
            if (global < firstNewIdx) continue; // already had a tangent

            gp_Vec normalVec(0.0, 0.0, 1.0);
            gp_Vec tangent(0.0, 0.0, 0.0);
            bool tangentValid = false;

            if (hasUV) {
                gp_Pnt2d uv = tri->UVNode(j);
                gp_Pnt surfP;
                gp_Vec dU, dV;
                adapter.D1(uv.X(), uv.Y(), surfP, dU, dV);

                gp_Vec faceNormal = dU.Crossed(dV);
                double normalMag = faceNormal.Magnitude();
                if (normalMag > 1e-10) {
                    normalVec = faceNormal / normalMag;
                    if (reversed) normalVec.Reverse();

                    // Re-orthogonalize dp/du against the normal so it's a
                    // clean tangent direction to guide the orientation field.
                    gp_Vec candidate = dU - normalVec * normalVec.Dot(dU);
                    double tangentMag = candidate.Magnitude();
                    if (tangentMag > 1e-10) {
                        tangent = candidate / tangentMag;
                        tangentValid = true;
                    }
                }
            }

            if ((size_t)global >= rawNormals.size()) {
                rawNormals.resize(global + 1, Eigen::Vector3d(0, 0, 1));
                rawTangents.resize(global + 1, Eigen::Vector3d(0, 0, 0));
                rawTangentWeights.resize(global + 1, 0.0);
            }
            rawNormals[global] = Eigen::Vector3d(normalVec.X(), normalVec.Y(), normalVec.Z());
            rawTangents[global] = Eigen::Vector3d(tangent.X(), tangent.Y(), tangent.Z());
            rawTangentWeights[global] = tangentValid ? 1.0 : 0.0;
        }

        for (int ti = 1; ti <= tri->NbTriangles(); ++ti) {
            int n1, n2, n3;
            tri->Triangle(ti).Get(n1, n2, n3);
            if (reversed) std::swap(n2, n3);
            rawFaces.emplace_back(faceNodeToGlobal[n1], faceNodeToGlobal[n2], faceNodeToGlobal[n3]);
        }
    }

    if (rawPoints.empty() || rawFaces.empty()) {
        LOG_DEBUG("InstantMeshesTessellationRoutine (quadriflow): no triangulation for " + protoName);
        return false;
    }

    // Hand it to Quadriflow.
    qflow::Parametrizer field;
    field.scale = 0.001;
    field.flag_preserve_sharp = 1;
    //field.flag_preserve_boundary = 1;
    field.flag_adaptive_scale = 1;
    //field.flag_minimum_cost_flow = 1;
    //field.flag_aggresive_sat = 1;
    //field.hierarchy.rng_seed = 1234;
    field.V.resize(3, (Eigen::Index)rawPoints.size());
    for (size_t i = 0; i < rawPoints.size(); ++i) {
        field.V.col((Eigen::Index)i) = rawPoints[i];
    }
    field.F.resize(3, (Eigen::Index)rawFaces.size());
    for (size_t i = 0; i < rawFaces.size(); ++i) {
        field.F.col((Eigen::Index)i) = rawFaces[i];
    }

    try {
        field.NormalizeMesh();
        field.Initialize(-1);

        qflow::Hierarchy& mRes = field.hierarchy;

        // Tangent buffer
        mRes.clearConstraints();
        const size_t constrainable = std::min(rawTangents.size(), (size_t)mRes.mV[0].cols());
        for (size_t i = 0; i < constrainable; ++i) {
            if (rawTangentWeights[i] <= 0.0) continue;
            mRes.mCQ[0].col((Eigen::Index)i) = rawTangents[i];
            mRes.mCQw[0][(Eigen::Index)i] = rawTangentWeights[i];
        }
        mRes.propagateConstraints();

        qflow::Optimizer::optimize_orientations(mRes);
        field.ComputeOrientationSingularities();

        qflow::Optimizer::optimize_scale(mRes, field.rho, field.flag_adaptive_scale);
        field.flag_adaptive_scale = 1;

        qflow::Optimizer::optimize_positions(mRes, field.flag_adaptive_scale);
        field.ComputePositionSingularities();

        field.ComputeIndexMap();
    } catch (const Standard_Failure& e) {
        LOG_ERR("Quadriflow: OCC exception on " + protoName + ": " + std::string(e.GetMessageString()));
        return false;
    } catch (const std::exception& e) {
        LOG_ERR("Quadriflow: std exception on " + protoName + ": " + std::string(e.what()));
        return false;
    }

    if (field.O_compact.empty() || field.F_compact.empty()) {
        LOG_DEBUG("Quadriflow produced no quad mesh for " + protoName);
        return false;
    }

    LOG_DEBUG(
        "InstantMeshesTessellationRoutine (quadriflow): " + protoName +
        " welded " + std::to_string(rawPoints.size()) + " unique verts / " +
        std::to_string(rawFaces.size()) + " triangles -> " +
        std::to_string(field.O_compact.size()) + " quad verts / " +
        std::to_string(field.F_compact.size()) + " quads"
    );

    // Welded triangle mesh goes into the debug "sample" points slot (same
    // slots the old point-cloud sampler used to fill).
    samplePoints.reserve(rawPoints.size());
    sampleNormals.reserve(rawNormals.size());
    for (size_t i = 0; i < rawPoints.size(); ++i) {
        samplePoints.push_back(GfVec3f(
            static_cast<float>(rawPoints[i].x()),
            static_cast<float>(rawPoints[i].y()),
            static_cast<float>(rawPoints[i].z())
        ));
        const Eigen::Vector3d& n = (i < rawNormals.size()) ? rawNormals[i] : Eigen::Vector3d(0, 0, 1);
        sampleNormals.push_back(GfVec3f(
            static_cast<float>(n.x()),
            static_cast<float>(n.y()),
            static_cast<float>(n.z())
        ));
    }

    // Undo Quadriflow's internal normalization and apply the exporter's
    // unit scale on the way out.
    points.reserve(field.O_compact.size());
    normals.reserve(field.N_compact.size());
    for (size_t i = 0; i < field.O_compact.size(); ++i) {
        Eigen::Vector3d p =
            (field.O_compact[i] * field.normalize_scale + field.normalize_offset) * params.unitScale;
        points.push_back(GfVec3f(
            static_cast<float>(p.x()),
            static_cast<float>(p.y()),
            static_cast<float>(p.z())
        ));
        const Eigen::Vector3d& n = field.N_compact[i];
        normals.push_back(GfVec3f(
            static_cast<float>(n.x()),
            static_cast<float>(n.y()),
            static_cast<float>(n.z())
        ));
    }

    faceVertexCounts.reserve(field.F_compact.size());
    faceVertexIndices.reserve(field.F_compact.size() * 4);
    for (const Eigen::Vector4i& quad : field.F_compact) {
        faceVertexCounts.push_back(4);
        faceVertexIndices.push_back(quad[0]);
        faceVertexIndices.push_back(quad[1]);
        faceVertexIndices.push_back(quad[2]);
        faceVertexIndices.push_back(quad[3]);
    }

    return true;
}