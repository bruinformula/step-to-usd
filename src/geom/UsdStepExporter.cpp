#include <stddef.h>
#include <chrono>
#include <filesystem>
#include <utility>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <functional>
#include <initializer_list>
#include <ostream>
#include <numeric>
#include <optional>
#include <unordered_map>
#include <unordered_set>

#include <opencascade/TDF_Label.hxx>
#include <opencascade/TopLoc_Location.hxx>
#include <opencascade/gp_Trsf.hxx>
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
#include <opencascade/OSD_Parallel.hxx>
#include <opencascade/BRepExtrema_MapOfIntegerPackedMapOfInteger.hxx>
#include <opencascade/Bnd_Box.hxx>
#include <opencascade/GeomAbs_Shape.hxx>
#include <opencascade/GeomAdaptor_Surface.hxx>
#include <opencascade/NCollection_IndexedDataMap.hxx>
#include <opencascade/NCollection_IndexedMap.hxx>
#include <opencascade/NCollection_List.hxx>
#include <opencascade/Poly_PolygonOnTriangulation.hxx>
#include <opencascade/Poly_Triangle.hxx>
#include <opencascade/Quantity_Color.hxx>
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

#pragma push_macro("Handle")
#undef Handle

#include <pxr/pxr.h>

#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/editContext.h>
#include <pxr/usd/usd/variantSets.h>
#include <pxr/usd/usd/modelAPI.h>
#include <pxr/usd/usd/inherits.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/references.h>

#include <pxr/usd/sdf/changeBlock.h>
#include <pxr/usd/sdf/types.h>

#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>
#include <pxr/usd/usdGeom/basisCurves.h>
#include <pxr/usd/usdGeom/imageable.h>
#include <pxr/usd/usdGeom/subset.h>
#include <pxr/usd/usdGeom/primvar.h>
#include <pxr/usd/usdGeom/xformOp.h>

#include <pxr/base/vt/array.h>
#include <pxr/base/vt/types.h>
#include <pxr/base/tf/error.h>
#include <pxr/base/tf/staticData.h>
#include <pxr/base/tf/errorMark.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/gf/vec4f.h>
#include <pxr/base/gf/vec3f.h>

#pragma pop_macro("Handle")

#include "UsdStepExporter.h"
#include "UsdStepExporterSchemaSupport.h"

class Geom_Surface;

PXR_NAMESPACE_USING_DIRECTIVE

// Utils 
// rotation block: transposed relative to OCC Value(row,col) convention
// translation: from TranslationPart() into the last row
static GfMatrix4d trsfToGfMatrix(const gp_Trsf& t) {
    gp_XYZ trans = t.TranslationPart();
    auto clean = [](double v) { return std::abs(v) < 1e-10 ? 0.0 : v; };
    return GfMatrix4d(
        clean(t.Value(1,1)), clean(t.Value(2,1)), clean(t.Value(3,1)), 0.0,
        clean(t.Value(1,2)), clean(t.Value(2,2)), clean(t.Value(3,2)), 0.0,
        clean(t.Value(1,3)), clean(t.Value(2,3)), clean(t.Value(3,3)), 0.0,
        clean(trans.X()),    clean(trans.Y()),    clean(trans.Z()),    1.0
    );
}

static std::string sanitizeUsdName(const std::string_view& name, int idx) {
    if (name.empty()) return "Node_" + std::to_string(idx);

    std::string result;
    result.reserve(name.size());

    for (char c : name) {
        if (std::isalnum(c) || c == '_') result += c;
        else result += '_'; // replace hyphens, spaces, dots, etc.
    }

    // Usd prim names must start with a letter or underscore
    if (!result.empty() && std::isdigit(result[0]))
        result = "_" + result;

    if (result.empty()) 
        return "Node_";

    return result + std::to_string(idx);
}

// UVs 
struct UVPatch {
    std::vector<GfVec2f> uvs; // one per face-vertex, in raw param space
    float uMin, uMax, vMin, vMax;
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
        // Tile dims proportional to param range
        float area = std::sqrt(uRange * vRange);
        tileWidths[i] = uRange / area;
        tileHeights[i] = vRange / area;
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

// Usd Export
static bool initUsdStage(UsdStageRefPtr stage) {
    

    UsdGeomSetStageUpAxis(stage, UsdGeomTokens->z);

    UsdGeomXform model = UsdGeomXform::Define(stage, SdfPath("/Model"));
    if (!model) {
        std::cerr << "Failed to define root /Model\n";
        return false;
    }

    stage->SetDefaultPrim(model.GetPrim());

    if (!UsdGeomXform::Define(stage, SdfPath("/Model/Assembly"))) {
        std::cerr << "Failed to define /Model/Assembly\n";
        return false;
    }

    if (!UsdGeomXform::Define(stage, SdfPath("/Model/Prototypes"))) {
        std::cerr << "Failed to define /Model/Prototypes\n";
        return false;
    }

    UsdPrim cadPartClass = stage->CreateClassPrim(SdfPath("/Model/CADPart"));
    UsdGeomImageable(cadPartClass).CreateVisibilityAttr().Set(UsdGeomTokens->inherited);

    auto makeClassChild = [&](const char* name) {
        SdfPath childPath = SdfPath("/Model/CADPart").AppendChild(TfToken(name));
        UsdPrim child = stage->DefinePrim(childPath);
        UsdGeomImageable(child).CreateVisibilityAttr().Set(UsdGeomTokens->inherited);
        return child;
    };

    makeClassChild("Mesh");
    makeClassChild("Wireframe");
    makeClassChild("Sketch");

    return true;
}

static bool tesselatePart(
    TessResult& result, 
    const TopoDS_Shape& defShape, 
    const TessParams& params
) {
    
    using Clock = std::chrono::high_resolution_clock;
    using Seconds = std::chrono::duration<double>;

    auto tesselateStart = Clock::now();

    Bnd_Box bbox;
    BRepBndLib::Add(defShape, bbox);
    double xmin, ymin, zmin, xmax, ymax, zmax;
    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);

    double diagonal = std::sqrt(
        std::pow(xmax - xmin, 2) + 
        std::pow(ymax - ymin, 2) + 
        std::pow(zmax - zmin, 2)
    );

    result.renderOnly = params.renderPurposeThreshold != std::numeric_limits<float>::infinity() && diagonal < params.renderPurposeThreshold;

    IMeshTools_Parameters meshParams;
    meshParams.Deflection = static_cast<float>(diagonal * params.meshLinearDeflection);
    meshParams.Angle = params.meshAngularDeflection; // in radians
    meshParams.MinSize = meshParams.Deflection * params.meshMinSize;
    BRepMesh_IncrementalMesh mesher(defShape, meshParams);
    mesher.Perform();

    int maxPasses = params.maxNumberRemeshPasses;
    IMeshTools_Parameters repairParams = meshParams;

    // repeat check for self-intersections 
    // if fail, refine the mesh until there are none 
    // or we hit the max pass count.

    // important note this is on the whole shape 
    // not just the intersected shapes, 
    // so the edge walk later works

    for (int pass = 0; pass < maxPasses; ++pass) {
        BRepExtrema_SelfIntersection checker(defShape, params.selfIntersectionThreshold);
        checker.Perform();

        if (!checker.IsDone()) break;

        const BRepExtrema_MapOfIntegerPackedMapOfInteger& overlaps = checker.OverlapElements();

        if (overlaps.IsEmpty()) break;

        repairParams.Deflection *= 0.5;
        repairParams.Angle *= 0.5;

        BRepTools::Clean(defShape); 
        BRepMesh_IncrementalMesh(defShape, repairParams).Perform();
    }
    

    auto meshEnd = Clock::now();
    //std::cout << "  Mesh time: " << Seconds(meshEnd - tesselateStart).count() << " s\n";
    // normals will be faceVarying: result.normals.size() == result.faceVertexIndices.size()

    // positions remain welded via topology
    // Poly_Triangululation is a tesselated representation of 
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
    TopExp::MapShapes(defShape, TopAbs_FACE, faceMap);

    NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher> edgeToFaces;
    TopExp::MapShapesAndAncestors(defShape, TopAbs_EDGE, TopAbs_FACE, edgeToFaces);

    // Per-edge data deferred for Linear mode. 
    struct DeferredLinearCurve {
        std::vector<TriNodeKey> keys;
        int continuity;
    };
    
    std::vector<DeferredLinearCurve> deferredLinearCurves;

    for (TopExp_Explorer edgeExp(defShape, TopAbs_EDGE); edgeExp.More(); edgeExp.Next()) {
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

        std::vector<TriNodeKey> edgeCanonicalKeys;
        if (isSurfaceBoundary && params.wireframeMode.sampling == CurveSampling::Underlying)
            edgeCanonicalKeys.reserve(numNodes);

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

        if (isSurfaceBoundary && params.wireframeMode.type != CurveType::None) {
            if (params.wireframeMode.sampling == CurveSampling::Underlying) {
                if (edgeCanonicalKeys.size() >= 2)
                    deferredLinearCurves.push_back({std::move(edgeCanonicalKeys), continuity});
            } else {
                BRepAdaptor_Curve adaptor(edge);
                GCPnts_QuasiUniformDeflection sampler(adaptor, params.wireframeDeflection, adaptor.FirstParameter(), adaptor.LastParameter());

                if (sampler.IsDone() && sampler.NbPoints() >= 2) {
                    int n = sampler.NbPoints();

                    if (params.wireframeMode.type == CurveType::CatmullRom) {
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

                        result.curveCounts.push_back(n + 2);
                    } else {
                        // ResampledLinear
                        for (int i = 1; i <= n; ++i) {
                            gp_Pnt p = sampler.Value(i);
                            result.curvePoints.push_back(GfVec3f(p.X(), p.Y(), p.Z()));
                        }
                        result.curveCounts.push_back(n);
                    }

                    result.curveContinuity.push_back(continuity);
                }
            }
        }
    }

    auto edgeWalkEnd = Clock::now();
    //std::cout << "  Edge-walk time: " << Seconds(edgeWalkEnd - meshEnd).count() << " s\n";

    // sketches in Step 242 are registered as free edges 
    // in the defintion shape and are not guaranteed to be connected to any faces, 
    // so we have to do a separate edge walk to find them and sample 
    if (params.sketchMode.type != CurveType::None) {
        for (TopExp_Explorer edgeExp(defShape, TopAbs_EDGE); edgeExp.More(); edgeExp.Next()) {
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

            if (params.sketchMode.type == CurveType::CatmullRom) {
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
    for (TopExp_Explorer faceExp(defShape, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
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
    for (TopExp_Explorer faceExp(defShape, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
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
            if (params.wireframeMode.type == CurveType::CatmullRom) {
                // Phantom start — duplicate first point
                result.curvePoints.push_back(result.points[resolved.front()]);
            }
            for (int idx : resolved)
                result.curvePoints.push_back(result.points[idx]);
            if (params.wireframeMode.type == CurveType::CatmullRom) {
                // Phantom end — duplicate last point
                result.curvePoints.push_back(result.points[resolved.back()]);
                result.curveCounts.push_back(static_cast<int>(resolved.size()) + 2);
            } else {
                result.curveCounts.push_back(static_cast<int>(resolved.size()));
            }
            result.curveContinuity.push_back(deferred.continuity);
        }
    }

    auto faceProcessEnd = Clock::now();
    //std::cout << "  Face processing time: " << Seconds(faceProcessEnd - edgeWalkEnd).count() << " s\n";

    auto tesselateEnd = Clock::now();
    //std::cout << "  Total tesselatePart time: " << Seconds(tesselateEnd - tesselateStart).count() << " s\n";

    // A definition is valid if it has mesh geometry OR sketch curves.
    // Pure edge compounds (e.g. AP242 PMI annotation shapes) have no faces
    // but do carry sketch curves, so only reject if both are absent.
    if (result.points.empty() && result.sketchCounts.empty()) {
        std::cerr << "  Warning: def produced no geometry or sketch curves\n";
        return false;
    }

    return true;
}

bool UsdStepExporter::writePrototypeGeometry(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessResult& r,
    const CurveMode& wireframeMode,
    const CurveMode& sketchMode,
    int defIdx
) {
    

    UsdGeomXform protoXform = UsdGeomXform::Define(stage, protoPath);
    UsdPrim protoPrim = protoXform.GetPrim();

    { // SdfChangeBlock
        SdfChangeBlock changeBlock;
        protoPrim.GetInherits().AddInherit(SdfPath("/Model/CADPart"));

        protoPrim.CreateAttribute(TfToken("stepExport:defIndex"), SdfValueTypeNames->Int, true).Set(defIdx);

        if (r.renderOnly) {
            UsdGeomImageable imageable(protoPrim);
            imageable.CreatePurposeAttr().Set(UsdGeomTokens->render);
        }
    }

    if (!r.points.empty()) {
        UsdGeomMesh proto = UsdGeomMesh::Define(stage, protoPath.AppendChild(TfToken("Mesh")));

        for (const auto& surfaceIDBounds : r.surfaceIDBounds) {
            int count = surfaceIDBounds.endIdx - surfaceIDBounds.startIdx;
            
            VtIntArray indices(count);
            std::iota(indices.begin(), indices.end(), surfaceIDBounds.startIdx);

            UsdGeomSubset::CreateGeomSubset(
                proto,
                TfToken("SurfaceSubset_" + std::to_string(surfaceIDBounds.surfaceID)),
                UsdGeomTokens->face,
                indices,
                TfToken("materialBind"),
                UsdGeomTokens->nonOverlapping 
            );
        }

        { // SdfChangeBlock
            SdfChangeBlock changeBlock;
            proto.GetPointsAttr().Set(r.points);
            proto.GetFaceVertexCountsAttr().Set(r.faceVertexCounts);
            proto.GetFaceVertexIndicesAttr().Set(r.faceVertexIndices);
            proto.GetSubdivisionSchemeAttr().Set(UsdGeomTokens->none);
            proto.SetNormalsInterpolation(UsdGeomTokens->faceVarying);
            proto.GetNormalsAttr().Set(r.normals);

            UsdGeomPrimvarsAPI api(proto);

            UsdGeomPrimvar primUV = api.CreatePrimvar(
                TfToken("st"),
                SdfValueTypeNames->TexCoord2fArray,
                UsdGeomTokens->faceVarying
            );

            UsdGeomPrimvar primSurfaceID = api.CreatePrimvar(
                TfToken("surfaceID"),
                SdfValueTypeNames->IntArray,
                UsdGeomTokens->uniform
            );

            UsdGeomPrimvar primIsBoundaryVertex = api.CreatePrimvar(
                TfToken("isBoundaryVertex"),
                SdfValueTypeNames->BoolArray,
                UsdGeomTokens->vertex
            );

            primUV.Set(r.perSurfaceUVs);
            primSurfaceID.Set(r.surfaceIDs);
            primIsBoundaryVertex.Set(r.isBoundaryVertex);
        } // SdfChangeBlock
    }

    if (!r.curveCounts.empty()) {
        int pointOffset = 0;
        SdfPath wireframePath = protoPath.AppendChild(TfToken("Wireframe"));
        UsdGeomXform curveXform = UsdGeomXform::Define(stage, wireframePath);
        for (int ci = 0; ci < (int)r.curveCounts.size(); ++ci) {
            int count = r.curveCounts[ci];

            VtArray<GfVec3f> pts(
                r.curvePoints.begin() + pointOffset,
                r.curvePoints.begin() + pointOffset + count
            );

            SdfPath curvePath = wireframePath.AppendChild(
                TfToken("Wireframe_" + std::to_string(ci))
            );

            UsdGeomBasisCurves curve = UsdGeomBasisCurves::Define(stage, curvePath);
            {
                SdfChangeBlock changeBlock;
                if (wireframeMode.type == CurveType::CatmullRom) {
                    curve.CreateTypeAttr().Set(UsdGeomTokens->cubic);
                    curve.CreateBasisAttr().Set(UsdGeomTokens->catmullRom);
                } else {
                    curve.CreateTypeAttr().Set(UsdGeomTokens->linear);
                }
                curve.CreateWrapAttr().Set(UsdGeomTokens->nonperiodic);
                curve.GetPointsAttr().Set(pts);
                curve.GetCurveVertexCountsAttr().Set(VtIntArray{count});

                VtArray<float> widths(count, 0.1f);
                curve.CreateWidthsAttr().Set(widths);

                VtArray<GfVec3f> color = {{0.8f, 0.8f, 0.8f}};
                curve.GetDisplayColorAttr().Set(color);

                if (ci < (int)r.curveContinuity.size()) {
                    UsdGeomPrimvarsAPI curveAPI(curve);
                    curveAPI.CreatePrimvar(
                        TfToken("continuityType"),
                        SdfValueTypeNames->IntArray,
                        UsdGeomTokens->uniform
                    ).Set(VtIntArray{r.curveContinuity[ci]});
                }
            }
            pointOffset += count;
        }
    }

    if (!r.sketchCounts.empty()) {
        int pointOffset = 0;
        SdfPath sketchPath = protoPath.AppendChild(TfToken("Sketch"));
        UsdGeomXform sketchXform = UsdGeomXform::Define(stage, sketchPath);
        for (int ci = 0; ci < (int)r.sketchCounts.size(); ++ci) {
            int count = r.sketchCounts[ci];

            VtArray<GfVec3f> pts(
                r.sketchPoints.begin() + pointOffset,
                r.sketchPoints.begin() + pointOffset + count
            );

            SdfPath curvePath = sketchPath.AppendChild(
                TfToken("Sketch_" + std::to_string(ci))
            );
            UsdGeomBasisCurves sketchCurve = UsdGeomBasisCurves::Define(stage, curvePath);
            {
                SdfChangeBlock changeBlock;
                if (sketchMode.type == CurveType::CatmullRom) {
                    sketchCurve.CreateTypeAttr().Set(UsdGeomTokens->cubic);
                    sketchCurve.CreateBasisAttr().Set(UsdGeomTokens->catmullRom);
                } else {
                    sketchCurve.CreateTypeAttr().Set(UsdGeomTokens->linear);
                }
                sketchCurve.CreateWrapAttr().Set(UsdGeomTokens->nonperiodic);
                sketchCurve.GetPointsAttr().Set(pts);
                sketchCurve.GetCurveVertexCountsAttr().Set(VtIntArray{count});

                VtArray<float> sketchWidths(count, 0.1f);
                sketchCurve.CreateWidthsAttr().Set(sketchWidths);

                VtArray<GfVec3f> sketchColor = {{0.4f, 0.7f, 1.0f}};
                sketchCurve.GetDisplayColorAttr().Set(sketchColor);
            }
            pointOffset += count;
        }
    }

    return true;
}

void UsdStepExporter::writeInstanceXforms(
    const std::vector<StepModel::PartInstance>& instances,
    UsdStageRefPtr stage, 
    const std::vector<SdfPath>& paths, 
    const LabelMap<SdfPath>& prototypePaths
) {
    
    // pre compute which instances have children
    std::vector<bool> hasChildren(instances.size(), false);
    for (size_t i = 0; i < instances.size(); i++) {
        if (instances[i].parentIdx != -1)
            hasChildren[instances[i].parentIdx] = true;
    }
    // Define all xform nodes, wire references, and author transforms
    const int total = (int)instances.size();
    int completed = 0;
    for (size_t i = 0; i < instances.size(); i++) {
        const StepModel::PartInstance& inst = instances[i];
        UsdGeomXform xform = UsdGeomXform::Define(stage, paths[i]);
        if (!xform) {
            std::cerr << "[" << i << "] Failed to define Xform at " << paths[i] << "\n";
            std::cerr << "\r[" << ++completed << "/" << total << "] Writing instances..." << std::flush;
            continue;
        }
        {
            SdfChangeBlock changeBlock;
            // Usd composes the full world transform later
            xform.AddTransformOp().Set(trsfToGfMatrix(inst.localTransform));
            if (inst.type == StepModel::InstanceType::Leaf) {
                auto protoIter = prototypePaths.find(inst.definitionLabel);
                if (protoIter == prototypePaths.end()) {
                    std::cerr << "\r[" << ++completed << "/" << total << "] Writing instances..." << std::flush;
                    continue;
                }
                xform.GetPrim().GetReferences().AddInternalReference(protoIter->second);
                UsdModelAPI(xform.GetPrim()).SetKind(TfToken("component"));
                if (!hasChildren[i]) {
                    xform.GetPrim().SetInstanceable(true);
                }
                if (inst.color.has_value()) {
                    VtArray<GfVec3f> displayColor = {{
                        static_cast<float>(inst.color->Red()),
                        static_cast<float>(inst.color->Green()),
                        static_cast<float>(inst.color->Blue())
                    }};
                    UsdAttribute colorAttr = xform.GetPrim().CreateAttribute(
                        TfToken("primvars:displayColor"),
                        SdfValueTypeNames->Color3fArray,
                        false
                    );
                    colorAttr.Set(displayColor);
                }
                if (!inst.visible) {
                    UsdGeomImageable(xform.GetPrim())
                        .CreateVisibilityAttr()
                        .Set(UsdGeomTokens->invisible);
                }
            } else {
                UsdModelAPI(xform.GetPrim()).SetKind(TfToken("assembly"));
            }
        } // SdfChangeBlock
        std::cerr << "\r[" << ++completed << "/" << total << "] Writing instances..." << std::flush;
    }
    std::cerr << "\n";
}

std::vector<SdfPath> UsdStepExporter::computeInstancePaths(const std::vector<StepModel::PartInstance>& instances) {
    std::unordered_map<std::string, int> nameCounts;
    std::vector<SdfPath> paths(instances.size());

    // pre-order guarantees parent path is always assigned before we 
    // reach any of its children or Usd will omplain about missing 
    // parent prims when we try to define them
    for (size_t i = 0; i < instances.size(); i++) {
        const StepModel::PartInstance& inst = instances[i];

        SdfPath parentPath;
        if (instances[i].parentIdx == -1) {
            parentPath = SdfPath("/Model/Assembly");
        } else {
            parentPath = paths[instances[i].parentIdx];
        }

        int count = nameCounts[inst.name]++;
        std::string finalName = sanitizeUsdName(inst.name, count);
        paths[i] = parentPath.AppendChild(TfToken(finalName));
    }

    return paths;
}

void UsdStepExporter::populateUsd(
    const StepModel& model, 
    UsdStageRefPtr stage, 
    const TessParams& params
) {
    
    using Clock = std::chrono::high_resolution_clock;
    using Seconds = std::chrono::duration<double>;
    auto totalStart = Clock::now();
    TfErrorMark mark;

    std::vector<std::pair<TDF_Label, TopoDS_Shape>> defs(
        model.definitionShapes.begin(),
        model.definitionShapes.end()
    );
    std::vector<TessResult> tessResults(defs.size());

    auto tessStart = Clock::now();
    std::atomic<int> tessCompleted(0);
    const int total = (int)defs.size();

    OSD_Parallel::For(0, total, [&](int i) {
        const TopoDS_Shape& defShape = defs[i].second;
        if (defShape.IsNull()) {
            int done = ++tessCompleted;
            std::cerr << "\r[" << done << "/" << total << "] Tessellating..." << std::flush;
            return;
        }

        auto partStart = Clock::now();
        std::string partName = getLabelName(defs[i].first);

        try {
            TessResult result;
            if (tesselatePart(result, defShape, params))
                tessResults[i] = std::move(result);
        } catch (const Standard_Failure& e) {
            std::cerr << "\nOCC exception during tessellation of def " << i << ": " << e.GetMessageString() << "\n";
        }

        double elapsed = Seconds(Clock::now() - partStart).count();
        if (elapsed > 10.0) {
            std::cerr << "\n[SLOW] def " << i << " (" << partName << ")"
                    << "  time=" << elapsed << "s"
                    << "  faces=" << [&]{ int n=0; for(TopExp_Explorer e(defShape,TopAbs_FACE);e.More();e.Next()) n++; return n; }()
                    << "\n";
        }

        int done = ++tessCompleted;
        std::cerr << "\r[" << done << "/" << total << "] Tessellating..." << std::flush;
    });
    std::cerr << "\n";

    if (!initUsdStage(stage))
        return;

    stage->SetMetadata(TfToken("metersPerUnit"), model.metersPerUnit);

    LabelMap<SdfPath> prototypePaths;
    std::unordered_map<std::string, int> protoNameCounts;
    const int protoTotal = (int)defs.size();
    int protoCompleted = 0;

    for (int i = 0; i < protoTotal; i++) {
        const TessResult& r = tessResults[i];
        if (r.points.empty() && r.sketchCounts.empty()) {
            std::cerr << "\r[" << ++protoCompleted << "/" << protoTotal << "] Writing prototypes..." << std::flush;
            continue;
        }

        std::string rawName = getLabelName(defs[i].first);
        if (rawName.empty()) {
            rawName = "Def_" + std::to_string(i);
        }
        int protoCount = protoNameCounts[rawName]++;
        std::string name = sanitizeUsdName(rawName, protoCount);
        SdfPath protoPath = SdfPath("/Model/Prototypes").AppendChild(TfToken(name));

        if (!writePrototypeGeometry(stage, protoPath, r, params.wireframeMode, params.sketchMode, i)) {
            std::cerr << "\r[" << ++protoCompleted << "/" << protoTotal << "] Writing prototypes..." << std::flush;
            continue;
        }

        prototypePaths[defs[i].first] = protoPath;
        //std::cout << "  Prototype " << protoPath << " -> " << r.points.size() << " verts\n";
        std::cerr << "\r[" << ++protoCompleted << "/" << protoTotal << "] Writing prototypes..." << std::flush;
    }
    std::cerr << "\n";

    // Hide /Model/Prototypes from renderers
    // Usd will complain and not define prims under an inactive parent
    UsdPrim prototypeRoot = stage->GetPrimAtPath(SdfPath("/Model/Prototypes"));
    if (prototypeRoot.IsValid()) {
        prototypeRoot.SetActive(false);
    }

    std::vector<SdfPath> paths = computeInstancePaths(model.instances);
    writeInstanceXforms(model.instances, stage, paths, prototypePaths);

    if (!mark.IsClean()) {
        for (const auto& error : mark)
            std::cerr << "Usd: " << error.GetCommentary() << "\n";
    }
}

void UsdStepExporter::populateVariantUsd(
    const StepModel& model,
    UsdStageRefPtr stage,
    const std::vector<VariantParams>& variantParams
) {
    
    using Clock = std::chrono::high_resolution_clock;
    using Seconds = std::chrono::duration<double>;

    auto totalStart = Clock::now();
    TfErrorMark mark;

    std::vector<std::pair<TDF_Label, TopoDS_Shape>> defs(
        model.definitionShapes.begin(),
        model.definitionShapes.end()
    );

    int numTessellations = (int)variantParams.size() * defs.size();

    std::vector<TessResult> tessVariantResults(numTessellations);

    LabelMap<int> labelToDefIdx;
    for (int i = 0; i < (int)defs.size(); i++) {
        labelToDefIdx[defs[i].first] = i;
    }

    auto tessStart = Clock::now();
    std::atomic<int> tessCompleted(0);

    OSD_Parallel::For(0, numTessellations, [&](int i) {
        int baseIdx = i / (int)variantParams.size();
        int variantIdx = i % (int)variantParams.size();
        const VariantParams& params = variantParams[variantIdx];

        const TopoDS_Shape& defShape = defs[baseIdx].second;
        if (defShape.IsNull()) {
            int done = ++tessCompleted;
            std::cerr << "\r[" << done << "/" << numTessellations
                      << "] Tessellating..." << std::flush;
            return;
        }

        try {
            TessResult result;
            if (tesselatePart(result, defShape, params.tessParams))
                tessVariantResults[variantIdx * (int)defs.size() + baseIdx] = std::move(result);
        } catch (const Standard_Failure& e) {
            std::cerr << "OCC exception during tessellation of def " << baseIdx << " variant " << variantIdx << ": " << e.GetMessageString() << "\n";
        }
        int done = ++tessCompleted;
        std::cerr << "\r[" << done << "/" << numTessellations << "] Tessellating..." << std::flush;
    });
    std::cerr << "\n";

    std::cout << "Tessellation time:  " << Seconds(Clock::now() - tessStart).count() << " s\n";

    if (!initUsdStage(stage))
        return;

    stage->SetMetadata(TfToken("metersPerUnit"), model.metersPerUnit);

    UsdGeomXform prototypes = UsdGeomXform(stage->GetPrimAtPath(SdfPath("/Model/Prototypes")));

    LabelMap<SdfPath> prototypePaths;
    std::unordered_map<std::string, int> protoNameCounts;

    UsdVariantSets variantSets = prototypes.GetPrim().GetVariantSets();

    for (const auto& p : variantParams) {
        if (!variantSets.HasVariantSet(p.variantSetName)) {
            variantSets.AddVariantSet(p.variantSetName);
        }
    }

    for (int v = 0; v < (int)variantParams.size(); v++) {
        const VariantParams& params = variantParams[v];
        const TessParams& tessParams = params.tessParams;
        int baseIdx = v * defs.size();
        protoNameCounts.clear();

        std::cerr << "Writing variant [" << (v + 1) << "/"
                  << variantParams.size() << "]: "
                  << params.variantName << "\n";

        bool overwrite = true;

        if (fs::exists(params.outpath)) {
            if (overwrite) {
                std::cerr << "Warning: overwriting existing file at " << params.outpath << "\n";
                fs::remove(params.outpath);
            } else {
                std::cerr << "Error: file already exists at " << params.outpath << ", skipping variant " << v << "\n";
                continue;
            }
        }

        UsdStageRefPtr lodStage = UsdStage::CreateNew(params.outpath);

        UsdPrim lodPrototypesPrim = lodStage->DefinePrim(SdfPath("/Model/Prototypes"));

        int protoCompleted = 0;
        const int protoTotal = (int)defs.size();

        for (int i = 0; i < defs.size(); i++) {
            const TessResult& r = tessVariantResults[baseIdx + i];
            if (r.points.empty() && r.sketchCounts.empty()) {
                std::cerr << "\r  [" << ++protoCompleted << "/" << protoTotal
                          << "] Writing prototypes..." << std::flush;
                continue;
            }

            std::string rawName = getLabelName(defs[i].first);
            if (rawName.empty()) {
                rawName = "Def_" + std::to_string(i);
            }

            int protoCount = protoNameCounts[rawName]++;
            std::string name = sanitizeUsdName(rawName, protoCount);
            SdfPath protoPath = SdfPath("/Model/Prototypes").AppendChild(TfToken(name));

            if (!writePrototypeGeometry(lodStage, protoPath, r, tessParams.wireframeMode, tessParams.sketchMode, i)) {
                std::cerr << "\r  [" << ++protoCompleted << "/" << protoTotal
                          << "] Writing prototypes..." << std::flush;
                continue;
            }

            prototypePaths[defs[i].first] = protoPath;

            //std::cout << "  Prototype " << protoPath << " -> " << r.points.size() << " verts\n";
            std::cerr << "\r  [" << ++protoCompleted << "/" << protoTotal << "] Writing prototypes..." << std::flush;
        }
        std::cerr << "\n";

        lodStage->Save();

        UsdVariantSet variantSet = variantSets.GetVariantSet(params.variantSetName);

        variantSet.AddVariant(params.variantName);
        variantSet.SetVariantSelection(params.variantName);
        {
            UsdEditContext ctx(variantSet.GetVariantEditContext());
            prototypes.GetPrim().GetReferences().AddReference(params.refpath.generic_string());
        }
    }

    std::cout << "Prototypes written: " << prototypePaths.size() << "\n";

    // Hide /Model/Prototypes from renderers
    // Usd will complain and not define prims under an inactive parent
    UsdPrim prototypeRoot = stage->GetPrimAtPath(SdfPath("/Model/Prototypes"));
    if (prototypeRoot.IsValid()) {
        prototypeRoot.SetActive(false);
        /*
        UsdGeomImageable imageable(prototypeRoot);
        if (imageable) {
            imageable.MakeInvisible();
        }
        */
    }

    std::vector<SdfPath> paths = computeInstancePaths(model.instances);
    writeInstanceXforms(model.instances, stage, paths, prototypePaths);

    if (!mark.IsClean()) {
        for (const auto& error : mark)
            std::cerr << "Usd: " << error.GetCommentary() << "\n";
    }

    std::cout << "Total export time:  " << Seconds(Clock::now() - totalStart).count() << " s\n";
}