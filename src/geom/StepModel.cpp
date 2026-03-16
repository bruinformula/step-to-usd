#include <cassert>
#include <chrono>
#include <filesystem>
#include <unordered_map>
#include <utility>

#include <opencascade/BRepBuilderAPI_Transform.hxx>
#include <opencascade/BinXCAFDrivers.hxx>
#include <opencascade/STEPCAFControl_Reader.hxx>
#include <opencascade/TDocStd_Application.hxx>
#include <opencascade/TDocStd_Document.hxx>
#include <opencascade/XCAFDoc_DocumentTool.hxx>
#include <opencascade/XCAFDoc_ShapeTool.hxx>
#include <opencascade/TDF_Label.hxx>
#include <opencascade/TDF_LabelMap.hxx>
#include <opencascade/TopLoc_Location.hxx>
#include <opencascade/gp_Trsf.hxx>
#include <opencascade/BRepMesh_IncrementalMesh.hxx>
#include <opencascade/BRep_Tool.hxx>
#include <opencascade/StlAPI_Writer.hxx>
#include <opencascade/TopExp_Explorer.hxx>
#include <opencascade/TopoDS.hxx>
#include <opencascade/Poly_Triangulation.hxx>
#include <opencascade/XCAFDoc_ColorTool.hxx>
#include <opencascade/XCAFDoc_MaterialTool.hxx>
#include <opencascade/BRepAdaptor_Surface.hxx>
#include <opencascade/GeomLProp_SLProps.hxx>
#include <opencascade/BRepBndLib.hxx>
#include <opencascade/TopExp.hxx>
#include <opencascade/TopTools_IndexedMapOfShape.hxx>
#include <opencascade/IMeshTools_Parameters.hxx>
#include <opencascade/BRepAdaptor_Curve.hxx>
#include <opencascade/GCPnts_QuasiUniformDeflection.hxx>
#include <opencascade/XSControl_WorkSession.hxx>
#include <opencascade/XSControl_TransferReader.hxx>
#include <opencascade/StepRepr_PropertyDefinition.hxx>
#include <opencascade/TransferBRep_ShapeBinder.hxx>
#include <opencascade/Transfer_TransientProcess.hxx>
#include <opencascade/IFSelect_WorkSession.hxx>
#include <opencascade/StepShape_ShapeDefinitionRepresentation.hxx>
#include <opencascade/StepRepr_Representation.hxx>
#include <opencascade/Interface_Static.hxx>
#include <opencascade/Interface_Graph.hxx>
#include <opencascade/Interface_EntityIterator.hxx>
#include <opencascade/BRepExtrema_SelfIntersection.hxx>
#include <opencascade/XCAFDoc_LayerTool.hxx>
#include <opencascade/BRepTools.hxx>
#include <opencascade/TDF_Tool.hxx>
#include <opencascade/OSD_Parallel.hxx>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/editContext.h>
#include <pxr/usd/usd/variantSets.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>
#include <pxr/usd/usdGeom/basisCurves.h>
#include <pxr/usd/usdGeom/imageable.h>
#include <pxr/usd/usdGeom/subset.h>

#include <pxr/base/vt/array.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/tf/errorMark.h>
#include <pxr/base/tf/token.h>
#include <pxr/usd/usd/modelAPI.h>
#include <pxr/usd/usd/inherits.h>

#include "tessParamsAPI.h"

#pragma pop_macro("Handle")

#include "XCAFUtils.h"
#include "StepModel.h"

// Init
std::optional<StepModel> StepModel::loadFromFile(const fs::path& stepPath) {
    try {
        OSD_Parallel::SetUseOcctThreads(true);

        occt::handle<TDocStd_Application> app = new TDocStd_Application();
        BinXCAFDrivers::DefineFormat(app);

        fs::path xbfPath = stepPath;
        xbfPath.replace_extension("xbf");

        occt::handle<TDocStd_Document> doc;
        app->NewDocument("BinXCAF", doc);

        // xbf is a binary XCAF cache — subsequent loads skip the Step parser
        // entirely. Invalidated whenever the Step file is newer than the cache.
        if (!fs::exists(xbfPath) || fs::last_write_time(xbfPath) < fs::last_write_time(stepPath)) {
            std::cout << "XBF doesn't exist or is out of date. Building from Step...\n";

            STEPCAFControl_Reader reader;
            if (reader.ReadFile(stepPath.c_str()) != IFSelect_RetDone) {
                std::cerr << "Error reading Step file\n";
                return std::nullopt;
            }
            if (!reader.Transfer(doc)) {
                std::cerr << "Error transferring Step data\n";
                return std::nullopt;
            }

            std::cout << "Saving XBF to " << xbfPath << "\n";
            if (app->SaveAs(doc, xbfPath.c_str()) != PCDM_SS_OK)
                std::cerr << "Warning: failed to save XBF cache\n";
        } else {
            std::cout << "Loading cached XBF from " << xbfPath << "\n";
            if (app->Open(xbfPath.c_str(), doc) != PCDM_RS_OK) {
                std::cerr << "Error opening XBF\n";
                return std::nullopt;
            }
        }

        auto shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
        auto colorTool = XCAFDoc_DocumentTool::ColorTool(doc->Main());
        auto materialTool = XCAFDoc_DocumentTool::MaterialTool(doc->Main());
        auto layerTool = XCAFDoc_DocumentTool::LayerTool(doc->Main());

        return StepModel(app, doc, shapeTool, colorTool, materialTool, layerTool);

    } catch (const Standard_Failure& e) {
        std::cerr << "OCC exception: " << e.GetMessageString() << "\n";
        return std::nullopt;
    } catch (const std::exception& e) {
        std::cerr << "std exception: " << e.what() << "\n";
        return std::nullopt;
    }
}

void StepModel::buildInstanceTree() {
    instances.clear();
    definitionShapes.clear();

    NCollection_Sequence<TDF_Label> freeShapes;
    shapeTool->GetFreeShapes(freeShapes);

    // Pass 1: count the total node count so we can resize instances[] once
    // and use direct index writes in pass 2
    int numNodes = 0;
    for (int i = 1; i <= freeShapes.Length(); i++) {
        numNodes += countNodes(freeShapes.Value(i));
    }

    instances.resize(numNodes);

    // Pass 2: pre-order fill — each assembly claims its slot then immediately
    // records firstChildIdx = cursor before recursing, so children are written
    // contiguously after it.
    int cursor = 0;
    gp_Trsf identity;
    for (int i = 1; i <= freeShapes.Length(); i++) {
        fillNode(freeShapes.Value(i), identity, -1, 0, cursor);
    }

    assert(cursor == numNodes); // should hit all the nodes we counted in pass 1

    int leaves = 0, assemblies = 0;
    for (const auto& n : instances) {
        if (n.type == InstanceType::Leaf)     leaves++;
        if (n.type == InstanceType::Assembly) assemblies++;
    }
    std::cout << "Instance tree built: " << instances.size() << " nodes\n";
    std::cout << "  Assemblies:          " << assemblies << "\n";
    std::cout << "  Leaves:              " << leaves     << "\n";
    std::cout << "  Unique definitions:  " << definitionShapes.size() << "\n";
}

bool StepModel::isLabelVisible(const TDF_Label& label) const {
    if (!colorTool->IsVisible(label))
        return false;

    // Some CAD tools might export visibility as 
    // a layer property rather than on the shape itself
    TDF_LabelSequence layers;
    layerTool->GetLayers(label, layers);
    for (int li = 1; li <= layers.Length(); ++li) {
        if (!layerTool->IsVisible(layers.Value(li)))
            return false;
    }

    return true;
}

// Counting
int StepModel::countNodes(const TDF_Label& label) {
    if (!shapeTool->IsShape(label)) return 0;

    if (shapeTool->IsComponent(label)) {
        TDF_Label defLabel;
        if (!shapeTool->GetReferredShape(label, defLabel)) return 0;
        if (shapeTool->IsSimpleShape(defLabel))  return 1;
        if (shapeTool->IsAssembly(defLabel))     return 1 + countAssemblyChildren(defLabel);
        return 1;
    }

    if (shapeTool->IsAssembly(label))    return 1 + countAssemblyChildren(label);
    if (shapeTool->IsSimpleShape(label)) return 1;
    return 0;
}

int StepModel::countAssemblyChildren(const TDF_Label& assemblyDef) {
    NCollection_Sequence<TDF_Label> components;
    shapeTool->GetComponents(assemblyDef, components);
    int total = 0;
    for (int i = 1; i <= components.Length(); i++)
        total += countNodes(components.Value(i));
    return total;
}

// Filling
void StepModel::fillNode(
    const TDF_Label& label,
    const gp_Trsf& parentWorld,
    int parentIdx,
    int depth,
    int& cursor
) {
    if (!shapeTool->IsShape(label)) return;

    // Extract just this label's own location
    // world is computed here purely so children can extract their own localTrsf
    // correctly relative to their parent; it is never stored.
    TopLoc_Location loc = shapeTool->GetLocation(label);
    gp_Trsf localTrsf = loc.Transformation();
    gp_Trsf world = parentWorld * localTrsf;

    if (shapeTool->IsComponent(label)) {
        TDF_Label defLabel;
        if (!shapeTool->GetReferredShape(label, defLabel)) return;

        if (shapeTool->IsSimpleShape(defLabel)) {
            fillLeaf(defLabel, localTrsf, parentIdx, depth, cursor);
        } else if (shapeTool->IsAssembly(defLabel)) {
            fillAssembly(defLabel, localTrsf, world, parentIdx, depth, cursor);
        } else {
            fillLeaf(defLabel, localTrsf, parentIdx, depth, cursor);
        }

    } else if (shapeTool->IsAssembly(label)) {
        fillAssembly(label, localTrsf, world, parentIdx, depth, cursor);

    } else if (shapeTool->IsSimpleShape(label)) {
        fillLeaf(label, localTrsf, parentIdx, depth, cursor);
    }
}

void StepModel::fillLeaf(
    const TDF_Label& defLabel,
    const gp_Trsf& localTrsf,
    int parentIdx,
    int depth,
    int& cursor
) {
    int myIdx = cursor++;

    instances[myIdx].name = getLabelName(defLabel);

    Quantity_Color color(0.8, 0.8, 0.8, Quantity_TOC_RGB);
    
    occt::handle<TCollection_HAsciiString> aName;
    occt::handle<TCollection_HAsciiString> aDescription;
    double                                 aDensity;
    occt::handle<TCollection_HAsciiString> aDensName;
    occt::handle<TCollection_HAsciiString> aDensValType;

    if (materialTool->GetMaterial(defLabel, aName, aDescription, aDensity, aDensName, aDensValType)) {
        aName->Print(std::cout);
        aDescription->Print(std::cout);
        aDensName->Print(std::cout);
        aDensValType->Print(std::cout);
    }

    //std::cout << instances[myIdx].materialName << std::endl;

    bool hasColor = colorTool->GetColor(defLabel, XCAFDoc_ColorSurf, color) || colorTool->GetColor(defLabel, XCAFDoc_ColorGen, color);
    if (hasColor) {
        instances[myIdx].color = color;
    } else {
        instances[myIdx].color = std::nullopt;
    }

    instances[myIdx].type             = InstanceType::Leaf;
    instances[myIdx].definitionLabel  = defLabel;
    instances[myIdx].localTransform   = localTrsf;
    instances[myIdx].parentIdx        = parentIdx;
    instances[myIdx].firstChildIdx    = -1;
    instances[myIdx].childCount       = 0;
    instances[myIdx].depth            = depth;
    instances[myIdx].visible          = isLabelVisible(defLabel);

    // Only store the shape the first time we see this definition label.
    // Subsequent instances of the same part share this entry.
    if (definitionShapes.find(defLabel) == definitionShapes.end()) {
        definitionShapes[defLabel] = shapeTool->GetShape(defLabel);
    }
}

void StepModel::fillAssembly(
    const TDF_Label& defLabel,
    const gp_Trsf& localTrsf,
    const gp_Trsf& world,
    int parentIdx,
    int depth,
    int& cursor
) {
    int myIdx = cursor++;

    instances[myIdx].name = getLabelName(defLabel);
    instances[myIdx].color = std::nullopt;
    instances[myIdx].visible = isLabelVisible(defLabel);

    NCollection_Sequence<TDF_Label> components;
    shapeTool->GetComponents(defLabel, components);

    int validChildren = 0;
    for (int i = 1; i <= components.Length(); i++) {
        if (countNodes(components.Value(i)) > 0) 
            validChildren++;
    }
    // Record firstChildIdx before recursing — cursor is at the first child slot
    // right now, and recursion will fill slots contiguously from here
    int firstChild = (validChildren > 0) ? cursor : -1;

    instances[myIdx].type             = InstanceType::Assembly;
    instances[myIdx].definitionLabel  = defLabel;
    instances[myIdx].localTransform   = localTrsf;
    instances[myIdx].parentIdx        = parentIdx;
    instances[myIdx].firstChildIdx    = firstChild;
    instances[myIdx].childCount       = validChildren;
    instances[myIdx].depth            = depth;

    for (int i = 1; i <= components.Length(); i++)
        fillNode(components.Value(i), world, myIdx, depth + 1, cursor);
}

// Debug 
void StepModel::debugPrintInstances() const {
    for (size_t i = 0; i < instances.size(); i++) {
        const PartInstance& inst = instances[i];
        std::string type = (inst.type == InstanceType::Assembly) ? "ASM" : "LEAF";
        gp_XYZ lt = inst.localTransform.TranslationPart();
        std::cout << "[" << i << "] " << type
                  << " parent="     << inst.parentIdx
                  << " firstChild=" << inst.firstChildIdx
                  << " childCount=" << inst.childCount
                  << " depth="      << inst.depth
                  << " localT=("    << lt.X() << "," << lt.Y() << "," << lt.Z() << ")"
                  << "\n";
    }
}

// UVs 
struct UVPatch {
    std::vector<pxr::GfVec2f> uvs; // one per face-vertex, in raw param space
    float uMin, uMax, vMin, vMax;
};

static pxr::VtArray<pxr::GfVec2f> packUVAtlas(std::vector<UVPatch>& patches) {
    using namespace pxr;
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

// Usd
bool StepModel::tesselatePart(TessResult& result, const TopoDS_Shape& defShape, const TessParams& params) const {
    using namespace pxr;
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

    if (params.lodCullingMinimumSize && diagonal < *params.lodCullingMinimumSize) {
        return false;
    }

    IMeshTools_Parameters meshParams;
    meshParams.Deflection = static_cast<float>(diagonal * params.meshLinearDeflection);
    meshParams.Angle = params.meshAngularDeflection; // in radians
    meshParams.MinSize = meshParams.Deflection * params.meshMinSize;
    BRepMesh_IncrementalMesh mesher(defShape, meshParams);
    mesher.Perform();

    int maxPasses = 0;
    IMeshTools_Parameters repairParams = meshParams;

    // repeat check for self-intersections 
    // if fail, refine the mesh until there are none 
    // or we hit the max pass count.
    
    for (int pass = 0; pass < maxPasses; ++pass) {
        BRepExtrema_SelfIntersection checker(defShape, 1e-6);
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
        if (isSurfaceBoundary && params.wireframeMode == CurveMode::Linear)
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

            if (isSurfaceBoundary && params.wireframeMode == CurveMode::Linear)
                edgeCanonicalKeys.push_back(canonKey);
        }

        if (isSurfaceBoundary && params.wireframeMode != CurveMode::None) {
            if (params.wireframeMode == CurveMode::Linear) {
                if (edgeCanonicalKeys.size() >= 2)
                    deferredLinearCurves.push_back({std::move(edgeCanonicalKeys), continuity});
            } else {
                // ResampledLinear and CatmullRom both sample from the curve
                BRepAdaptor_Curve adaptor(edge);
                GCPnts_QuasiUniformDeflection sampler(adaptor, params.wireframeDeflection, adaptor.FirstParameter(), adaptor.LastParameter());

                if (sampler.IsDone() && sampler.NbPoints() >= 2) {
                    int n = sampler.NbPoints();

                    if (params.wireframeMode == CurveMode::CatmullRom) {
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
    if (params.sketchMode != CurveMode::None) {
        for (TopExp_Explorer edgeExp(defShape, TopAbs_EDGE); edgeExp.More(); edgeExp.Next()) {
            const TopoDS_Edge& edge = TopoDS::Edge(edgeExp.Current());
            if (BRep_Tool::Degenerated(edge)) continue;

            int edgeIdx = edgeToFaces.FindIndex(edge);
            bool isFreeEdge = (edgeIdx == 0) || (edgeToFaces.FindFromIndex(edgeIdx).Extent() == 0);
            if (!isFreeEdge) continue;

            BRepAdaptor_Curve adaptor(edge);

            double deflection;
            
            if (params.sketchMode == CurveMode::Linear) {
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

            if (params.sketchMode == CurveMode::CatmullRom) {
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
            for (int idx : resolved)
                result.curvePoints.push_back(result.points[idx]);
            result.curveCounts.push_back(static_cast<int>(resolved.size()));
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

static bool initUsdStage(pxr::UsdStageRefPtr stage) {
    using namespace pxr;

    UsdGeomSetStageUpAxis(stage, UsdGeomTokens->z);
    stage->SetMetadata(TfToken("metersPerUnit"), 0.001);

    UsdGeomXform assembly = UsdGeomXform::Define(stage, SdfPath("/Assembly"));
    if (!assembly) {
        std::cerr << "Failed to define root /Assembly\n";
        return false;
    }
    stage->SetDefaultPrim(assembly.GetPrim());

    if (!UsdGeomXform::Define(stage, SdfPath("/Prototypes"))) {
        std::cerr << "Failed to define /Prototypes\n";
        return false;
    }

    UsdPrim cadPartClass = stage->CreateClassPrim(SdfPath("/CADPart"));
    UsdGeomImageable(cadPartClass).CreateVisibilityAttr().Set(UsdGeomTokens->inherited);

    auto makeClassChild = [&](const char* name) {
        SdfPath childPath = SdfPath("/CADPart").AppendChild(TfToken(name));
        UsdPrim child = stage->DefinePrim(childPath);
        UsdGeomImageable(child).CreateVisibilityAttr().Set(UsdGeomTokens->inherited);
        return child;
    };

    makeClassChild("mesh");
    makeClassChild("wire");
    makeClassChild("sketch");

    return true;
}

static bool writePrototypeGeometry(
    pxr::UsdStageRefPtr stage,
    const pxr::SdfPath& protoPath,
    const StepModel::TessResult& r,
    CurveMode wireframeMode,
    CurveMode sketchMode
) {
    using namespace pxr;

    UsdGeomXform protoXform = UsdGeomXform::Define(stage, protoPath);
    protoXform.GetPrim().GetInherits().AddInherit(SdfPath("/CADPart"));

    UsdGeomMesh proto = UsdGeomMesh::Define(stage, protoPath.AppendChild(TfToken("mesh")));
    UsdGeomBasisCurves curves = UsdGeomBasisCurves::Define(stage, protoPath.AppendChild(TfToken("wire")));
    UsdGeomBasisCurves sketchCurves = UsdGeomBasisCurves::Define(stage, protoPath.AppendChild(TfToken("sketch")));

    if (!proto) {
        std::cerr << "Failed to define prototype at " << protoPath << "\n";
        return false;
    }

    if (!r.points.empty()) {
        proto.GetPointsAttr().Set(r.points);
        proto.GetFaceVertexCountsAttr().Set(r.faceVertexCounts);
        proto.GetFaceVertexIndicesAttr().Set(r.faceVertexIndices);
        proto.GetSubdivisionSchemeAttr().Set(UsdGeomTokens->none);
        proto.SetNormalsInterpolation(UsdGeomTokens->faceVarying);
        proto.GetNormalsAttr().Set(r.normals);

        for (const auto& surfaceIDBounds : r.surfaceIDBounds) {
            int count = surfaceIDBounds.endIdx - surfaceIDBounds.startIdx;
            
            VtIntArray indices(count);
            std::iota(indices.begin(), indices.end(), surfaceIDBounds.startIdx);

            UsdGeomSubset::CreateGeomSubset(
                proto,
                TfToken("surfaceSubset_" + std::to_string(surfaceIDBounds.surfaceID)),
                UsdGeomTokens->face,
                indices,
                TfToken("materialBind"),
                UsdGeomTokens->nonOverlapping 
            );
        }

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
    }

    if (!r.curveCounts.empty()) {
        if (wireframeMode == CurveMode::CatmullRom) {
            curves.CreateTypeAttr().Set(UsdGeomTokens->cubic);
            curves.CreateBasisAttr().Set(UsdGeomTokens->catmullRom);
        } else {
            // Linear and ResampledLinear both produce polylines
            curves.CreateTypeAttr().Set(UsdGeomTokens->linear);
        }
        curves.CreateWrapAttr().Set(UsdGeomTokens->nonperiodic);
        curves.GetPointsAttr().Set(r.curvePoints);
        curves.GetCurveVertexCountsAttr().Set(r.curveCounts);

        VtArray<float> widths(r.curvePoints.size(), 0.1f); // constant widths option
        curves.CreateWidthsAttr().Set(widths);

        VtArray<GfVec3f> color = {{0.8, 0.8, 0.8}};
        curves.GetDisplayColorAttr().Set(color);

        UsdGeomPrimvarsAPI curveAPI(curves);
        UsdGeomPrimvar primContinuityType = curveAPI.CreatePrimvar(
            TfToken("continuityType"),
            SdfValueTypeNames->IntArray,
            UsdGeomTokens->uniform
        );
        primContinuityType.Set(r.curveContinuity);
    }

    if (!r.sketchCounts.empty()) {
        if (sketchMode == CurveMode::CatmullRom) {
            sketchCurves.CreateTypeAttr().Set(UsdGeomTokens->cubic);
            sketchCurves.CreateBasisAttr().Set(UsdGeomTokens->catmullRom);
        } else {
            // Linear and ResampledLinear both produce polylines
            sketchCurves.CreateTypeAttr().Set(UsdGeomTokens->linear);
        }
        sketchCurves.CreateWrapAttr().Set(UsdGeomTokens->nonperiodic);
        sketchCurves.GetPointsAttr().Set(r.sketchPoints);
        sketchCurves.GetCurveVertexCountsAttr().Set(r.sketchCounts);

        VtArray<float> sketchWidths(r.sketchPoints.size(), 0.1f);
        sketchCurves.CreateWidthsAttr().Set(sketchWidths);

        VtArray<GfVec3f> sketchColor = {{0.4f, 0.7f, 1.0f}}; // blue tint to distinguish from wireframe
        sketchCurves.GetDisplayColorAttr().Set(sketchColor);
    }

    return true;
}

void StepModel::populateUsd(
    pxr::UsdStageRefPtr stage, 
    const TessParams& params
) const {
    using namespace pxr;
    using Clock = std::chrono::high_resolution_clock;
    using Seconds = std::chrono::duration<double>;
    auto totalStart = Clock::now();
    TfErrorMark mark;

    std::vector<std::pair<TDF_Label, TopoDS_Shape>> defs(
        definitionShapes.begin(),
        definitionShapes.end()
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
        try {
            TessResult result;
            if (tesselatePart(result, defShape, params))
                tessResults[i] = std::move(result);
        } catch (const Standard_Failure& e) {
            std::cerr << "\nOCC exception during tessellation of def " << i << ": " << e.GetMessageString() << "\n";
        }
        int done = ++tessCompleted;
        std::cerr << "\r[" << done << "/" << total << "] Tessellating..." << std::flush;
    });
    std::cerr << "\n";

    if (!initUsdStage(stage))
        return;

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
        SdfPath protoPath = SdfPath("/Prototypes").AppendChild(TfToken(name));

        if (!writePrototypeGeometry(stage, protoPath, r, params.wireframeMode, params.sketchMode)) {
            std::cerr << "\r[" << ++protoCompleted << "/" << protoTotal << "] Writing prototypes..." << std::flush;
            continue;
        }

        prototypePaths[defs[i].first] = protoPath;
        //std::cout << "  Prototype " << protoPath << " -> " << r.points.size() << " verts\n";
        std::cerr << "\r[" << ++protoCompleted << "/" << protoTotal << "] Writing prototypes..." << std::flush;
    }
    std::cerr << "\n";

    // Hide /Prototypes from renderers
    // Usd will complain and not define prims under an inactive parent
    UsdPrim prototypeRoot = stage->GetPrimAtPath(SdfPath("/Prototypes"));
    if (prototypeRoot.IsValid())
        prototypeRoot.SetActive(false);

    UsdPrim wireframeRoot = stage->GetPrimAtPath(SdfPath("/Wireframe"));
    if (wireframeRoot.IsValid())
        wireframeRoot.SetActive(false);

    std::vector<SdfPath> paths = computeInstancePaths();
    writeInstanceXforms(stage, paths, prototypePaths);

    if (!mark.IsClean()) {
        for (const auto& error : mark)
            std::cerr << "Usd: " << error.GetCommentary() << "\n";
    }
}

void StepModel::populateVariantUsd(
    pxr::UsdStageRefPtr stage, 
    const std::vector<VariantParams>& variantParams
) const {
    using namespace pxr;
    using Clock = std::chrono::high_resolution_clock;
    using Seconds = std::chrono::duration<double>;

    auto totalStart = Clock::now();
    TfErrorMark mark;

    std::vector<std::pair<TDF_Label, TopoDS_Shape>> defs(
        definitionShapes.begin(),
        definitionShapes.end()
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

    UsdGeomXform prototypes = UsdGeomXform(stage->GetPrimAtPath(SdfPath("/Prototypes")));

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

        UsdPrim lodPrototypesPrim = lodStage->DefinePrim(SdfPath("/Prototypes"));
        lodStage->SetDefaultPrim(lodPrototypesPrim);

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
            SdfPath protoPath = SdfPath("/Prototypes").AppendChild(TfToken(name));

            if (!writePrototypeGeometry(lodStage, protoPath, r, tessParams.wireframeMode, tessParams.sketchMode)) {
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

    // Hide /Prototypes from renderers
    // Usd will complain and not define prims under an inactive parent
    UsdPrim prototypeRoot = stage->GetPrimAtPath(SdfPath("/Prototypes"));
    if (prototypeRoot.IsValid())
        prototypeRoot.SetActive(false);

    UsdPrim wireframeRoot = stage->GetPrimAtPath(SdfPath("/Wireframe"));
    if (wireframeRoot.IsValid())
        wireframeRoot.SetActive(false);

    std::vector<SdfPath> paths = computeInstancePaths();
    writeInstanceXforms(stage, paths, prototypePaths);

    if (!mark.IsClean()) {
        for (const auto& error : mark)
            std::cerr << "Usd: " << error.GetCommentary() << "\n";
    }

    std::cout << "Total export time:  " << Seconds(Clock::now() - totalStart).count() << " s\n";
}

std::vector<pxr::SdfPath> StepModel::computeInstancePaths() const {
    using namespace pxr;

    std::unordered_map<std::string, int> nameCounts;
    std::vector<SdfPath> paths(instances.size());

    // pre-order guarantees parent path is always assigned before we 
    // reach any of its children or Usd will omplain about missing 
    // parent prims when we try to define them
    for (size_t i = 0; i < instances.size(); i++) {
        const PartInstance& inst = instances[i];

        SdfPath parentPath;
        if (instances[i].parentIdx == -1) {
            parentPath = SdfPath("/Assembly");
        } else {
            parentPath = paths[instances[i].parentIdx];
        }

        int count = nameCounts[inst.name]++;
        std::string finalName = sanitizeUsdName(inst.name, count);
        paths[i] = parentPath.AppendChild(TfToken(finalName));
    }

    return paths;
}

void StepModel::writeInstanceXforms(
    pxr::UsdStageRefPtr stage,
    const std::vector<pxr::SdfPath>& paths,
    const LabelMap<pxr::SdfPath>& prototypePaths
) const {
    using namespace pxr;

    // pre compute which instances have children
    std::vector<bool> hasChildren(instances.size(), false);
    for (size_t i = 0; i < instances.size(); i++) {
        if (instances[i].parentIdx != -1)
            hasChildren[instances[i].parentIdx] = true;
    }

    // Define all xform nodes, wire references, and author transforms
    for (size_t i = 0; i < instances.size(); i++) {
        const PartInstance& inst = instances[i];

        UsdGeomXform xform = UsdGeomXform::Define(stage, paths[i]);
        if (!xform) {
            std::cerr << "[" << i << "] Failed to define Xform at " << paths[i] << "\n";
            continue;
        }

        // Usd composes the full world transform later
        xform.AddTransformOp().Set(trsfToGfMatrix(inst.localTransform));

        if (inst.type == InstanceType::Leaf) {
            auto protoIter = prototypePaths.find(inst.definitionLabel);
            if (protoIter == prototypePaths.end()) continue;

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
    }
}