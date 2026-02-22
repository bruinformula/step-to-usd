#include <cassert>
#include <chrono>

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

#include <OSD_Parallel.hxx>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/base/vt/array.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/tf/errorMark.h>
#include <pxr/base/tf/token.h>

#pragma pop_macro("Handle")

#include "XCAFUtils.h"
#include "STEPModel.h"

// Init
std::optional<STEPModel> STEPModel::loadFromFile(const fs::path& stepPath) {
    try {
        OSD_Parallel::SetUseOcctThreads(true);

        occt::handle<TDocStd_Application> app = new TDocStd_Application();
        BinXCAFDrivers::DefineFormat(app);

        fs::path xbfPath = stepPath;
        xbfPath.replace_extension("xbf");

        occt::handle<TDocStd_Document> doc;

        // xbf is a binary XCAF cache — subsequent loads skip the STEP parser
        // entirely. Invalidated whenever the STEP file is newer than the cache.
        if (!fs::exists(xbfPath) || fs::last_write_time(xbfPath) < fs::last_write_time(stepPath)) {
            std::cout << "XBF doesn't exist or is out of date. Building from STEP...\n";
            app->NewDocument("BinXCAF", doc);

            STEPCAFControl_Reader reader;
            if (reader.ReadFile(stepPath.c_str()) != IFSelect_RetDone) {
                std::cerr << "Error reading STEP file\n";
                return std::nullopt;
            }
            if (!reader.Transfer(doc)) {
                std::cerr << "Error transferring STEP data\n";
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
        return STEPModel(app, doc, shapeTool, colorTool, materialTool);

    } catch (const Standard_Failure& e) {
        std::cerr << "OCC exception: " << e.GetMessageString() << "\n";
        return std::nullopt;
    } catch (const std::exception& e) {
        std::cerr << "std exception: " << e.what() << "\n";
        return std::nullopt;
    }
}

void STEPModel::buildInstanceTree() {
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

// Counting
int STEPModel::countNodes(const TDF_Label& label) {
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

int STEPModel::countAssemblyChildren(const TDF_Label& assemblyDef) {
    NCollection_Sequence<TDF_Label> components;
    shapeTool->GetComponents(assemblyDef, components);
    int total = 0;
    for (int i = 1; i <= components.Length(); i++)
        total += countNodes(components.Value(i));
    return total;
}

// Filling
void STEPModel::fillNode(
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

void STEPModel::fillLeaf(
    const TDF_Label& defLabel,
    const gp_Trsf& localTrsf,
    int parentIdx,
    int depth,
    int& cursor
) {
    int myIdx = cursor++;

    instances[myIdx].name = getLabelName(defLabel);

    Quantity_Color color(0.8, 0.8, 0.8, Quantity_TOC_RGB);
    
    Handle(TCollection_HAsciiString) aName;
    Handle(TCollection_HAsciiString) aDescription;
    Standard_Real                    aDensity;
    Handle(TCollection_HAsciiString) aDensName;
    Handle(TCollection_HAsciiString) aDensValType;

    if (materialTool->GetMaterial(defLabel, aName, aDescription, aDensity, aDensName, aDensValType)) {
        aName->Print(std::cout);
        aDescription->Print(std::cout);
        aDensName->Print(std::cout);
        aDensValType->Print(std::cout);
    }

    //std::cout << instances[myIdx].materialName <<std::endl;

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

    // Only store the shape the first time we see this definition label.
    // Subsequent instances of the same part share this entry.
    if (definitionShapes.find(defLabel) == definitionShapes.end()) {
        definitionShapes[defLabel] = shapeTool->GetShape(defLabel);
    }
}

void STEPModel::fillAssembly(
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
void STEPModel::debugPrintInstances() const {
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

// USD

STEPModel::TessResult STEPModel::tesselatePart(const TopoDS_Shape& defShape) const {
    using namespace pxr;
    // Copy before tessellating — BRepMesh writes into the TShape node
    // which is shared across all handles pointing at the same definition
    BRepBuilderAPI_Transform copier(defShape, gp_Trsf(), true);
    TopoDS_Shape localShape = copier.Shape();
    BRepMesh_IncrementalMesh(localShape, 0.1).Perform();

    TessResult result;
    int vertexOffset = 0;

    for (TopExp_Explorer exp(localShape, TopAbs_FACE); exp.More(); exp.Next()) {
        const TopoDS_Face& face = TopoDS::Face(exp.Current());
        TopLoc_Location loc;
        occt::handle<Poly_Triangulation> tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull()) continue;

        gp_Trsf trsf = loc.Transformation();

        BRepAdaptor_Surface adapter = BRepAdaptor_Surface(face);

        float uMin = adapter.FirstUParameter();
        float uMax = adapter.LastUParameter();
        float vMin = adapter.FirstVParameter();
        float vMax = adapter.LastVParameter();

        bool hasUV = tri->HasUVNodes();
        bool hasNormals = tri->HasNormals();

        //std::cout << "Has Normals: " << hasNormals << "\nHas UV: " << hasUV << "\n";

        for (int j = 1; j <= tri->NbNodes(); j++) {
            gp_Pnt p = tri->Node(j).Transformed(trsf);
            result.points.push_back(GfVec3f(
                static_cast<float>(p.X()),
                static_cast<float>(p.Y()),
                static_cast<float>(p.Z())
            ));
        }

        for (int j = 1; j <= tri->NbTriangles(); j++) {
            int n1, n2, n3;
            tri->Triangle(j).Get(n1, n2, n3);
            if (face.Orientation() == TopAbs_REVERSED) 
                std::swap(n2, n3);

            result.faceVertexCounts.push_back(3);
            result.faceVertexIndices.push_back(vertexOffset + n1 - 1);
            result.faceVertexIndices.push_back(vertexOffset + n2 - 1);
            result.faceVertexIndices.push_back(vertexOffset + n3 - 1);

            // Normals
            for (int idx : {n1, n2, n3}) {
                gp_Pnt2d uv = tri->UVNode(idx);

                float u;
                float v;

                if (hasUV) {
                    u = static_cast<float>(uv.X());
                    v = static_cast<float>(uv.Y());

                    u = fmax(uMin, fmin(uMax, u));
                    v = fmax(vMin, fmin(vMax, v));
                } else {
                    u = 0.0f;
                    v = 0.0f;
                }

                GeomAdaptor_Surface adapterSurface = adapter.Surface();
                occt::handle<Geom_Surface> geomSurface = adapterSurface.Surface(); // the .Surface().Surface() what even is this 

                GeomLProp_SLProps props = GeomLProp_SLProps(geomSurface, u, v, 1, 1e-6);

                //std::cout << props.D1U().X() << " " << props.D1U().Y() << " " << props.D1U().Z() << "\n";
                //std::cout << props.D1V().X() << " " << props.D1V().Y() << " " << props.D1V().Z() << "\n";

                GfVec3f normal;
                if (props.IsNormalDefined()) {
                    gp_Vec n = props.Normal();

                    GfVec3f rawNormal(
                        static_cast<float>(n.X()),
                        static_cast<float>(n.Y()),
                        static_cast<float>(n.Z())
                    );

                    float length = rawNormal.GetLength();

                    if (length > 1e-10) {
                        normal = rawNormal / length;
                    } else {
                        normal = GfVec3f(0.0f, 0.0f, 1.0f);
                    }

                } else {
                    normal = GfVec3f(0.0f, 0.0f, 1.0f);
                }

                if (face.Orientation() == TopAbs_REVERSED) {
                    normal = -normal;
                }

                result.normals.push_back(normal);

            }

        }

        vertexOffset += tri->NbNodes();
    }

    result.valid = !result.points.empty();
    if (!result.valid) {
        std::cerr << "  Warning: def produced no geometry\n";
    } else { 
        //std::cout << "  Def " << i << ": " << result.points.size() << " verts, " << result.faceVertexCounts.size() << " faces\n";
    }
    return result;
}



void STEPModel::writeUSD(const fs::path& outputPath) const {
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

    LabelMap<int> labelToDefIdx;
    for (int i = 0; i < (int)defs.size(); i++) {
        labelToDefIdx[defs[i].first] = i;
    }

    auto tessStart = Clock::now();

    OSD_Parallel::For(0, (int)defs.size(), [&](int i) {
        const TopoDS_Shape& defShape = defs[i].second;
        if (defShape.IsNull()) return;

        tessResults[i] = tesselatePart(defShape);
    });

    std::cout << "Tessellation time:  " << Seconds(Clock::now() - tessStart).count() << " s\n";

    UsdStageRefPtr stage = UsdStage::CreateNew(outputPath.string());
    if (!stage) {
        std::cerr << "Failed to create stage at " << outputPath << "\n";
        return;
    }

    UsdGeomSetStageUpAxis(stage, UsdGeomTokens->z);
    stage->SetMetadata(TfToken("metersPerUnit"), 0.001);

    if (!UsdGeomXform::Define(stage, SdfPath("/Assembly"))) {
        std::cerr << "Failed to define root /Assembly\n";
        return;
    }

    if (!UsdGeomXform::Define(stage, SdfPath("/Prototypes"))) {
        std::cerr << "Failed to define /Prototypes\n";
        return;
    }

    // Define and immediately populate each prototype — no deferred writes,
    // no SdfChangeBlock. Geometry must be in the layer before references
    // to it are wired up in the instance hierarchy below.
    LabelMap<SdfPath> prototypePaths;
    for (int i = 0; i < (int)defs.size(); i++) {
        const TessResult& r = tessResults[i];
        if (!r.valid) continue;

        std::string name = "Def_" + std::to_string(i);

        SdfPath protoPath = SdfPath("/Prototypes")
            .AppendChild(TfToken(name));

        UsdGeomMesh proto = UsdGeomMesh::Define(stage, protoPath);
        if (!proto) {
            std::cerr << "Failed to define prototype at " << protoPath << "\n";
            continue;
        }

        proto.GetPointsAttr().Set(r.points);
        proto.GetFaceVertexCountsAttr().Set(r.faceVertexCounts);
        proto.GetFaceVertexIndicesAttr().Set(r.faceVertexIndices);
        proto.GetSubdivisionSchemeAttr().Set(UsdGeomTokens->none);
        proto.SetNormalsInterpolation(UsdGeomTokens->faceVarying);
        proto.GetNormalsAttr().Set(r.normals);

        prototypePaths[defs[i].first] = protoPath;

        //std::cout << "  Prototype " << protoPath << " -> " << r.points.size() << " verts\n";
    }

    std::cout << "Prototypes written: " << prototypePaths.size() << "\n";

    // Hide /Prototypes from renderers
    // USD will complain and not define prims under an inactive parent
    UsdPrim prototypeRoot = stage->GetPrimAtPath(SdfPath("/Prototypes"));
    if (prototypeRoot.IsValid())
        prototypeRoot.SetActive(false);

    // pre-order guarantees parent path is always assigned before we 
    // reach any of its children or USD will omplain about missing 
    // parent prims when we try to define them

    std::unordered_map<std::string, int> nameCounts;

    std::vector<SdfPath> paths(instances.size());
    for (size_t i = 0; i < instances.size(); i++) {
        const PartInstance& inst = instances[i];

        SdfPath parentPath;
        
        if (instances[i].parentIdx == -1) {
            parentPath = SdfPath("/Assembly");
        } else {
            parentPath = paths[instances[i].parentIdx];
        }

        int count = nameCounts[inst.name]++;

        std::string finalName = sanitizeUSDName(inst.name, count);

        paths[i] = parentPath.AppendChild(TfToken(finalName));
    }

    // Define all xform nodes, wire references, and author transforms
    for (size_t i = 0; i < instances.size(); i++) {
        const PartInstance& inst = instances[i];

        UsdGeomXform xform = UsdGeomXform::Define(stage, paths[i]);
        if (!xform) {
            std::cerr << "[" << i << "] Failed to define Xform at " << paths[i] << "\n";
            continue;
        }

        // USD composes the full world transform later
        xform.AddTransformOp().Set(trsfToGfMatrix(inst.localTransform));

        if (inst.type == InstanceType::Leaf) {
            auto it = prototypePaths.find(inst.definitionLabel);
            if (it == prototypePaths.end()) {
                std::cerr << "[" << i << "] No prototype for leaf\n";
                continue;
            }

            // Child mesh prim pulls geometry from the prototype via reference.
            // there is the Def_X in Prototypes
            // Assembly Node references /Prototypes/Def_X
            SdfPath meshPath = paths[i].AppendChild(TfToken("mesh"));
            UsdGeomMesh meshPrim = UsdGeomMesh::Define(stage, meshPath);
            if (!meshPrim) {
                std::cerr << "[" << i << "] Failed to define mesh at " << meshPath << "\n";
                continue;
            }

            meshPrim.GetPrim().GetReferences().AddInternalReference(it->second);
            if (inst.color.has_value()) {
                pxr::VtArray<pxr::GfVec3f> displayColor = {{
                    static_cast<float>(inst.color.value().Red()),
                    static_cast<float>(inst.color.value().Green()),
                    static_cast<float>(inst.color.value().Blue())
                }};
                meshPrim.GetDisplayColorAttr().Set(displayColor);
            }

            meshPrim.GetPrim().SetDisplayName(inst.name);
        }
    }

    if (!mark.IsClean()) {
        for (const auto& error : mark)
            std::cerr << "USD: " << error.GetCommentary() << "\n";
    }

    auto saveStart = Clock::now();
    stage->GetRootLayer()->Save();

    std::cout << "Layer save time:    " << Seconds(Clock::now() - saveStart).count() << " s\n";
    std::cout << "Total export time:  " << Seconds(Clock::now() - totalStart).count() << " s\n";
    std::cout << "Saved USD to " << outputPath << "\n";
}