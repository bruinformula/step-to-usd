#include <cassert>

#include <opencascade/BRepBuilderAPI_Transform.hxx>
#include <opencascade/BinXCAFDrivers.hxx>
#include <opencascade/STEPCAFControl_Reader.hxx>
#include <opencascade/TDocStd_Application.hxx>
#include <opencascade/TDocStd_Document.hxx>
#include <opencascade/XCAFDoc_DocumentTool.hxx>
#include <opencascade/XCAFDoc_ShapeTool.hxx>
#include <opencascade/TDF_Label.hxx>
#include <opencascade/TDataStd_Name.hxx>
#include <opencascade/TopLoc_Location.hxx>
#include <opencascade/gp_Trsf.hxx>
#include <opencascade/BRepMesh_IncrementalMesh.hxx>
#include <opencascade/BRep_Tool.hxx>
#include <opencascade/StlAPI_Writer.hxx>
#include <opencascade/TopExp_Explorer.hxx>
#include <opencascade/TopoDS.hxx>
#include <opencascade/Poly_Triangulation.hxx>
#include <OSD_Parallel.hxx>

#pragma push_macro("Handle") // pxr, CGAL, and occt all define Handle
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

#include "STEPModel.h"

std::optional<STEPModel> STEPModel::loadFromFile(const fs::path& stepPath) {
    try {
        OSD_Parallel::SetUseOcctThreads(true);

        occt::handle<TDocStd_Application> app = new TDocStd_Application();
        BinXCAFDrivers::DefineFormat(app);

        fs::path xbfPath = stepPath;
        xbfPath.replace_extension("xbf");

        occt::handle<TDocStd_Document> doc;

        // xbf files are a binary representation of the document
        // makes reloading much faster after the first time, but
        // they are not human-readable and can get out of sync
        // with the step file if the step file changes
        if (!fs::exists(xbfPath) || fs::last_write_time(xbfPath) < fs::last_write_time(stepPath)) {
            std::cout << "XBF doesn't exist or is out of date. Building from STEP..." << std::endl;
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

            std::cout << "Saving XBF to " << xbfPath << std::endl;
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
        return STEPModel(app, doc, shapeTool);

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

    // Pass 1: count total nodes in the tree so we can reserve exactly
    // the right amount and assign contiguous child slots up front
    int numNodes = 0;
    for (int i = 1; i <= freeShapes.Length(); i++)
        numNodes += countNodes(freeShapes.Value(i));

    instances.resize(numNodes);

    // Pass 2: fill instances[] in pre-order using a write cursor.
    // Each assembly claims a contiguous block for its children before
    // recursing, so [firstChildIdx, firstChildIdx + childCount) is
    // always a valid contiguous range.
    int cursor = 0; // next free slot in instances[]

    gp_Trsf identity;
    for (int i = 1; i <= freeShapes.Length(); i++)
        fillNode(freeShapes.Value(i), identity, -1, 0, cursor);

    // cursor should have consumed exactly numNodes slots
    assert(cursor == numNodes);

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
                    << " defTag="     << inst.definitionTag
                    << " localT=("    << lt.X() << "," << lt.Y() << "," << lt.Z() << ")"
                    << "\n";
    }
}

const TopoDS_Shape& STEPModel::getDefinitionShape(const PartInstance& inst) const {
    return definitionShapes.at(inst.definitionTag);
}

void STEPModel::writeUSD(const fs::path& outputPath) const {
    using namespace pxr;

    TfErrorMark mark;
    
    struct TessResult {
        VtArray<GfVec3f> points;
        VtArray<int>     faceVertexCounts;
        VtArray<int>     faceVertexIndices;
        bool             valid = false;
    };

    std::vector<TessResult> tessResults(instances.size());

    // Step 1: tessellate all leaves in parallel — pure OCC, no USD
    // USD composes the transform hierarchy itself via local xform ops.

    OSD_Parallel::For(0, (int)instances.size(), [&](int i) {
        const PartInstance& inst = instances[i];
        if (inst.type != InstanceType::Leaf) return;

        const TopoDS_Shape& defShape = getDefinitionShape(inst); // Copy BRepMesh_IncrementalMesh is not thread-safe
        BRepBuilderAPI_Transform copier(defShape, gp_Trsf(), true);
        TopoDS_Shape localShape = copier.Shape();

        BRepMesh_IncrementalMesh(localShape, 0.1).Perform();

        TessResult& result = tessResults[i];
        int vertexOffset = 0;

        for (TopExp_Explorer exp(localShape, TopAbs_FACE); exp.More(); exp.Next()) {
            const TopoDS_Face& face = TopoDS::Face(exp.Current());
            TopLoc_Location loc;
            Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
            if (tri.IsNull()) continue;

            gp_Trsf trsf = loc.Transformation();

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
                if (face.Orientation() == TopAbs_REVERSED) std::swap(n2, n3);
                result.faceVertexCounts.push_back(3);
                result.faceVertexIndices.push_back(vertexOffset + n1 - 1);
                result.faceVertexIndices.push_back(vertexOffset + n2 - 1);
                result.faceVertexIndices.push_back(vertexOffset + n3 - 1);
            }

            vertexOffset += tri->NbNodes();
        }

        result.valid = !result.points.empty();
        if (!result.valid)
            std::cerr << "[" << i << "] Warning: tessellation produced no geometry\n";
    });

    // USD Compose Stuff 
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

    // pre-order guarantees parent path is filled before child. otherwise USD will complain
    std::vector<SdfPath> paths(instances.size());
    for (size_t i = 0; i < instances.size(); i++) {
        SdfPath parentPath = (instances[i].parentIdx == -1)
            ? SdfPath("/Assembly")
            : paths[instances[i].parentIdx];
        paths[i] = parentPath.AppendChild(TfToken("Node_" + std::to_string(i)));
    }

    // Parents must exist in the stage before children can be defined
    for (size_t i = 0; i < instances.size(); i++) {
        if (instances[i].type == InstanceType::Assembly) {
            if (!UsdGeomXform::Define(stage, paths[i]))
                std::cerr << "[" << i << "] Failed to define Xform at " << paths[i] << "\n";
        } else {
            if (!UsdGeomMesh::Define(stage, paths[i]))
                std::cerr << "[" << i << "] Failed to define Mesh at " << paths[i] << "\n";
        }
    }

    // All prims already exist so GetPrimAtPath is safe
    { // SdfChangeBlock 
        SdfChangeBlock block;

        for (size_t i = 0; i < instances.size(); i++) {
            const PartInstance& inst = instances[i];

            UsdPrim prim = stage->GetPrimAtPath(paths[i]);
            if (!prim.IsValid()) {
                std::cerr << "[" << i << "] Prim missing at " << paths[i] << "\n";
                continue;
            }

            if (inst.type == InstanceType::Assembly) {
                UsdGeomXform(prim).AddTransformOp()
                    .Set(trsfToGfMatrix(inst.localTransform));

            } else {
                const TessResult& r = tessResults[i];
                if (!r.valid) continue;

                // Local transform positions the mesh within its parent assembly.
                UsdGeomMesh mesh(prim);
                mesh.AddTransformOp().Set(trsfToGfMatrix(inst.localTransform));

                mesh.GetPointsAttr().Set(r.points);
                mesh.GetFaceVertexCountsAttr().Set(r.faceVertexCounts);
                mesh.GetFaceVertexIndicesAttr().Set(r.faceVertexIndices);
                mesh.GetSubdivisionSchemeAttr().Set(UsdGeomTokens->none);
            }
        }
    } // SdfChangeBlock 

    if (!mark.IsClean()) {
        for (const auto& error : mark)
            std::cerr << "USD: " << error.GetCommentary() << "\n";
    }

    stage->GetRootLayer()->Save();
    std::cout << "Saved USD to " << outputPath << "\n";
}

// Counting
int STEPModel::countNodes(const TDF_Label& label) {
    if (!shapeTool->IsShape(label)) return 0;

    if (shapeTool->IsComponent(label)) { // components are instances
        TDF_Label defLabel;
        if (!shapeTool->GetReferredShape(label, defLabel)) return 0;

        if (shapeTool->IsSimpleShape(defLabel))  return 1; // just the leaf
        if (shapeTool->IsAssembly(defLabel))     return 1 + countAssemblyChildren(defLabel);
        return 1; // defensive: treat unknown as leaf
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

    // localTransform is just this label's own location, nothing accumulated
    // parentWorld is only used here to pass down to children
    TopLoc_Location loc       = shapeTool->GetLocation(label);
    gp_Trsf         localTrsf = loc.Transformation();
    gp_Trsf         world     = parentWorld * localTrsf; // for children only

    if (shapeTool->IsComponent(label)) {
        TDF_Label defLabel;
        if (!shapeTool->GetReferredShape(label, defLabel)) return;

        if (shapeTool->IsSimpleShape(defLabel)) {
            fillLeaf(defLabel, localTrsf, parentIdx, depth, cursor);
        } else if (shapeTool->IsAssembly(defLabel)) {
            fillAssembly(defLabel, localTrsf, world, parentIdx, depth, cursor);
        } else {
            fillLeaf(defLabel, localTrsf, parentIdx, depth, cursor); // defensive
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

    instances[myIdx].type           = InstanceType::Leaf;
    instances[myIdx].definitionTag  = defLabel.Tag();
    instances[myIdx].localTransform = localTrsf;
    instances[myIdx].parentIdx      = parentIdx;
    instances[myIdx].firstChildIdx  = -1;
    instances[myIdx].childCount     = 0;
    instances[myIdx].depth          = depth;

    if (definitionShapes.find(defLabel.Tag()) == definitionShapes.end())
        definitionShapes[defLabel.Tag()] = shapeTool->GetShape(defLabel);
}

void STEPModel::fillAssembly(
    const TDF_Label& defLabel,
    const gp_Trsf& localTrsf,
    const gp_Trsf& world, // passed to children so they can compute their local
    int parentIdx,
    int depth,
    int& cursor
) {
    int myIdx = cursor++; // claim this node's slot

    NCollection_Sequence<TDF_Label> components;
    shapeTool->GetComponents(defLabel, components);

    // Count direct children 
    // This is not full subtree yet — just immediate components
    int validChildren = 0;
    for (int i = 1; i <= components.Length(); i++) {
        if (countNodes(components.Value(i)) > 0) {
            validChildren++;
        }
    }

    // Children will occupy [cursor, cursor + subtreeSize). cursor starts at the first child slot
    int firstChild = (validChildren > 0) ? cursor : -1;

    instances[myIdx].type           = InstanceType::Assembly;
    instances[myIdx].definitionTag  = defLabel.Tag();
    instances[myIdx].localTransform = localTrsf;
    instances[myIdx].parentIdx      = parentIdx;
    instances[myIdx].firstChildIdx  = firstChild;
    instances[myIdx].childCount     = validChildren;
    instances[myIdx].depth          = depth;

    for (int i = 1; i <= components.Length(); i++)
        fillNode(components.Value(i), world, myIdx, depth + 1, cursor);
}