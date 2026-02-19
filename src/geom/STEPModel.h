#pragma once

#include <cassert>
#include <filesystem>
#include <iostream>
#include <opencascade/BRepBuilderAPI_Transform.hxx>
#include <optional>
#include <map>

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
#include <opencascade/StlAPI_Writer.hxx>
#include <OSD_Parallel.hxx>

namespace occt = opencascade;
namespace fs = std::filesystem;

enum class InstanceType {
    Assembly,
    Leaf
};

struct PartInstance {
    InstanceType type;
    int definitionTag;  // unique id of the geometry definition
    gp_Trsf worldTransform;
    int parentIdx; // -1 if root
    int firstChildIdx; // -1 if leaf
    int childCount;
    int depth;
};

struct STEPModel {

    STEPModel(occt::handle<TDocStd_Application> a, occt::handle<TDocStd_Document> d, occt::handle<XCAFDoc_ShapeTool> st) : 
        app(a),    
        doc(d), 
        shapeTool(st) 
    {}

    static std::optional<STEPModel> loadFromFile(const fs::path& stepPath) {
        try {
            OSD_Parallel::SetUseOcctThreads(true);

            occt::handle<TDocStd_Application> app = new TDocStd_Application();
            BinXCAFDrivers::DefineFormat(app);

            fs::path xbfPath = stepPath;
            xbfPath.replace_extension("xbf");

            occt::handle<TDocStd_Document> doc;

            if (!fs::exists(xbfPath) || fs::last_write_time(xbfPath) < fs::last_write_time(stepPath)) {
                std::cout << "XBF out of date, building from STEP...\n";
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

    // Instance
    void buildInstanceTree() {
        instances.clear();
        definitionShapes.clear();

        NCollection_Sequence<TDF_Label> freeShapes;
        shapeTool->GetFreeShapes(freeShapes);

        // Get a count of the total number of nodes 
        // in the tree. Nodes ares simple shapes or assemblies
        int numNodes = 0;
        for (int i = 1; i <= freeShapes.Length(); i++) {
            numNodes += countNodes(freeShapes.Value(i));
        }

        instances.resize(numNodes);

        // Pass 2: fill instances[] in pre-order using a write cursor.
        // Each assembly claims a contiguous block for its children
        // [firstChildIdx, firstChildIdx + childCount)

        int cursor = 0; // next free slot in instances[]

        gp_Trsf identity;
        for (int i = 1; i <= freeShapes.Length(); i++) {
            fillNode(freeShapes.Value(i), identity, -1, 0, cursor);
        }

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

    void printInstanceTree() const {
        for (const auto& inst : instances) {
            std::cout << "Instance: defTag=" << inst.definitionTag 
                      << " type=" << (inst.type == InstanceType::Assembly ? "Assembly" : "Leaf")
                      << " parentIdx=" << inst.parentIdx
                      << " childCount=" << inst.childCount
                      << " depth=" << inst.depth
                      << "\n";
        }
    }

    void printDefinitionShapes() const {
        for (const auto& [defTag, shape] : definitionShapes) {
            std::cout << "Definition Tag: " << defTag << "\n";
            std::cout << "  Shape Type: " << shape.ShapeType() << "\n";
            std::cout << "  Number of Subshapes: " << shape.NbChildren() << "\n";
        }
    }

    const TopoDS_Shape& getDefinitionShape(const PartInstance& inst) const {
        return definitionShapes.at(inst.definitionTag);
    }

    void writeMeshTest() const {
        int leafIdx = 0;
        for (const auto& inst : instances) {
            if (inst.type != InstanceType::Leaf) continue;

            const TopoDS_Shape& shape = getDefinitionShape(inst);

            BRepBuilderAPI_Transform xform(shape, inst.worldTransform, true);
            TopoDS_Shape worldShape = xform.Shape();

            fs::path path = "leaf_" + std::to_string(leafIdx++) + ".stl";

            BRepMesh_IncrementalMesh mesh(worldShape, 0.1);
            mesh.Perform();

            StlAPI_Writer writer;
            writer.Write(worldShape, path.c_str());
        }
    }
    
    occt::handle<TDocStd_Application> app;
    occt::handle<TDocStd_Document> doc;
    occt::handle<XCAFDoc_ShapeTool> shapeTool;

    
    std::vector<PartInstance> instances; // the raw defintions of parts in the tree
    std::map<int, TopoDS_Shape> definitionShapes; // int is the unique definitionTag : shape

private:
    // Counts 
    int countNodes(const TDF_Label& label) {
        if (!shapeTool->IsShape(label)) 
            return 0;

        if (shapeTool->IsComponent(label)) { // components are instances
            TDF_Label defLabel;
            if (!shapeTool->GetReferredShape(label, defLabel)) 
                return 0;

            if (shapeTool->IsSimpleShape(defLabel)) {
                return 1; // just the leaf
            } else if (shapeTool->IsAssembly(defLabel)) {
                return 1 + countAssemblyChildren(defLabel); // assembly node + subtree
            } else {
                return 1; // this is defensive. treat unknown as leaf
            }

        } else if (shapeTool->IsAssembly(label)) {
            return 1 + countAssemblyChildren(label);

        } else if (shapeTool->IsSimpleShape(label)) {
            return 1;
        }

        return 0;
    }

    int countAssemblyChildren(const TDF_Label& assemblyDef) {
        NCollection_Sequence<TDF_Label> components;
        shapeTool->GetComponents(assemblyDef, components);

        int total = 0;
        for (int i = 1; i <= components.Length(); i++) {
            total += countNodes(components.Value(i));
        }
        return total;
    }

    // Filling
    void fillNode(
        const TDF_Label& label,
        const gp_Trsf& parentWorld,
        int parentIdx,
        int depth,
        int& cursor
    ) {
        if (!shapeTool->IsShape(label)) 
            return;

        TopLoc_Location loc = shapeTool->GetLocation(label);
        gp_Trsf world = parentWorld * loc.Transformation();

        if (shapeTool->IsComponent(label)) {
            TDF_Label defLabel;
            if (!shapeTool->GetReferredShape(label, defLabel)) 
                return;

            if (shapeTool->IsSimpleShape(defLabel)) {
                fillLeaf(defLabel, world, parentIdx, depth, cursor);
            } else if (shapeTool->IsAssembly(defLabel)) {
                fillAssembly(defLabel, world, parentIdx, depth, cursor);
            } else {
                fillLeaf(defLabel, world, parentIdx, depth, cursor); // defensive
            }

        } else if (shapeTool->IsAssembly(label)) {
            fillAssembly(label, world, parentIdx, depth, cursor);

        } else if (shapeTool->IsSimpleShape(label)) {
            fillLeaf(label, world, parentIdx, depth, cursor);
        }
    }

    void fillLeaf(
        const TDF_Label& defLabel,
        const gp_Trsf&   world,
        int parentIdx,
        int depth,
        int& cursor
    ) {
        int myIdx = cursor++;

        instances[myIdx].type           = InstanceType::Leaf;
        instances[myIdx].definitionTag  = defLabel.Tag();
        instances[myIdx].worldTransform = world;
        instances[myIdx].parentIdx      = parentIdx;
        instances[myIdx].firstChildIdx  = -1;
        instances[myIdx].childCount     = 0;
        instances[myIdx].depth          = depth;

        if (definitionShapes.find(defLabel.Tag()) == definitionShapes.end()) {
            definitionShapes[defLabel.Tag()] = shapeTool->GetShape(defLabel);
        }
    }

    void fillAssembly(
        const TDF_Label& defLabel,
        const gp_Trsf&   world,
        int parentIdx,
        int depth,
        int& cursor
    ) {
        int myIdx = cursor++; // claim this node's slot

        // Count direct children 
        // This is not full subtree yet — just immediate components
        NCollection_Sequence<TDF_Label> components;
        shapeTool->GetComponents(defLabel, components);

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
        instances[myIdx].worldTransform = world;
        instances[myIdx].parentIdx      = parentIdx;
        instances[myIdx].firstChildIdx  = firstChild;
        instances[myIdx].childCount     = validChildren;
        instances[myIdx].depth          = depth;

        for (int i = 1; i <= components.Length(); i++) {
            fillNode(components.Value(i), world, myIdx, depth + 1, cursor);
        }
    }
};
