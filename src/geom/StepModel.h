#pragma once

#include <stddef.h>
#include <filesystem>
#include <optional>
#include <unordered_map>
#include <functional>
#include <string>
#include <vector>

#include <opencascade/gp_Trsf.hxx>
#include <opencascade/TDF_Label.hxx>
#include <opencascade/TopoDS_Shape.hxx>
#include <opencascade/Quantity_Color.hxx>
#include <opencascade/Standard_Handle.hxx>

class TDocStd_Application;
class TDocStd_Document;
class XCAFDoc_ColorTool;
class XCAFDoc_LayerTool;
class XCAFDoc_MaterialTool;
class XCAFDoc_ShapeTool;

#pragma push_macro("Handle") // pxr, CGAL, and occt all define Handle as a macro
#undef Handle

#pragma pop_macro("Handle")

namespace occt = opencascade;
namespace fs = std::filesystem;

// TDF_Label is a handle into the document's label tree. Two labels pointing
// at the same node are equal — we use that for deduplicating definitions.
// We hash by walking the tag chain to the root, which uniquely identifies
// any node in the tree.
struct LabelHash {
    size_t operator()(const TDF_Label& label) const {
        size_t h = 0;
        TDF_Label l = label;
        while (!l.IsNull()) {
            h ^= std::hash<int>{}(l.Tag()) + 0x9e3779b9 + (h << 6) + (h >> 2);
            l = l.Father();
        }
        return h;
    }
};

struct LabelEqual {
    bool operator()(const TDF_Label& a, const TDF_Label& b) const {
        return a.IsEqual(b);
    }
};

template<typename V>
using LabelMap = std::unordered_map<TDF_Label, V, LabelHash, LabelEqual>;

std::string getLabelName(const TDF_Label& label);

struct StepModel {

    enum class InstanceType {
        Assembly,
        Leaf
    };

    struct PartInstance {
        InstanceType type;
        TDF_Label definitionLabel; // this is the key into definitionShapes
        gp_Trsf localTransform;  // this node's transform relative to its parent only
        int parentIdx;       // index into instances[], -1 if root
        int firstChildIdx;   // first child index, children occupy contiguous range in instances[]
        int childCount;
        int depth;
        bool visible = true;

        std::string name;
        std::optional<Quantity_Color> color;
    };

    StepModel(
        occt::handle<TDocStd_Application> a,
        occt::handle<TDocStd_Document>    d,
        occt::handle<XCAFDoc_ShapeTool>   st,
        occt::handle<XCAFDoc_ColorTool>    ct,
        occt::handle<XCAFDoc_MaterialTool>   mt,
        occt::handle<XCAFDoc_LayerTool>   lt,
        double metersPerUnit
    ) : app(a), doc(d), shapeTool(st), colorTool(ct), materialTool(mt), layerTool(lt), metersPerUnit(metersPerUnit) {}

    static std::optional<StepModel> loadFromFile(const fs::path& stepPath);

    void buildInstanceTree();

    void debugPrintInstances() const;

    occt::handle<TDocStd_Application> app;
    occt::handle<TDocStd_Document> doc;
    occt::handle<XCAFDoc_ShapeTool> shapeTool;
    occt::handle<XCAFDoc_ColorTool> colorTool;
    occt::handle<XCAFDoc_MaterialTool> materialTool;
    occt::handle<XCAFDoc_LayerTool> layerTool;

    std::vector<PartInstance> instances;        // flat pre-order instance tree
    LabelMap<TopoDS_Shape> definitionShapes; // definition label -> geometry
    double metersPerUnit;

private:
    int countNodes(const TDF_Label& label);
    int countAssemblyChildren(const TDF_Label& assemblyDef);
    bool isLabelVisible(const TDF_Label& label) const;

    void fillNode(
        const TDF_Label& label,
        const gp_Trsf&   parentWorld,
        int              parentIdx,
        int              depth,
        int&             cursor
    );

    void fillLeaf(
        const TDF_Label& defLabel,
        const gp_Trsf&   localTrsf,
        int              parentIdx,
        int              depth,
        int&             cursor
    );

    void fillAssembly(
        const TDF_Label& defLabel,
        const gp_Trsf&   localTrsf,
        const gp_Trsf&   world,
        int              parentIdx,
        int              depth,
        int&             cursor
    );
};