#pragma once

#include <cassert>
#include <filesystem>
#include <optional>
#include <map>

#include <opencascade/gp_Trsf.hxx>
#include <opencascade/TDocStd_Application.hxx>
#include <opencascade/TDocStd_Document.hxx>
#include <opencascade/TopoDS_Shape.hxx>
#include <opencascade/XCAFDoc_ShapeTool.hxx>

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

namespace occt = opencascade;
namespace fs   = std::filesystem;

enum class InstanceType {
    Assembly,
    Leaf
};

struct PartInstance {
    InstanceType type;
    int definitionTag;  // unique id of the geometry definition
    gp_Trsf localTransform; // this node's local transform only
    int parentIdx; // -1 if root
    int firstChildIdx; // index into instances[], -1 if leaf
    int childCount;
    int depth;
};

// rotation block: transposed relative to OCC Value(row,col) convention
// translation: from TranslationPart() into the last row
inline pxr::GfMatrix4d trsfToGfMatrix(const gp_Trsf& t) {
    gp_XYZ trans = t.TranslationPart();
    auto clean = [](double v) { return std::abs(v) < 1e-10 ? 0.0 : v; };
    return pxr::GfMatrix4d(
        clean(t.Value(1,1)), clean(t.Value(2,1)), clean(t.Value(3,1)), 0.0,
        clean(t.Value(1,2)), clean(t.Value(2,2)), clean(t.Value(3,2)), 0.0,
        clean(t.Value(1,3)), clean(t.Value(2,3)), clean(t.Value(3,3)), 0.0,
        clean(trans.X()),    clean(trans.Y()),    clean(trans.Z()),    1.0
    );
}

struct STEPModel {

    STEPModel(
        occt::handle<TDocStd_Application> a,
        occt::handle<TDocStd_Document>    d,
        occt::handle<XCAFDoc_ShapeTool>   st
    ) : app(a), doc(d), shapeTool(st) {}

    static std::optional<STEPModel> loadFromFile(const fs::path& stepPath);

    void buildInstanceTree();

    void debugPrintInstances() const;

    const TopoDS_Shape& getDefinitionShape(const PartInstance& inst) const;

    void writeUSD(const fs::path& outputPath) const;

    occt::handle<TDocStd_Application> app;
    occt::handle<TDocStd_Document> doc;
    occt::handle<XCAFDoc_ShapeTool> shapeTool;

    std::vector<PartInstance> instances; // the flat pre-order instance tree
    std::map<int, TopoDS_Shape> definitionShapes; // definitionTag -> shape, built during traversal

private:
    // Counting
    int countNodes(const TDF_Label& label);

    int countAssemblyChildren(const TDF_Label& assemblyDef);

    // Filling
    void fillNode(
        const TDF_Label& label,
        const gp_Trsf& parentWorld,
        int parentIdx,
        int depth,
        int& cursor
    );

    void fillLeaf(
        const TDF_Label& defLabel,
        const gp_Trsf& localTrsf,
        int parentIdx,
        int depth,
        int& cursor
    );

    void fillAssembly(
        const TDF_Label& defLabel,
        const gp_Trsf& localTrsf,
        const gp_Trsf& world, // passed to children so they can compute their local
        int parentIdx,
        int depth,
        int& cursor
    );
};