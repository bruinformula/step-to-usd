#pragma once

#include <cassert>
#include <filesystem>
#include <optional>
#include <unordered_map>

#include <opencascade/gp_Trsf.hxx>
#include <opencascade/TDF_Label.hxx>
#include <opencascade/TDocStd_Application.hxx>
#include <opencascade/TDocStd_Document.hxx>
#include <opencascade/TopoDS_Shape.hxx>
#include <opencascade/XCAFDoc_ShapeTool.hxx>
#include <opencascade/Quantity_Array1OfColor.hxx>
#include <opencascade/XCAFDoc_DocumentTool.hxx>

#pragma push_macro("Handle") // pxr, CGAL, and occt all define Handle as a macro
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

    std::string name;
    std::optional<Quantity_Color> color;
};

enum class WireframeMode {
    None,
    Linear,          // polyline using the tessellated mesh boundary vertices directly
    ResampledLinear, // polyline resampled from the underlying curve geometry
    CatmullRom,      // cubic Catmull-Rom resampled from the underlying curve geometry
};

enum class SketchMode {
    None,
    Linear,          // polyline sampled from the STEP curve at wireframe deflection
    ResampledLinear, // polyline resampled with independent sketch deflection
    CatmullRom,      // cubic Catmull-Rom with independent sketch deflection
};

struct STEPModel {

    struct TessResult {
        pxr::VtArray<pxr::GfVec3f> points;
        pxr::VtArray<pxr::GfVec3f> normals;
        pxr::VtArray<int> faceVertexCounts;
        pxr::VtArray<int> faceVertexIndices;

        pxr::VtArray<pxr::GfVec2f> perSurfaceUVs;
        pxr::VtArray<int> surfaceIDs;
        pxr::VtArray<bool> isBoundaryVertex;

        // Wireframe curves 
        pxr::VtArray<pxr::GfVec3f> curvePoints;
        pxr::VtArray<int> curveCounts;
        pxr::VtArray<int> curveContinuity;

        // Sketch curves
        pxr::VtArray<pxr::GfVec3f> sketchPoints;
        pxr::VtArray<int> sketchCounts;
    };

    struct TessParams {
        std::optional<float> lodCullingMinimumSize = std::nullopt; // in the units of the model along the diagonal

        float meshLinearDeflection = 0.05f; // as a fraction of the diagonal of the bounding box
        float meshAngularDeflection = 0.35f;
        float meshMinSize = 0.1f; // as a fraction of the linear deflection

        float wireframeDeflection = 1.0f;
        WireframeMode wireframeMode = WireframeMode::Linear;

        float sketchDeflection = 0.5f;
        SketchMode sketchMode = SketchMode::Linear;
    };

    STEPModel(
        occt::handle<TDocStd_Application> a,
        occt::handle<TDocStd_Document>    d,
        occt::handle<XCAFDoc_ShapeTool>   st,
        occt::handle<XCAFDoc_ColorTool>    ct,
        occt::handle<XCAFDoc_MaterialTool>   mt
    ) : app(a), doc(d), shapeTool(st), colorTool(ct), materialTool(mt) {}

    static std::optional<STEPModel> loadFromFile(const fs::path& stepPath);

    void buildInstanceTree();

    void debugPrintInstances() const;
    
    bool tesselatePart(TessResult& result, const TopoDS_Shape& defShape, const TessParams& params) const;
    void populateUSD(pxr::UsdStageRefPtr stage, const TessParams& params) const;

    occt::handle<TDocStd_Application> app;
    occt::handle<TDocStd_Document> doc;
    occt::handle<XCAFDoc_ShapeTool> shapeTool;
    occt::handle<XCAFDoc_ColorTool> colorTool;
    occt::handle<XCAFDoc_MaterialTool> materialTool;

    std::vector<PartInstance> instances;        // flat pre-order instance tree
    LabelMap<TopoDS_Shape> definitionShapes; // definition label -> geometry

private:
    int countNodes(const TDF_Label& label);
    int countAssemblyChildren(const TDF_Label& assemblyDef);

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