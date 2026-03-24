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
#include <opencascade/STEPCAFControl_Reader.hxx>

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
    bool visible = true;

    std::string name;
    std::optional<Quantity_Color> color;
};

enum class CurveType {
    None,                // ain't got nothing on me
    Linear,              // polyline using the tessellated mesh boundary vertices directly
    CatmullRom          // cubic Catmull-Rom using the tessellated mesh boundary vertices directly
};

enum class CurveSampling {
    Underlying, // polyline using the tessellated mesh boundary vertices
    Resampled   // resampled from the underlying curve geometry
};

struct CurveMode {
    CurveType type;
    CurveSampling sampling;
};

struct StepModel {

    struct TessResult {
        pxr::VtArray<pxr::GfVec3f> points;
        pxr::VtArray<pxr::GfVec3f> normals;
        pxr::VtArray<int> faceVertexCounts;
        pxr::VtArray<int> faceVertexIndices;

        pxr::VtArray<pxr::GfVec2f> perSurfaceUVs;
        pxr::VtArray<int> surfaceIDs;
        pxr::VtArray<bool> isBoundaryVertex;

        struct SurfaceIDBounds {
            int startIdx;
            int endIdx;
            int surfaceID;
        };

        std::vector<SurfaceIDBounds> surfaceIDBounds; // start, end index for a particular face id 

        // Wireframe curves 
        pxr::VtArray<pxr::GfVec3f> curvePoints;
        pxr::VtArray<int> curveCounts;
        pxr::VtArray<int> curveContinuity;

        // Sketch curves
        pxr::VtArray<pxr::GfVec3f> sketchPoints;
        pxr::VtArray<int> sketchCounts;

        // Render Purpose
        bool renderOnly; 
    };

    struct TessParams {
        std::optional<float> renderPurposeThreshold = std::nullopt; 
        // in the units of the model along the diagonal. 
        // if proto is smaller it gets marked as a render only asset

        float meshLinearDeflection = 0.05f; // as a fraction of the diagonal of the bounding box
        float meshAngularDeflection = 0.35f;
        float meshMinSize = 0.1f; // as a fraction of the linear deflection

        float wireframeDeflection = 1.0f;
        CurveMode wireframeMode = { CurveType::Linear, CurveSampling::Underlying };

        float sketchDeflection = 0.5f;
        CurveMode sketchMode = { CurveType::Linear, CurveSampling::Underlying };

        bool defaultMeshVisibility = true;
        bool defaultWireframeVisibility = false;
        bool defaultSketchVisibility = false;
    };

    struct VariantParams {
        TessParams tessParams;
        std::string variantName;
        std::string variantSetName;
        std::filesystem::path outpath;  
        std::filesystem::path refpath;
        bool overwrite = false;
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
    static double readStepLengthUnit(STEPControl_Reader& cafReader);

    void buildInstanceTree();

    void debugPrintInstances() const;
    
    bool tesselatePart(TessResult& result, const TopoDS_Shape& defShape, const TessParams& params) const;
    void populateUsd(pxr::UsdStageRefPtr stage, const TessParams& params) const;
    void populateVariantUsd(pxr::UsdStageRefPtr stage, const std::vector<VariantParams>& variantParams) const;

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
    static std::map<pxr::SdfPath, StepModel::TessParams> resolveParams(const pxr::UsdPrim& rootPrim);

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

    std::vector<pxr::SdfPath> computeInstancePaths() const;
    void writeInstanceXforms(
        pxr::UsdStageRefPtr stage,
        const std::vector<pxr::SdfPath>& paths,
        const LabelMap<pxr::SdfPath>& prototypePaths
    ) const;
};