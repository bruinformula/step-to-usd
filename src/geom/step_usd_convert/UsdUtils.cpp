#include <stddef.h>
#include <_ctype.h>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <unordered_map>
#include <cctype>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include <opencascade/gp_XYZ.hxx>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/pxr.h>
#include <pxr/usd/sdf/declareHandles.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/primSpec.h>

#include <pxr/usd/usd/common.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/inherits.h>

#include <pxr/usd/usdGeom/tokens.h>

#include <pxr/base/vt/array.h>
#include <pxr/base/vt/value.h>
#include <pxr/base/tf/staticData.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/tf/refPtr.h>
#include <pxr/base/gf/vec4f.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/vec2f.h>

#pragma pop_macro("Handle")

#include "tokens.h"

#include "UsdStepExporter.h"
#include "StepModel.h"

PXR_NAMESPACE_USING_DIRECTIVE

// rotation block: transposed relative to OCC Value(row,col) convention
// // translation: from TranslationPart() into the last row
GfMatrix4d UsdStepExporter::trsfToGfMatrix(const gp_Trsf& t) {
    gp_XYZ trans = t.TranslationPart();
    auto clean = [](double v) { return std::abs(v) < 1e-10 ? 0.0 : v; };
    return GfMatrix4d(
        clean(t.Value(1,1)), clean(t.Value(2,1)), clean(t.Value(3,1)), 0.0,
        clean(t.Value(1,2)), clean(t.Value(2,2)), clean(t.Value(3,2)), 0.0,
        clean(t.Value(1,3)), clean(t.Value(2,3)), clean(t.Value(3,3)), 0.0,
        clean(trans.X()),    clean(trans.Y()),    clean(trans.Z()),    1.0
    );
}

std::string UsdStepExporter::sanitizeUsdName(const std::string_view& name, int idx) {
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

VtArray<GfVec2f> UsdStepExporter::packUVAtlas(std::vector<UVPatch>& patches) {
    
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

std::vector<SdfPath> UsdStepExporter::computeNodePaths(
    const std::vector<StepModel::PartNode>& partNodes,
    const SdfPath& assemblyPath
) {
    std::unordered_map<std::string, int> nameCounts;
    std::vector<SdfPath> paths(partNodes.size());

    // pre-order guarantees parent path is always assigned before we 
    // reach any of its children or Usd will omplain about missing 
    // parent prims when we try to define them
    for (size_t i = 0; i < partNodes.size(); i++) {
        const StepModel::PartNode& node = partNodes[i];

        SdfPath parentPath;
        if (partNodes[i].parentIdx == -1) {
            parentPath = assemblyPath;
        } else {
            parentPath = paths[partNodes[i].parentIdx];
        }

        int count = nameCounts[node.name]++;
        std::string finalName = sanitizeUsdName(node.name, count);
        paths[i] = parentPath.AppendChild(TfToken(finalName));
    }

    return paths;
}

UsdStageRefPtr UsdStepExporter::initUsdStage(
    const fs::path& newStagePath, 
    const SdfPath& rootPrimPath,
    bool clearExisting
) {
    SdfLayerRefPtr layer = SdfLayer::FindOrOpen(newStagePath.string());
    
    if (!layer) {
        //std::cout << "Creating new layer at " << newStagePath << "\n";
        layer = SdfLayer::CreateNew(newStagePath.string());
    } else if (clearExisting) {
        //std::cout << "Cleaning internal contents of layer: " << newStagePath << "\n";
        layer->Clear();
    }

    if (!layer) return nullptr;

    // Set metadata on the layer so composition knows where to look
    layer->SetDefaultPrim(rootPrimPath.GetNameToken());
    layer->SetField(SdfPath::AbsoluteRootPath(), UsdGeomTokens->upAxis, VtValue(UsdGeomTokens->z));

    // Open the stage. 
    UsdStageRefPtr stage = UsdStage::Open(layer);

    if (!stage->GetPrimAtPath(rootPrimPath)) {
        stage->DefinePrim(rootPrimPath);
    }

    stage->Save();
    return stage;
}

template <typename T>
void updateIfAuthored(const UsdAttribute& attr, T* value) {
    bool hasValue = attr.HasValue();
    if (hasValue) {
        attr.Get(value);
    }
}

// Helper to read a token attr and convert to CurveType
template <>
void updateIfAuthored(const UsdAttribute& attr, CurveSampling* value) {
    bool hasValue = attr.HasValue();
    if (!hasValue) return;

    TfToken token;
    if (!attr.Get(&token)) {
        *value = CurveSampling::Underlying;
        return;
    }

    if (token == AutolibTokens->underlying) {
        *value = CurveSampling::Underlying;
        return;
    } else if (token == AutolibTokens->resampled) {
        *value = CurveSampling::Resampled;
        return;
    }
}

template <>
void updateIfAuthored(const UsdAttribute& attr, CurveType* value) {
    bool hasValue = attr.HasValue();
    if (!hasValue) return;

    TfToken token;
    if (!attr.Get(&token)) {
        *value = CurveType::None;
        return;
    }

    if (token == AutolibTokens->none) {
        *value = CurveType::None;
        return;
    } else if (token == AutolibTokens->linear) {
        *value = CurveType::Linear;
        return;
    } else if (token == AutolibTokens->catmullRom) {
        *value = CurveType::CatmullRom;
        return;
    }
}

template void updateIfAuthored<float>(const UsdAttribute&, float*);
template void updateIfAuthored<double>(const UsdAttribute&, double*);
template void updateIfAuthored<int>(const UsdAttribute&, int*);