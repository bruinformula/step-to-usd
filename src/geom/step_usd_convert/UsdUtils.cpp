
#include <cctype>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/tokens.h>

#include <pxr/usd/usd/variantSets.h>

#pragma pop_macro("Handle")

#include "tokens.h"

#include "UsdStepExporter.h"
#include "Logger.h"

PXR_NAMESPACE_USING_DIRECTIVE

UsdStageRefPtr initUsdStage(
    const fs::path& newStagePath, 
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

    layer->SetField(SdfPath::AbsoluteRootPath(), UsdGeomTokens->upAxis, VtValue(UsdGeomTokens->z));

    // Open the stage. 
    UsdStageRefPtr stage = UsdStage::Open(layer);

    stage->Save();
    return stage;
}

void addStageSubLayer(
    const UsdStageRefPtr& stage, 
    const fs::path& subLayerPath
) {
    SdfLayerHandle rootLayer = stage->GetRootLayer();

    bool alreadyExists = false;
    for (const auto& path : rootLayer->GetSubLayerPaths()) {
        if (path == subLayerPath.string() || path == subLayerPath) { 
            alreadyExists = true; 
            break; 
        }
    }

    if (!alreadyExists) {
        rootLayer->InsertSubLayerPath(subLayerPath.string());
        LOG_DEBUG("Added sublayer: " + subLayerPath.string());
    } else {
        LOG_DEBUG("Sublayer already exists, skipping: " + subLayerPath.string());
    }
}

std::unordered_set<SdfPath, SdfPath::Hash> getVariantsOnPrim(
    const UsdPrim& prim
) {
    std::vector<std::string> variantSetNames;
    prim.GetVariantSets().GetNames(&variantSetNames);

    std::unordered_set<SdfPath, SdfPath::Hash> variantPaths;

    for (const std::string& vsetName : variantSetNames) {
        UsdVariantSet vset = prim.GetVariantSet(vsetName);
        std::vector<std::string> variantNames = vset.GetVariantNames();
        for (const std::string& variant : variantNames) {
            SdfPath variantPath = prim.GetPath().AppendVariantSelection(vsetName, variant);
            variantPaths.insert(variantPath);
        }
    }
    return variantPaths;
}

std::string sanitizeUsdName(const std::string_view& name, int idx) {
    if (name.empty()) return "Node_" + std::to_string(idx);

    std::string result;
    result.reserve(name.size());

    for (char c : name) {
        if (std::isalnum(c) || c == '_') result += c;
        else result += '_'; // replace hyphens, spaces, dots
    }

    // Usd prim names must start with a letter or underscore
    if (!result.empty() && std::isdigit(result[0]))
        result = "_" + result;

    if (result.empty()) 
        return "Node_" + std::to_string(idx);

    if (idx != 0)
        result += "_" + std::to_string(idx);

    return result;
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
void updateIfAuthored(const UsdAttribute& attr, TessParams::CurveSampling* value) {
    bool hasValue = attr.HasValue();
    if (!hasValue) return;

    TfToken token;
    if (!attr.Get(&token)) {
        *value = TessParams::CurveSampling::Underlying;
        return;
    }

    if (token == AutolibTokens->underlying) {
        *value = TessParams::CurveSampling::Underlying;
        return;
    } else if (token == AutolibTokens->resampled) {
        *value = TessParams::CurveSampling::Resampled;
        return;
    }
}

template <>
void updateIfAuthored(const UsdAttribute& attr, TessParams::CurveType* value) {
    bool hasValue = attr.HasValue();
    if (!hasValue) return;

    TfToken token;
    if (!attr.Get(&token)) {
        *value = TessParams::CurveType::None;
        return;
    }

    if (token == AutolibTokens->none) {
        *value = TessParams::CurveType::None;
        return;
    } else if (token == AutolibTokens->linear) {
        *value = TessParams::CurveType::Linear;
        return;
    } else if (token == AutolibTokens->cubic) {
        *value = TessParams::CurveType::Cubic;
        return;
    }
}

template void updateIfAuthored<float>(const UsdAttribute&, float*);
template void updateIfAuthored<double>(const UsdAttribute&, double*);
template void updateIfAuthored<int>(const UsdAttribute&, int*);
template void updateIfAuthored<uint64_t>(const UsdAttribute&, uint64_t*);
template void updateIfAuthored<bool>(const UsdAttribute&, bool*);