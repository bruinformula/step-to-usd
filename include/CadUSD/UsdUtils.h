#pragma once

#include <filesystem>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/prim.h>

#pragma pop_macro("Handle")

#include "CadUSD/Tessellation/TessellationRoutine.h"

namespace fs = std::filesystem;

PXR_NAMESPACE_USING_DIRECTIVE

UsdStageRefPtr initUsdStage(
    const fs::path& newStagePath, 
    bool clearExisting
);

void addStageSubLayer(
    const UsdStageRefPtr& stage, 
    const fs::path& subLayerPath
);

std::unordered_set<SdfPath, SdfPath::Hash> getVariantsOnPrim(
    const UsdPrim& prim
);

std::string sanitizeUsdName(
    const std::string_view& name
);

std::unordered_set<SdfPath, SdfPath::Hash> reparentPaths(
    const SdfPath& newRoot,
    const std::unordered_set<SdfPath, SdfPath::Hash>& paths
);

template <typename T>
void updateIfAuthored(const UsdAttribute& attr, T* value);

template <>
void updateIfAuthored<TessParams::CurveType>(const UsdAttribute& attr, TessParams::CurveType* value);

template <>
void updateIfAuthored<TessParams::CurveSampling>(const UsdAttribute& attr, TessParams::CurveSampling* value);

extern template void updateIfAuthored<float>(const UsdAttribute&, float*);
extern template void updateIfAuthored<double>(const UsdAttribute&, double*);
extern template void updateIfAuthored<int>(const UsdAttribute&, int*);
extern template void updateIfAuthored<uint64_t>(const UsdAttribute&, uint64_t*);
extern template void updateIfAuthored<bool>(const UsdAttribute&, bool*);