#include "stepTessellationAPI.h"
#include "tokens.h"

#include "UsdStepExporter.h"

PXR_NAMESPACE_USING_DIRECTIVE

template <typename T>
void updateIfAuthored(const UsdAttribute& attr, T* value) {
    if (attr.HasAuthoredValue()) {
        attr.Get(value);
    }
}

// Helper to read a token attr and convert to CurveType

template <>
void updateIfAuthored(const UsdAttribute& attr, CurveSampling* value) {
    if (!attr.HasAuthoredValue()) return;

    TfToken token;
    if (!attr.Get(&token)) {
        *value = CurveSampling::Underlying;
        return;
    }

    if (token == AutolibTokens->underlying) {
        *value = CurveSampling::Underlying;
    } else if (token == AutolibTokens->resampled) {
        *value = CurveSampling::Resampled;
    }
}

template <>
void updateIfAuthored(const UsdAttribute& attr, CurveType* value) {
    if (!attr.HasAuthoredValue()) return;

    TfToken token;
    if (!attr.Get(&token)) {
        *value = CurveType::None;
        return;
    }

    if (token == AutolibTokens->none) {
        *value = CurveType::None;
    } else if (token == AutolibTokens->linear) {
        *value = CurveType::Linear;
    } else if (token == AutolibTokens->catmullRom) {
        *value = CurveType::CatmullRom;
    }
}

static void traverseForTessParams(
    UsdPrim prim, 
    TessParams currentParams,
    std::map<SdfPath, TessParams>& results
) {
    if (prim.HasAPI<AutolibStepTessellationAPI>()) {
        AutolibStepTessellationAPI api(prim);

        // Update settings only if values are authored on this prim
        updateIfAuthored(api.GetStepMeshLinearDeflectionAttr(), &currentParams.meshLinearDeflection);
        updateIfAuthored(api.GetStepMeshAngularDeflectionAttr(), &currentParams.meshAngularDeflection);
        updateIfAuthored(api.GetStepMeshMinSizeAttr(), &currentParams.meshMinSize);

        updateIfAuthored(api.GetStepWireframeDeflectionAttr(), &currentParams.wireframeDeflection);
        updateIfAuthored(api.GetStepWireframeTypeAttr(), &currentParams.wireframeMode.type);
        updateIfAuthored(api.GetStepWireframeSamplingAttr(), &currentParams.wireframeMode.sampling);

        updateIfAuthored(api.GetStepSketchDeflectionAttr(), &currentParams.sketchDeflection);
        updateIfAuthored(api.GetStepSketchTypeAttr(), &currentParams.sketchMode.type);
        updateIfAuthored(api.GetStepSketchSamplingAttr(), &currentParams.sketchMode.sampling);

        updateIfAuthored(api.GetStepRenderPurposeThresholdAttr(), &currentParams.renderPurposeThreshold);
        updateIfAuthored(api.GetStepSelfIntersectionThresholdAttr(), &currentParams.selfIntersectionThreshold);
        updateIfAuthored(api.GetStepMaxNumberRemeshPassesAttr(), &currentParams.maxNumberRemeshPasses);

    }

    results[prim.GetPath()] = currentParams;

    for (const auto& child : prim.GetChildren()) {
        traverseForTessParams(child, currentParams, results);
    }
}

std::map<SdfPath, TessParams> resolveParams(
    const UsdPrim& rootPrim
) {
    TessParams defaultParams = {};

    std::map<SdfPath, TessParams> results;
    traverseForTessParams(rootPrim, defaultParams, results);

    return results;
}