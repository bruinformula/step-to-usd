#include "stepTessellationOptions.h"
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
        updateIfAuthored(api.GetStepTessMeshLinearDeflectionAttr(), &currentParams.meshLinearDeflection);
        updateIfAuthored(api.GetStepTessMeshAngularDeflectionAttr(), &currentParams.meshAngularDeflection);
        updateIfAuthored(api.GetStepTessMeshMinSizeAttr(), &currentParams.meshMinSize);

        updateIfAuthored(api.GetStepTessWireframeDeflectionAttr(), &currentParams.wireframeDeflection);
        updateIfAuthored(api.GetStepTessWireframeTypeAttr(), &currentParams.wireframeMode.type);
        updateIfAuthored(api.GetStepTessWireframeSamplingAttr(), &currentParams.wireframeMode.sampling);

        updateIfAuthored(api.GetStepTessSketchDeflectionAttr(), &currentParams.sketchDeflection);
        updateIfAuthored(api.GetStepTessSketchTypeAttr(), &currentParams.sketchMode.type);
        updateIfAuthored(api.GetStepTessSketchSamplingAttr(), &currentParams.sketchMode.sampling);

        updateIfAuthored(api.GetStepTessRenderPurposeThresholdAttr(), &currentParams.renderPurposeThreshold);
        updateIfAuthored(api.GetStepTessSelfIntersectionThresholdAttr(), &currentParams.selfIntersectionThreshold);
        updateIfAuthored(api.GetStepTessMaxNumberRemeshPassesAttr(), &currentParams.maxNumberRemeshPasses);

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