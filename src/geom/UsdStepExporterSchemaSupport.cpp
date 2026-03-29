#include <pxr/usd/usd/inherits.h>

#include "stepTessellationAPI.h"
#include "tokens.h"

#include "UsdStepExporter.h"

PXR_NAMESPACE_USING_DIRECTIVE

template <typename T>
void updateIfAuthored(const UsdAttribute& attr, T* value, bool& primHasAnyValue) {
    bool hasValue = attr.HasValue();
    primHasAnyValue |= hasValue;
    if (hasValue) {
        attr.Get(value);
    }
}

// Helper to read a token attr and convert to CurveType
template <>
void updateIfAuthored(const UsdAttribute& attr, CurveSampling* value, bool& primHasAnyValue) {
    bool hasValue = attr.HasValue();
    primHasAnyValue |= hasValue;
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
void updateIfAuthored(const UsdAttribute& attr, CurveType* value, bool& primHasAnyValue) {
    bool hasValue = attr.HasValue();
    primHasAnyValue |= hasValue;
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

static void traverseForTessParams(
    UsdPrim prim, 
    TessParams currentParams,
    std::map<SdfPath, TessParams>& partNodes
) {
    AutolibStepTessellationAPI api(prim);

    bool hasAnyValue = false;
    updateIfAuthored(api.GetStepMeshLinearDeflectionAttr(), &currentParams.meshLinearDeflection, hasAnyValue);
    updateIfAuthored(api.GetStepMeshAngularDeflectionAttr(), &currentParams.meshAngularDeflection, hasAnyValue);
    updateIfAuthored(api.GetStepMeshMinSizeAttr(), &currentParams.meshMinSize, hasAnyValue);

    updateIfAuthored(api.GetStepWireframeDeflectionAttr(), &currentParams.wireframeDeflection, hasAnyValue);
    updateIfAuthored(api.GetStepWireframeTypeAttr(), &currentParams.wireframeMode.type, hasAnyValue);
    updateIfAuthored(api.GetStepWireframeSamplingAttr(), &currentParams.wireframeMode.sampling, hasAnyValue);

    updateIfAuthored(api.GetStepSketchDeflectionAttr(), &currentParams.sketchDeflection, hasAnyValue);
    updateIfAuthored(api.GetStepSketchTypeAttr(), &currentParams.sketchMode.type, hasAnyValue);
    updateIfAuthored(api.GetStepSketchSamplingAttr(), &currentParams.sketchMode.sampling, hasAnyValue);

    updateIfAuthored(api.GetStepRenderPurposeThresholdAttr(), &currentParams.renderPurposeThreshold, hasAnyValue);
    updateIfAuthored(api.GetStepSelfIntersectionThresholdAttr(), &currentParams.selfIntersectionThreshold, hasAnyValue);
    updateIfAuthored(api.GetStepMaxNumberRemeshPassesAttr(), &currentParams.maxNumberRemeshPasses, hasAnyValue);

    partNodes[prim.GetPath()] = currentParams;

    // Avoid parts that inherit from /CADPart
    for (const SdfPath& inheritPath : prim.GetInherits().GetAllDirectInherits()) {
        if (inheritPath == SdfPath("/CADPart")) {
            return;
        }
    }

    for (const auto& child : prim.GetChildren()) {
        traverseForTessParams(child, currentParams, partNodes);
    }
}

std::map<SdfPath, TessParams> resolveParams(
    const UsdPrim& rootPrim,
    const TessParams& defaultParams
) {
    std::map<SdfPath, TessParams> results;
    traverseForTessParams(rootPrim, defaultParams, results);

    return results;
}

TessParams getTessParams(
    UsdPrim prim
) {
    AutolibStepTessellationAPI api(prim);

    TessParams params;

    bool hasAnyValue = false;
    updateIfAuthored(api.GetStepMeshLinearDeflectionAttr(), &params.meshLinearDeflection, hasAnyValue);
    updateIfAuthored(api.GetStepMeshAngularDeflectionAttr(), &params.meshAngularDeflection, hasAnyValue);
    updateIfAuthored(api.GetStepMeshMinSizeAttr(), &params.meshMinSize, hasAnyValue);

    updateIfAuthored(api.GetStepWireframeDeflectionAttr(), &params.wireframeDeflection, hasAnyValue);
    updateIfAuthored(api.GetStepWireframeTypeAttr(), &params.wireframeMode.type, hasAnyValue);
    updateIfAuthored(api.GetStepWireframeSamplingAttr(), &params.wireframeMode.sampling, hasAnyValue);

    updateIfAuthored(api.GetStepSketchDeflectionAttr(), &params.sketchDeflection, hasAnyValue);
    updateIfAuthored(api.GetStepSketchTypeAttr(), &params.sketchMode.type, hasAnyValue);
    updateIfAuthored(api.GetStepSketchSamplingAttr(), &params.sketchMode.sampling, hasAnyValue);

    updateIfAuthored(api.GetStepRenderPurposeThresholdAttr(), &params.renderPurposeThreshold, hasAnyValue);
    updateIfAuthored(api.GetStepSelfIntersectionThresholdAttr(), &params.selfIntersectionThreshold, hasAnyValue);
    updateIfAuthored(api.GetStepMaxNumberRemeshPassesAttr(), &params.maxNumberRemeshPasses, hasAnyValue);

    return params;
}
