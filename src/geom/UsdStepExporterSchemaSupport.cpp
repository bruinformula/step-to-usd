#include "stepTessellationAPI.h"
#include "tokens.h"

#include "UsdStepExporter.h"
#include <iostream>

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
    std::map<SdfPath, TessParams>& result
) {
    AutolibStepTessellationAPI api(prim);
    bool hasAnyValue = false;

    // Apply relationship-bound options first
    // should this even be a relationship? 
    UsdRelationship optionsRel = api.GetStepTessellationOptionsRel();
    SdfPathVector targets;
    if (optionsRel && optionsRel.GetForwardedTargets(&targets) && !targets.empty()) {
        UsdPrim optionsPrim = prim.GetStage()->GetPrimAtPath(targets[0]);
        if (optionsPrim.IsValid()) {
            AutolibStepTessellationAPI optionsApi(optionsPrim);
            updateIfAuthored(optionsApi.GetStepMeshLinearDeflectionAttr(), &currentParams.meshLinearDeflection, hasAnyValue);
            updateIfAuthored(optionsApi.GetStepMeshAngularDeflectionAttr(), &currentParams.meshAngularDeflection, hasAnyValue);
            updateIfAuthored(optionsApi.GetStepMeshMinSizeAttr(), &currentParams.meshMinSize, hasAnyValue);
            updateIfAuthored(optionsApi.GetStepWireframeDeflectionAttr(), &currentParams.wireframeDeflection, hasAnyValue);
            updateIfAuthored(optionsApi.GetStepWireframeTypeAttr(), &currentParams.wireframeMode.type, hasAnyValue);
            updateIfAuthored(optionsApi.GetStepWireframeSamplingAttr(), &currentParams.wireframeMode.sampling, hasAnyValue);
            updateIfAuthored(optionsApi.GetStepSketchDeflectionAttr(), &currentParams.sketchDeflection, hasAnyValue);
            updateIfAuthored(optionsApi.GetStepSketchTypeAttr(), &currentParams.sketchMode.type, hasAnyValue);
            updateIfAuthored(optionsApi.GetStepSketchSamplingAttr(), &currentParams.sketchMode.sampling, hasAnyValue);
            updateIfAuthored(optionsApi.GetStepRenderPurposeThresholdAttr(), &currentParams.renderPurposeThreshold, hasAnyValue);
            updateIfAuthored(optionsApi.GetStepSelfIntersectionThresholdAttr(), &currentParams.selfIntersectionThreshold, hasAnyValue);
            updateIfAuthored(optionsApi.GetStepMaxNumberRemeshPassesAttr(), &currentParams.maxNumberRemeshPasses, hasAnyValue);
        }
    }

    // Local authored attrs always win over the rel binding
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

    result[prim.GetPath()] = currentParams;

    for (const auto& child : prim.GetChildren())
        traverseForTessParams(child, currentParams, result);
}

std::map<SdfPath, TessParams> resolveParams(
    const UsdPrim& rootPrim
) {
    TessParams defaultParams = {};

    std::map<SdfPath, TessParams> results;
    traverseForTessParams(rootPrim, defaultParams, results);

    return results;
}