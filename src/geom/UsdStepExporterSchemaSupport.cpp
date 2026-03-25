#include "stepTessellationOptions.h"
#include "tokens.h"

#include "UsdStepExporter.h"

PXR_NAMESPACE_USING_DIRECTIVE

// Helper to read a token attr and convert to CurveType
static CurveType tokenToCurveType(const UsdAttribute& attr, CurveType fallback) {
    TfToken token;
    if (!attr || !attr.Get(&token)) return fallback;
    if (token == AutolibTokens->none) return CurveType::None;
    if (token == AutolibTokens->linear) return CurveType::Linear;
    if (token == AutolibTokens->catmullRom) return CurveType::CatmullRom;
    return fallback;
}

static CurveSampling tokenToCurveSampling(const UsdAttribute& attr, CurveSampling fallback) {
    TfToken token;
    if (!attr || !attr.Get(&token)) return fallback;
    if (token == AutolibTokens->underlying) return CurveSampling::Underlying;
    if (token == AutolibTokens->resampled)  return CurveSampling::Resampled;
    return fallback;
}

TessParams getResolvedTessParams(const UsdPrim& prim) {
    TessParams params;
    AutolibStepTessellationOptions options(prim);

    options.GetStepTessMeshLinearDeflectionAttr().Get(&params.meshLinearDeflection);
    options.GetStepTessMeshAngularDeflectionAttr().Get(&params.meshAngularDeflection);
    options.GetStepTessMeshMinSizeAttr().Get(&params.meshMinSize);
    options.GetStepTessWireframeDeflectionAttr().Get(&params.wireframeDeflection);
    options.GetStepTessSketchDeflectionAttr().Get(&params.sketchDeflection);
    options.GetStepTessRenderPurposeThresholdAttr().Get(&params.renderPurposeThreshold);
    options.GetStepTessSelfIntersectionThresholdAttr().Get(&params.selfIntersectionThreshold);
    options.GetStepTessMaxNumberRemeshPassesAttr().Get(&params.maxNumberRemeshPasses);

    // Tokens must read as TfToken then convert to enum
    params.wireframeMode.type = tokenToCurveType    (options.GetStepTessWireframeTypeAttr(),     params.wireframeMode.type);
    params.wireframeMode.sampling = tokenToCurveSampling(options.GetStepTessWireframeSamplingAttr(), params.wireframeMode.sampling);
    params.sketchMode.type = tokenToCurveType    (options.GetStepTessSketchTypeAttr(),         params.sketchMode.type);
    params.sketchMode.sampling = tokenToCurveSampling(options.GetStepTessSketchSamplingAttr(),     params.sketchMode.sampling);

    return params;
}