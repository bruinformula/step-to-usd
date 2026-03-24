#include <iostream>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/inherits.h>

#include "stepTessellationAPI.h"
#include "stepTessellationDefaults.h"
#include "tokens.h"

using namespace pxr;

int main() {
    TfType t = TfType::FindByName("AutolibStepTessellationAPI");
    if (t == TfType::GetUnknownType()) {
        std::cerr << "Fail: AutolibStepTessellationAPI in TfType registry" << std::endl;
        return 1;
    }

    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    UsdPrim prim = stage->DefinePrim(SdfPath("/test"), TfToken("Mesh"));
    AutolibStepTessellationAPI api = AutolibStepTessellationAPI::Apply(prim);
    if (!api) {
        std::cerr << "Fail: Apply() returned valid schema" << std::endl;
        return 1;
    }

    if (!prim.HasAPI<AutolibStepTessellationAPI>()) {
        std::cerr << "Fail: HasAPI<> true after Apply" << std::endl;
        return 1;
    }

    api.CreateStepTessMeshLinearDeflectionAttr(VtValue(0.25f));
    float val = 0.0f;
    api.GetStepTessMeshLinearDeflectionAttr().Get(&val);
    if (val != 0.25f) {
        std::cerr << "Fail: write/read stepTessMeshLinearDeflection = 0.25" << std::endl;
        return 1;
    }

    api.CreateStepTessMeshAngularDeflectionAttr(VtValue(0.1f));
    api.GetStepTessMeshAngularDeflectionAttr().Get(&val);
    if (val != 0.1f) {
        std::cerr << "Fail: write/read stepTessMeshAngularDeflection = 0.1" << std::endl;
        return 1;
    }

    if (!(AutolibTokens->stepTessMeshLinearDeflection == TfToken("stepTess:meshLinearDeflection"))) {
        std::cerr << "Fail: token stepTessMeshLinearDeflection == 'stepTess:meshLinearDeflection'" << std::endl;
        return 1;
    }

    std::string usda;
    stage->ExportToString(&usda);
    if (usda.find("StepTessellationAPI") == std::string::npos) {
        std::cerr << "Fail: exported layer contains schema name" << std::endl;
        return 1;
    }

    AutolibStepTessellationDefaults defaults = AutolibStepTessellationDefaults::Define(stage, SdfPath("/TessDefaults"));
    if (!defaults) {
        std::cerr << "Fail: StepTessellationDefaults::Define" << std::endl;
        return 1;
    }

    defaults.CreateStepTessMeshLinearDeflectionAttr(VtValue(1.0f));
    defaults.CreateStepTessMeshAngularDeflectionAttr(VtValue(0.5f));
    defaults.CreateStepTessDefaultMeshVisibilityAttr(VtValue(true));
    defaults.CreateStepTessWireframeTypeAttr(VtValue(AutolibTokens->linear));
    defaults.CreateStepTessWireframeSamplingAttr(VtValue(AutolibTokens->underlying));
    defaults.CreateStepTessWireframeDeflectionAttr(VtValue(0.01f));
    defaults.CreateStepTessDefaultWireframeVisibilityAttr(VtValue(true));
    defaults.CreateStepTessSketchTypeAttr(VtValue(AutolibTokens->none));
    defaults.CreateStepTessSketchSamplingAttr(VtValue(AutolibTokens->underlying));
    defaults.CreateStepTessSketchDeflectionAttr(VtValue(0.005f));
    defaults.CreateStepTessDefaultSketchVisibilityAttr(VtValue(false));
    defaults.CreateStepTessRenderPurposeThresholdAttr(VtValue(0.0f));

    UsdPrim childPrim = stage->DefinePrim(SdfPath("/Assembly"), TfToken("Xform"));
    childPrim.GetInherits().AddInherit(SdfPath("/TessDefaults"));
    AutolibStepTessellationAPI childApi = AutolibStepTessellationAPI::Apply(childPrim);

    // Only author the overrides on the child
    childApi.CreateStepTessMeshLinearDeflectionAttr(VtValue(0.25f));
    childApi.CreateStepTessWireframeTypeAttr(VtValue(AutolibTokens->catmullRom));
    childApi.CreateStepTessDefaultSketchVisibilityAttr(VtValue(true));

    // These should resolve through the inherit arc, not be locally authored
    float resolvedAngular = 0.0f;
    childApi.GetStepTessMeshAngularDeflectionAttr().Get(&resolvedAngular);
    if (resolvedAngular != 0.5f) { 
        std::cerr << "Fail: overridden meshLinearDeflection should resolve to 0.25, got " << resolvedAngular << std::endl;
        return 1;
    }

    TfToken resolvedWireframeSampling;
    childApi.GetStepTessWireframeSamplingAttr().Get(&resolvedWireframeSampling);
    if (resolvedWireframeSampling != AutolibTokens->underlying) {
        std::cerr << "Fail: meshAngularDeflection should not be locally authored on child" << std::endl;
        return 1;
    }

    for (const auto& spec : childApi.GetStepTessMeshAngularDeflectionAttr().GetPropertyStack()) {
        if (spec->GetPath().GetPrimPath() == SdfPath("/Assembly")) {
            std::cerr << "Fail: found local spec for meshAngularDeflection on /Assembly" << std::endl;
            return 1;
        }
    }

    std::string finalUsda;
    stage->ExportToString(&finalUsda);

    std::cout << finalUsda << std::endl;

    std::cout << "\nAll tests passed." << std::endl;
    return 0;
}