
#include <iostream>

#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/sdf/path.h>

#include "stepTessellationAPI.h"
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

    api.CreateStepTessLinearDeflectionAttr(VtValue(0.25f));
    float val = 0.f;
    api.GetStepTessLinearDeflectionAttr().Get(&val);

    if (val != 0.25f) {
        std::cerr << "Fail: write/read stepTessLinearDeflection = 0.25" << std::endl;
        return 1;
    }

    api.CreateStepTessAngularDeflectionAttr(VtValue(0.1f));
    api.GetStepTessAngularDeflectionAttr().Get(&val);

    if (val != 0.1f) {
        std::cerr << "Fail: write/read stepTessAngularDeflection = 0.1" << std::endl;
        return 1;
    }

    if (!(AutolibTokens->stepTessLinearDeflection == TfToken("stepTess:linearDeflection"))) {
        std::cerr << "Fail: token stepTessLinearDeflection == 'stepTess:linearDeflection'" << std::endl;
        return 1;
    }

    std::string usda;
    stage->ExportToString(&usda);

    if (usda.find("StepTessellationAPI") == std::string::npos) {
        std::cerr << "Fail: exported layer contains schema name" << std::endl;
        return 1;
    }

    std::cout << "\nAll tests passed." << std::endl;
    return 0;
}