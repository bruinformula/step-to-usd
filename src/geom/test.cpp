#include <iostream>
#include <string>

#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/inherits.h>
#include <pxr/base/tf/staticData.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/tf/type.h>
#include <pxr/base/vt/value.h>
#include <pxr/pxr.h>
#include <pxr/usd/sdf/declareHandles.h>
#include <pxr/usd/sdf/propertySpec.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/common.h>

#include "stepTessellationOptions.h"
#include "tokens.h"

#include "UsdStepExporterSchemaSupport.h"

PXR_NAMESPACE_USING_DIRECTIVE

int main() {
    UsdStageRefPtr stage = UsdStage::CreateInMemory();

    AutolibStepTessellationOptions defaultParams = AutolibStepTessellationOptions::Define(stage, SdfPath("/DefaultParams"));
    defaultParams.CreateStepMeshLinearDeflectionAttr(VtValue(1.0f));
    defaultParams.CreateStepMeshAngularDeflectionAttr(VtValue(0.5f));
    defaultParams.CreateStepWireframeTypeAttr(VtValue(AutolibTokens->linear));
    defaultParams.CreateStepWireframeSamplingAttr(VtValue(AutolibTokens->underlying));
    defaultParams.CreateStepWireframeDeflectionAttr(VtValue(0.01f));
    defaultParams.CreateStepSketchTypeAttr(VtValue(AutolibTokens->none));
    defaultParams.CreateStepSketchSamplingAttr(VtValue(AutolibTokens->underlying));
    defaultParams.CreateStepSketchDeflectionAttr(VtValue(0.005f));
    defaultParams.CreateStepRenderPurposeThresholdAttr(VtValue(0.0f));

    AutolibStepTessellationOptions wheelParams = AutolibStepTessellationOptions::Define(stage, SdfPath("/WheelParams"));
    wheelParams.GetPrim().GetInherits().AddInherit(SdfPath("/DefaultParams"));
    wheelParams.CreateStepMeshLinearDeflectionAttr(VtValue(0.3f));

    UsdPrim car = stage->DefinePrim(SdfPath("/Car"), TfToken("Xform"));
    car.GetInherits().AddInherit(SdfPath("/DefaultParams"));

    UsdPrim wheelR = stage->DefinePrim(SdfPath("/Car/WheelR"));
    wheelR.GetInherits().AddInherit(SdfPath("/WheelParams"));

    UsdPrim wheelL = stage->DefinePrim(SdfPath("/Car/WheelL"));
    wheelL.GetInherits().AddInherit(SdfPath("/WheelParams"));
    AutolibStepTessellationOptions wheelLOptions(wheelL);
    wheelLOptions.CreateStepMeshLinearDeflectionAttr(VtValue(100.0f));

    UsdPrim wheelM = stage->DefinePrim(SdfPath("/Car/WheelM"));


    int failures = 0;

    auto CHECK = [&](bool condition, const std::string& msg) {
        if (!condition) {
            std::cerr << "FAIL: " << msg << std::endl;
            ++failures;
        }
    };

    auto CHECK_FLOAT = [&](float got, float expected, const std::string& msg) {
        if (got != expected) {
            std::cerr << "FAIL: " << msg
                    << " expected=" << expected << " got=" << got << std::endl;
            ++failures;
        }
    };

    auto CHECK_TOKEN = [&](TfToken got, TfToken expected, const std::string& msg) {
        if (got != expected) {
            std::cerr << "FAIL: " << msg
                    << " expected=" << expected << " got=" << got << std::endl;
            ++failures;
        }
    };

    { // /DefaultParams values should be exactly what was authored 
        float v = 0;
        defaultParams.GetStepMeshLinearDeflectionAttr().Get(&v);
        CHECK_FLOAT(v, 1.0f,  "/DefaultParams meshLinearDeflection");

        defaultParams.GetStepMeshAngularDeflectionAttr().Get(&v);
        CHECK_FLOAT(v, 0.5f,  "/DefaultParams meshAngularDeflection");

        defaultParams.GetStepWireframeDeflectionAttr().Get(&v);
        CHECK_FLOAT(v, 0.01f, "/DefaultParams wireframeDeflection");

        defaultParams.GetStepSketchDeflectionAttr().Get(&v);
        CHECK_FLOAT(v, 0.005f,"/DefaultParams sketchDeflection");

        defaultParams.GetStepRenderPurposeThresholdAttr().Get(&v);
        CHECK_FLOAT(v, 0.0f,  "/DefaultParams renderPurposeThreshold");

        TfToken tok;
        defaultParams.GetStepWireframeTypeAttr().Get(&tok);
        CHECK_TOKEN(tok, AutolibTokens->linear,     "/DefaultParams wireframeType");

        defaultParams.GetStepWireframeSamplingAttr().Get(&tok);
        CHECK_TOKEN(tok, AutolibTokens->underlying, "/DefaultParams wireframeSampling");

        defaultParams.GetStepSketchTypeAttr().Get(&tok);
        CHECK_TOKEN(tok, AutolibTokens->none,       "/DefaultParams sketchType");

        defaultParams.GetStepSketchSamplingAttr().Get(&tok);
        CHECK_TOKEN(tok, AutolibTokens->underlying, "/DefaultParams sketchSampling");
    }

    { // /WheelParams authored one override, rest resolved via inherit
        AutolibStepTessellationOptions wp(stage->GetPrimAtPath(SdfPath("/WheelParams")));
        float v = 0;

        // Authored override
        wp.GetStepMeshLinearDeflectionAttr().Get(&v);
        CHECK_FLOAT(v, 0.3f,  "/WheelParams meshLinearDeflection (authored override)");

        // Everything below should come from /DefaultParams via inherit
        wp.GetStepMeshAngularDeflectionAttr().Get(&v);
        CHECK_FLOAT(v, 0.5f,  "/WheelParams meshAngularDeflection (inherited)");

        wp.GetStepWireframeDeflectionAttr().Get(&v);
        CHECK_FLOAT(v, 0.01f, "/WheelParams wireframeDeflection (inherited)");

        wp.GetStepSketchDeflectionAttr().Get(&v);
        CHECK_FLOAT(v, 0.005f,"/WheelParams sketchDeflection (inherited)");

        TfToken tok;
        wp.GetStepWireframeTypeAttr().Get(&tok);
        CHECK_TOKEN(tok, AutolibTokens->linear,     "/WheelParams wireframeType (inherited)");

        wp.GetStepSketchTypeAttr().Get(&tok);
        CHECK_TOKEN(tok, AutolibTokens->none,       "/WheelParams sketchType (inherited)");

        // Confirm no local spec was authored for the inherited attrs
        for (const auto& spec : wp.GetStepMeshAngularDeflectionAttr().GetPropertyStack()) {
            CHECK(spec->GetPath().GetPrimPath() != SdfPath("/WheelParams"),
                "/WheelParams must not have a local spec for meshAngularDeflection");
        }
    }

    { // /Car inherits DefaultParams with no local overrides
        AutolibStepTessellationOptions carOpts(car);
        float v = 0;

        carOpts.GetStepMeshLinearDeflectionAttr().Get(&v);
        CHECK_FLOAT(v, 1.0f,  "/Car meshLinearDeflection (inherited from DefaultParams)");

        carOpts.GetStepMeshAngularDeflectionAttr().Get(&v);
        CHECK_FLOAT(v, 0.5f,  "/Car meshAngularDeflection (inherited from DefaultParams)");

        TfToken tok;
        carOpts.GetStepWireframeTypeAttr().Get(&tok);
        CHECK_TOKEN(tok, AutolibTokens->linear, "/Car wireframeType (inherited from DefaultParams)");
    }

    { // /Car/WheelR has no local overrides and resolves entirely via WheelParams
        AutolibStepTessellationOptions wrOpts(wheelR);
        float v = 0;

        // Comes from WheelParams authored override
        wrOpts.GetStepMeshLinearDeflectionAttr().Get(&v);
        CHECK_FLOAT(v, 0.3f,  "/Car/WheelR meshLinearDeflection (from WheelParams)");

        // Comes from DefaultParams via WheelParams inherit
        wrOpts.GetStepMeshAngularDeflectionAttr().Get(&v);
        CHECK_FLOAT(v, 0.5f,  "/Car/WheelR meshAngularDeflection (from DefaultParams via WheelParams)");

        wrOpts.GetStepWireframeDeflectionAttr().Get(&v);
        CHECK_FLOAT(v, 0.01f, "/Car/WheelR wireframeDeflection (from DefaultParams via WheelParams)");

        wrOpts.GetStepSketchDeflectionAttr().Get(&v);
        CHECK_FLOAT(v, 0.005f,"/Car/WheelR sketchDeflection (from DefaultParams via WheelParams)");

        TfToken tok;
        wrOpts.GetStepWireframeTypeAttr().Get(&tok);
        CHECK_TOKEN(tok, AutolibTokens->linear, "/Car/WheelR wireframeType (from DefaultParams via WheelParams)");

        wrOpts.GetStepSketchTypeAttr().Get(&tok);
        CHECK_TOKEN(tok, AutolibTokens->none,   "/Car/WheelR sketchType (from DefaultParams via WheelParams)");

        // No local specs anywhere on /Car/WheelR itself
        for (const auto& spec : wrOpts.GetStepMeshLinearDeflectionAttr().GetPropertyStack()) {
            CHECK(spec->GetPath().GetPrimPath() != SdfPath("/Car/WheelR"),
                "/Car/WheelR must not have any local specs");
        }
    }

    { // /Car/WheelL one local override, rest resolves via WheelParams 
        float v = 0;

        // Local override wins over WheelParams
        wheelLOptions.GetStepMeshLinearDeflectionAttr().Get(&v);
        CHECK_FLOAT(v, 100.0f, "/Car/WheelL meshLinearDeflection (local override)");

        // No local spec for angular — should fall through WheelParams → DefaultParams
        wheelLOptions.GetStepMeshAngularDeflectionAttr().Get(&v);
        CHECK_FLOAT(v, 0.5f,   "/Car/WheelL meshAngularDeflection (from DefaultParams via WheelParams)");

        wheelLOptions.GetStepWireframeDeflectionAttr().Get(&v);
        CHECK_FLOAT(v, 0.01f,  "/Car/WheelL wireframeDeflection (from DefaultParams via WheelParams)");

        TfToken tok;
        wheelLOptions.GetStepWireframeTypeAttr().Get(&tok);
        CHECK_TOKEN(tok, AutolibTokens->linear, "/Car/WheelL wireframeType (from DefaultParams via WheelParams)");

        // Confirm the local override is the strongest opinion
        bool foundLocalSpec = false;
        for (const auto& spec : wheelLOptions.GetStepMeshLinearDeflectionAttr().GetPropertyStack()) {
            if (spec->GetPath().GetPrimPath() == SdfPath("/Car/WheelL")) {
                foundLocalSpec = true;
                break;
            }
        }
        CHECK(foundLocalSpec, "/Car/WheelL must have a local spec for meshLinearDeflection");

        // Confirm NO local spec for angular on WheelL
        for (const auto& spec : wheelLOptions.GetStepMeshAngularDeflectionAttr().GetPropertyStack()) {
            CHECK(spec->GetPath().GetPrimPath() != SdfPath("/Car/WheelL"),
                "/Car/WheelL must not have a local spec for meshAngularDeflection");
        }
    }

    // Final result
    if (failures == 0) {
        std::cout << "All tests passed." << std::endl;
    } else {
        std::cerr << failures << " test(s) failed." << std::endl;
        return 1;
    }

    std::map<SdfPath, TessParams> params = resolveParams(car);
    /*
    std::string finalUsda;
    stage->ExportToString(&finalUsda);
    std::cout << finalUsda << std::endl;
    */

    for (const auto& p : params) {
        std::cout << "Prim: " << p.first << std::endl;
        const TessParams& tp = p.second;
        std::cout << "  meshLinearDeflection: " << tp.meshLinearDeflection << std::endl;
        std::cout << "  meshAngularDeflection: " << tp.meshAngularDeflection << std::endl;
        std::cout << "  wireframeDeflection: " << tp.wireframeDeflection << std::endl;
        std::cout << "  wireframeType: " << tp.wireframeMode.type << std::endl;
        std::cout << "  wireframeSampling: " << tp.wireframeMode.sampling << std::endl;
        std::cout << "  sketchDeflection: " << tp.sketchDeflection << std::endl;
        std::cout << "  sketchType: " << tp.sketchMode.type << std::endl;
        std::cout << "  sketchSampling: " << tp.sketchMode.sampling << std::endl;
        std::cout << "  renderPurposeThreshold: " << tp.renderPurposeThreshold << std::endl;
        std::cout << "  selfIntersectionThreshold: " << tp.selfIntersectionThreshold << std::endl;
        std::cout << "  maxNumberRemeshPasses: " << tp.maxNumberRemeshPasses << std::endl;
    }

    std::string sparseUsda;
    stage->GetRootLayer()->ExportToString(&sparseUsda);
    std::cout << sparseUsda << std::endl;
    
}