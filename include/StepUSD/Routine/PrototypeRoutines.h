#pragma once

#include "StepUSD/StepUsdPipeline.h"

PXR_NAMESPACE_USING_DIRECTIVE

struct PrototypeRoutine {

    virtual ~PrototypeRoutine() = default;

    virtual bool definePrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        const TessResult& r,
        const TessParams& params
    ) = 0;

    virtual bool writePrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        const TessResult& r,
        const TessParams& params
    ) = 0;
};

struct MeshRoutine : public PrototypeRoutine {

    bool definePrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        const TessResult& r,
        const TessParams& params
    ) override;
    bool writePrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        const TessResult& r,
        const TessParams& params
    ) override;
};

struct WireframeRoutine : public PrototypeRoutine {

    bool definePrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        const TessResult& r,
        const TessParams& params
    ) override;
    bool writePrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        const TessResult& r,
        const TessParams& params
    ) override;
};

struct SketchRoutine : public PrototypeRoutine {

    bool definePrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        const TessResult& r,
        const TessParams& params
    ) override;
    bool writePrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        const TessResult& r,
        const TessParams& params
    ) override;
};

struct SketchPlaneRoutine : public PrototypeRoutine {

    bool definePrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        const TessResult& r,
        const TessParams& params
    ) override;
    bool writePrim(
        UsdStageRefPtr stage,
        const SdfPath& protoPath,
        const TessResult& r,
        const TessParams& params
    ) override;
};