#pragma once

#include "StepUSD/StepUsdPipeline.h"

PXR_NAMESPACE_USING_DIRECTIVE

struct PrototypeMember {

    virtual ~PrototypeMember() = default;

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

struct MeshPrim : public PrototypeMember {

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

struct WireframePrim : public PrototypeMember {

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

struct SketchPrim : public PrototypeMember {

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

struct SketchPlanePrim : public PrototypeMember {

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