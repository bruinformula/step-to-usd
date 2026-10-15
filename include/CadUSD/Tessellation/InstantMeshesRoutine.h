#pragma once

#include <cstdint>

#include <pxr/pxr.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/sdf/path.h>

#include <pxr/base/gf/vec3f.h>

#include "CadUSD/Tessellation/TessellationRoutineInterface.h"

PXR_NAMESPACE_USING_DIRECTIVE

class TopoDS_Shape;

#ifdef WITH_QUADRIFLOW

namespace CadUSD {

struct InstantMeshesTessellationRoutine : public TessellationRoutineInterface {
    double unitScale = 1;
    
    bool writePartMember(UsdStageRefPtr stage,
                         SdfPath& partPath) const override;

    bool tessellate(const TopoDS_Shape& defShape,
                    const TessParams& params,
                    const SdfPath& protoPath) override;

    bool definePrim(UsdStageRefPtr stage,
                    const SdfPath& protoPath,
                    const TessParams& params) const override;

    bool writePrim(UsdStageRefPtr stage,
                    const SdfPath& protoPath,
                    const TessParams& params) const override;

    void clearPrim(UsdStageRefPtr stage,
                    const SdfPath& protoPath) const override;

    size_t size() const override;

    private:
    VtArray<GfVec3f> points;
    VtArray<GfVec3f> normals;
    VtArray<int> faceVertexCounts;
    VtArray<int> faceVertexIndices;

    VtArray<GfVec3f> samplePoints;
    VtArray<GfVec3f> sampleNormals;
};

} // namespace CadUSD

#endif