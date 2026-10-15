#pragma once

#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/pxr.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/sdf/path.h>

#include <pxr/base/gf/vec2f.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/vt/array.h>

#include <pxr/base/work/loops.h>
#include <pxr/base/work/workTBB/loops_impl.h>

#pragma pop_macro("Handle")

#include "CadUSD/Tessellation/TessellationRoutineInterface.h"
#include "CadUSD/Tessellation/MeshRoutine.h"
#include "CadUSD/Tessellation/SketchRoutine.h"
#include "CadUSD/Tessellation/InstantMeshesRoutine.h"

PXR_NAMESPACE_USING_DIRECTIVE

class Poly_Triangulation;
class Geom_Surface;

namespace occt = opencascade;

namespace CadUSD {

struct GenericParams {
    double renderPurposeThreshold = std::numeric_limits<double>::infinity();

    void populateParams(const AutolibCadTessellationAPI& api);
};

struct TessParams {
    MeshParams mesh;
    WireframeParams wireframe;
    SketchParams sketch;
    SketchPlaneParams sketchPlane;
    GenericParams generic;
    double unitScale;
};

struct TessellationRoutine : public TessellationRoutineInterface {

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
    bool renderOnly = false;

    SketchTessellationRoutine sketchRoutine;
    MeshTessellationRoutine meshRoutine;
#ifdef WITH_QUADRIFLOW
    InstantMeshesTessellationRoutine instRoutine;
#endif
};

} // namespace CadUSD