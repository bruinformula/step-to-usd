#pragma once

#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/pxr.h>
#include <pxr/usd/usd/stage.h>

#include <pxr/base/gf/vec2f.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/vt/array.h>
#include <pxr/base/tf/token.h>
#include <pxr/usd/sdf/path.h>

#include <pxr/base/work/loops.h>
#include <pxr/base/work/workTBB/loops_impl.h>

#pragma pop_macro("Handle")

#include "cadTessellationAPI.h"

PXR_NAMESPACE_USING_DIRECTIVE

class Poly_Triangulation;
class Geom_Surface;

namespace occt = opencascade;

namespace CadUSD {

struct TessParams;

struct TessellationRoutineInterface {
    virtual ~TessellationRoutineInterface() = default;

    virtual bool writePartMember(UsdStageRefPtr stage,
                                 SdfPath& partPath) const = 0;

    virtual bool tessellate(const TopoDS_Shape& defShape,
                            const TessParams& params,
                            const SdfPath& protoPath) = 0;

    virtual bool definePrim(UsdStageRefPtr stage,
                            const SdfPath& protoPath,
                            const TessParams& params) const = 0;

    virtual bool writePrim(UsdStageRefPtr stage,
                           const SdfPath& protoPath,
                           const TessParams& params) const = 0;

    virtual void clearPrim(UsdStageRefPtr stage,
                           const SdfPath& protoPath) const = 0;

    virtual size_t size() const = 0;
};

} // namespace CadUSD