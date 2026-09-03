#include <string>
#include <vector>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/pxr.h>
#include <pxr/usd/usd/common.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usdGeom/scope.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>
#include <pxr/usd/usdGeom/basisCurves.h>
#include <pxr/usd/usdGeom/primvar.h>
#include <pxr/base/vt/array.h>
#include <pxr/base/vt/types.h>
#include <pxr/base/tf/staticData.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/usd/usdGeom/pointBased.h>
#include <stddef.h>
#include <stdint.h>

#pragma pop_macro("Handle")

#include "CadUSD/Logger.h"
#include "CadUSD/Tessellation/TessellationUtils.h"
#include "CadUSD/Tessellation/TessellationRoutine.h"

PXR_NAMESPACE_USING_DIRECTIVE

bool SketchTessellationRoutine::defineSketchPrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessParams& params
) const {
    SdfPath sketchPath = protoPath.AppendChild(TfToken("Sketch"));
    UsdGeomScope::Define(stage, sketchPath);

    if (params.sketchPointLimit <= 0) {
        LOG_ERR("Prim: " + protoPath.GetString() + ": defineSketchGeometry: invalid sketchPointLimit " + std::to_string(params.sketchPointLimit) + ", must be > 0.");
        return false;
    }

    uint64_t numSubCurves = (sketchPoints.size() + params.sketchPointLimit - 1) / params.sketchPointLimit;

    if (params.sketchCombineCurves && numSubCurves == 1) {
        UsdGeomBasisCurves curves = UsdGeomBasisCurves::Define(
            stage, sketchPath.AppendChild(TfToken("Curves"))
        );
        if (!sketchArcValues.empty()) {
            UsdGeomPrimvarsAPI(curves).CreatePrimvar(
                TfToken("uArc"), SdfValueTypeNames->FloatArray, UsdGeomTokens->vertex
            );
        }
    } else {
        std::vector<CurveChunk> chunks = computeCurveChunks(sketchCounts, params.sketchPointLimit, params.sketchCombineCurves);

        for (size_t ci = 0; ci < chunks.size(); ++ci) {
            UsdGeomBasisCurves sketchCurve = UsdGeomBasisCurves::Define(
                stage, sketchPath.AppendChild(TfToken("Curve_" + std::to_string(ci)))
            );

            if (!sketchArcValues.empty()) {
                UsdGeomPrimvarsAPI(sketchCurve).CreatePrimvar(
                    TfToken("uArc"), SdfValueTypeNames->FloatArray, UsdGeomTokens->vertex
                );
            }
        }
    }
    return true;
}

bool SketchTessellationRoutine::writeSketchPrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessParams& params
) const {
    SdfPath sketchPath = protoPath.AppendChild(TfToken("Sketch"));

    if (params.sketchPointLimit <= 0) {
        LOG_ERR("Prim: " + protoPath.GetString() + ": defineSketchGeometry: invalid sketchPointLimit " + std::to_string(params.sketchPointLimit) + ", must be > 0.");
        return false;
    }

    uint64_t numSubCurves = (sketchPoints.size() + params.sketchPointLimit - 1) / params.sketchPointLimit;

    if (params.sketchCombineCurves && numSubCurves == 1) {
        UsdGeomBasisCurves sketchCurve(stage->GetPrimAtPath(sketchPath.AppendChild(TfToken("Curves"))));

        if (!sketchCurve) {
            LOG_ERR("writeSketchGeometry: missing curve prim");
            return false;
        }

        if (params.sketchMode.type == TessParams::CurveType::Cubic) {
            sketchCurve.CreateTypeAttr().Set(UsdGeomTokens->cubic);
            sketchCurve.CreateBasisAttr().Set(UsdGeomTokens->catmullRom);
        } else {
            sketchCurve.CreateTypeAttr().Set(UsdGeomTokens->linear);
        }
        sketchCurve.CreateWrapAttr().Set(UsdGeomTokens->nonperiodic);
        sketchCurve.GetPointsAttr().Set(sketchPoints);
        sketchCurve.GetCurveVertexCountsAttr().Set(sketchCounts);

        {
            VtVec3fArray extent(2);
            if (UsdGeomPointBased::ComputeExtent(sketchPoints, &extent)) {
                sketchCurve.CreateExtentAttr().Set(extent);
            } else {
                LOG_ERR("writeSketchGeometry: ComputeExtent failed at " + protoPath.GetString());
            }
        }

        if (sketchArcValues.size() == sketchPoints.size()) {
            UsdGeomPrimvarsAPI(sketchCurve).CreatePrimvar(TfToken("uArc"), SdfValueTypeNames->FloatArray, UsdGeomTokens->vertex);
        }
        
        if (sketchArcValues.size() == sketchPoints.size()) {
            UsdGeomPrimvarsAPI api(sketchCurve);
            if (UsdGeomPrimvar p = api.GetPrimvar(TfToken("uArc")))
                p.Set(sketchArcValues);
        }

        sketchCurve.CreateWidthsAttr().Set(VtArray<float>{0.005f});
        UsdGeomPrimvarsAPI(sketchCurve).CreatePrimvar(TfToken("widths"), SdfValueTypeNames->FloatArray, UsdGeomTokens->constant).Set(VtArray<float>{0.005f});
        
        sketchCurve.GetDisplayColorAttr().Set(VtArray<GfVec3f>{{0.4f, 0.7f, 1.0f}});
    } else {
        std::vector<CurveChunk> chunks = computeCurveChunks(sketchCounts, params.sketchPointLimit, params.sketchCombineCurves);

        for (size_t ci = 0; ci < chunks.size(); ++ci) {
            const CurveChunk& chunk = chunks[ci];

            UsdGeomBasisCurves sketchCurve(stage->GetPrimAtPath(sketchPath.AppendChild(TfToken("Curve_" + std::to_string(ci)))));
            if (!sketchCurve) {
                LOG_ERR("writeSketchGeometry: missing curve prim Curve_" + std::to_string(ci));
                continue;
            }

            if (params.sketchMode.type == TessParams::CurveType::Cubic) {
                sketchCurve.CreateTypeAttr().Set(UsdGeomTokens->cubic);
                sketchCurve.CreateBasisAttr().Set(UsdGeomTokens->catmullRom);
            } else {
                sketchCurve.CreateTypeAttr().Set(UsdGeomTokens->linear);
            }
            sketchCurve.CreateWrapAttr().Set(UsdGeomTokens->nonperiodic);

            VtArray<GfVec3f> pts = gatherChunkValues(sketchPoints, chunk);
            sketchCurve.GetPointsAttr().Set(pts);
            sketchCurve.GetCurveVertexCountsAttr().Set(chunkVertexCounts(chunk));
            sketchCurve.GetDisplayColorAttr().Set(VtArray<GfVec3f>{{0.4f, 0.7f, 1.0f}});

            {
                VtVec3fArray extent(2);
                if (UsdGeomPointBased::ComputeExtent(pts, &extent)) {
                    sketchCurve.CreateExtentAttr().Set(extent);
                } else {
                    LOG_ERR("writeSketchGeometry: ComputeExtent failed at " + protoPath.GetString());
                }
            }

            if (!sketchArcValues.empty()) {
                VtArray<float> arcUs = gatherChunkValues(sketchArcValues, chunk);
                UsdGeomPrimvarsAPI api(sketchCurve);
                if (UsdGeomPrimvar p = api.GetPrimvar(TfToken("uArc")))
                    p.Set(arcUs);
            }
        }
    }

    return true;
}

void SketchTessellationRoutine::clearPrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath
) const {
    stage->RemovePrim(protoPath.AppendChild(TfToken("Sketch")));
    stage->RemovePrim(protoPath.AppendChild(TfToken("SketchPlane")));
}

size_t SketchTessellationRoutine::size() const {
    size_t sketchSize = sketchPoints.size();
    size_t sketchPlaneSize = sketchPlanePoints.size();
    return sketchSize + sketchPlaneSize;
}