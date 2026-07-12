#include <ostream>
#include <iostream>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>

#include <TDF_Label.hxx>
#include <Quantity_Color.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_XYZ.hxx>
#include <TDF_Tool.hxx>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/pxr.h>
#include <pxr/usd/usd/common.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/modelAPI.h>
#include <pxr/usd/usd/inherits.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/references.h>
#include <pxr/usd/usd/variantSets.h>
#include <pxr/usd/usd/editContext.h>

#include <pxr/usd/sdf/changeBlock.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/variantSpec.h>
#include <pxr/usd/sdf/variantSetSpec.h>

#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdGeom/scope.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>
#include <pxr/usd/usdGeom/basisCurves.h>
#include <pxr/usd/usdGeom/imageable.h>
#include <pxr/usd/usdGeom/subset.h>
#include <pxr/usd/usdGeom/primvar.h>
#include <pxr/usd/usdGeom/xformOp.h>

#include <pxr/usd/sdf/copyUtils.h>
#include <pxr/base/work/loops.h>

#include <pxr/base/vt/array.h>
#include <pxr/base/vt/types.h>
#include <pxr/base/tf/staticData.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/gf/vec3f.h>

#pragma pop_macro("Handle")

#include "StepUSD/StepUsdPipeline.h"
#include "StepUSD/Logger.h"
#include "StepUSD/Routine/PrototypeRoutines.h"
#include "StepUSD/Routine/CurveUtils.h"

PXR_NAMESPACE_USING_DIRECTIVE

bool SketchRoutine::definePrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessResult& r,
    const TessParams& params
) {
    SdfPath sketchPath = protoPath.AppendChild(TfToken("Sketch"));
    UsdGeomScope::Define(stage, sketchPath);

    if (params.sketchPointLimit <= 0) {
        LOG_ERR("Prim: " + protoPath.GetString() + ": defineSketchGeometry: invalid sketchPointLimit " + std::to_string(params.sketchPointLimit) + ", must be > 0.");
        return false;
    }

    uint64_t numSubCurves = (r.sketchPoints.size() + params.sketchPointLimit - 1) / params.sketchPointLimit;

    if (params.sketchCombineCurves && numSubCurves == 1) {
        UsdGeomBasisCurves curves = UsdGeomBasisCurves::Define(
            stage, sketchPath.AppendChild(TfToken("Curves"))
        );
        if (!r.sketchArcValues.empty()) {
            UsdGeomPrimvarsAPI(curves).CreatePrimvar(
                TfToken("uArc"), SdfValueTypeNames->FloatArray, UsdGeomTokens->vertex
            );
        }
    } else {
        std::vector<CurveChunk> chunks = computeCurveChunks(r.sketchCounts, params.sketchPointLimit, params.sketchCombineCurves);

        for (size_t ci = 0; ci < chunks.size(); ++ci) {
            UsdGeomBasisCurves sketchCurve = UsdGeomBasisCurves::Define(
                stage, sketchPath.AppendChild(TfToken("Curve_" + std::to_string(ci)))
            );

            if (!r.sketchArcValues.empty()) {
                UsdGeomPrimvarsAPI(sketchCurve).CreatePrimvar(
                    TfToken("uArc"), SdfValueTypeNames->FloatArray, UsdGeomTokens->vertex
                );
            }
        }
    }
    return true;
}

bool SketchRoutine::writePrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessResult& r,
    const TessParams& params
) {
    SdfPath sketchPath = protoPath.AppendChild(TfToken("Sketch"));

    if (params.sketchPointLimit <= 0) {
        LOG_ERR("Prim: " + protoPath.GetString() + ": defineSketchGeometry: invalid sketchPointLimit " + std::to_string(params.sketchPointLimit) + ", must be > 0.");
        return false;
    }

    uint64_t numSubCurves = (r.sketchPoints.size() + params.sketchPointLimit - 1) / params.sketchPointLimit;

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
        sketchCurve.GetPointsAttr().Set(r.sketchPoints);
        sketchCurve.GetCurveVertexCountsAttr().Set(r.sketchCounts);

        {
            VtVec3fArray extent(2);
            if (UsdGeomPointBased::ComputeExtent(r.sketchPoints, &extent)) {
                sketchCurve.CreateExtentAttr().Set(extent);
            } else {
                LOG_ERR("writeSketchGeometry: ComputeExtent failed at " + protoPath.GetString());
            }
        }

        if (r.sketchArcValues.size() == r.sketchPoints.size()) {
            UsdGeomPrimvarsAPI(sketchCurve).CreatePrimvar(TfToken("uArc"), SdfValueTypeNames->FloatArray, UsdGeomTokens->vertex);
        }
        
        if (r.sketchArcValues.size() == r.sketchPoints.size()) {
            UsdGeomPrimvarsAPI api(sketchCurve);
            if (UsdGeomPrimvar p = api.GetPrimvar(TfToken("uArc")))
                p.Set(r.sketchArcValues);
        }

        //sketchCurve.CreateWidthsAttr().Set(VtArray<float>{0.005f});
        //UsdGeomPrimvarsAPI(sketchCurve).CreatePrimvar(TfToken("widths"), SdfValueTypeNames->FloatArray, UsdGeomTokens->constant).Set(VtArray<float>{0.005f});
        
        sketchCurve.GetDisplayColorAttr().Set(VtArray<GfVec3f>{{0.4f, 0.7f, 1.0f}});
    } else {
        std::vector<CurveChunk> chunks = computeCurveChunks(r.sketchCounts, params.sketchPointLimit, params.sketchCombineCurves);

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

            VtArray<GfVec3f> pts = gatherChunkValues(r.sketchPoints, chunk);
            sketchCurve.GetPointsAttr().Set(pts);
            sketchCurve.GetCurveVertexCountsAttr().Set(chunkVertexCounts(chunk));
            sketchCurve.GetDisplayColorAttr().Set(VtArray<GfVec3f>{{0.4f, 0.7f, 1.0f}});

            {
                VtVec3fArray extent(2);
                if (UsdGeomPointBased::ComputeExtent(r.points, &extent)) {
                    sketchCurve.CreateExtentAttr().Set(extent);
                } else {
                    LOG_ERR("writeMeshGeometry: ComputeExtent failed at " + protoPath.GetString());
                }
            }

            if (!r.sketchArcValues.empty()) {
                VtArray<float> arcUs = gatherChunkValues(r.sketchArcValues, chunk);
                UsdGeomPrimvarsAPI api(sketchCurve);
                if (UsdGeomPrimvar p = api.GetPrimvar(TfToken("uArc")))
                    p.Set(arcUs);
            }
        }
    }

    return true;
}