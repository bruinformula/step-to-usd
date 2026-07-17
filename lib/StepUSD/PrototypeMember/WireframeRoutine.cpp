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
#include <pxr/usd/usdGeom/xform.h>
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

#include "StepUSD/StepUsdPipeline.h"
#include "StepUSD/Logger.h"
#include "StepUSD/PrototypeMember/PrototypeMember.h"
#include "StepUSD/Tessellation/TessellationUtils.h"

PXR_NAMESPACE_USING_DIRECTIVE

bool WireframePrim::definePrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessResult& r,
    const TessParams& params
) {
    SdfPath wireframePath = protoPath.AppendChild(TfToken("Wireframe"));
    UsdGeomXform::Define(stage, wireframePath);

    if (params.wireframePointLimit <= 0) {
        LOG_ERR("Prim: " + protoPath.GetString() + ": defineWireframeGeometry: invalid wireframePointLimit " + std::to_string(params.wireframePointLimit) + ", must be > 0.");
        return false;
    }

    uint64_t numSubCurves = (r.wireframePoints.size() + params.wireframePointLimit - 1) / params.wireframePointLimit;

    if (params.wireframeCombineCurves && numSubCurves == 1) {
        UsdGeomBasisCurves curve = UsdGeomBasisCurves::Define(
            stage, wireframePath.AppendChild(TfToken("Curves"))
        );
        if (!r.wireframeContinuity.empty()) {
            UsdGeomPrimvarsAPI(curve).CreatePrimvar(TfToken("continuityType"), SdfValueTypeNames->IntArray, UsdGeomTokens->uniform);
        }
        if (params.wireframeEmbedSurfaceNormals && r.wireframeSurfaceNormals.size() == r.wireframePoints.size()) {
            UsdGeomPrimvarsAPI(curve).CreatePrimvar(TfToken("surfaceNormal"), SdfValueTypeNames->Normal3fArray, UsdGeomTokens->vertex);
        }

        if (params.wireframeEmbedSurfaceNormals && r.wireframeSurfaceNormals.size() == r.wireframePoints.size()) {
            LOG_DEBUG("defineWireframeGeometry: Created surfaceNormal primvar for wireframe");
        }
        if (r.wireframeArcValues.size() == r.wireframePoints.size()) {
            UsdGeomPrimvarsAPI(curve).CreatePrimvar(TfToken("uArc"), SdfValueTypeNames->FloatArray, UsdGeomTokens->vertex);
        }
    } else {
        std::vector<CurveChunk> chunks = computeCurveChunks(r.wireframeCounts, params.wireframePointLimit, params.wireframeCombineCurves);

        for (size_t ci = 0; ci < chunks.size(); ++ci) {
            UsdGeomBasisCurves curve = UsdGeomBasisCurves::Define(
                stage, wireframePath.AppendChild(TfToken("Wireframe_" + std::to_string(ci)))
            );

            if (!r.wireframeContinuity.empty()) {
                UsdGeomPrimvarsAPI(curve).CreatePrimvar(TfToken("continuityType"), SdfValueTypeNames->IntArray, UsdGeomTokens->uniform);
            }
            if (params.wireframeEmbedSurfaceNormals && r.wireframeSurfaceNormals.size() == r.wireframePoints.size()) {
                UsdGeomPrimvarsAPI(curve).CreatePrimvar(TfToken("surfaceNormal"), SdfValueTypeNames->Normal3fArray, UsdGeomTokens->vertex);
            }
            if (!r.wireframeArcValues.empty()) {
                UsdGeomPrimvarsAPI(curve).CreatePrimvar(TfToken("uArc"), SdfValueTypeNames->FloatArray, UsdGeomTokens->vertex);
            }
        }
    }
    return true;
}

bool WireframePrim::writePrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessResult& r,
    const TessParams& params
) {
    SdfPath wireframePath = protoPath.AppendChild(TfToken("Wireframe"));

    if (params.wireframePointLimit <= 0) {
        LOG_ERR("Prim: " + protoPath.GetString() + ": WireframePrim::writePrim: invalid wireframePointLimit " + std::to_string(params.wireframePointLimit) + ", must be > 0.");
        return false;
    }
    
    uint64_t numSubCurves = (r.wireframePoints.size() + params.wireframePointLimit - 1) / params.wireframePointLimit;

    if (params.wireframeCombineCurves && numSubCurves == 1) {
        UsdGeomBasisCurves curve(stage->GetPrimAtPath(wireframePath.AppendChild(TfToken("Curves"))));

        if (!curve) {
            LOG_ERR("Prim: " + protoPath.GetString() + ": WireframePrim::writePrim: missing curve prim");
            return false;
        }

        if (params.wireframeMode.type == TessParams::CurveType::Cubic) {
            curve.CreateTypeAttr().Set(UsdGeomTokens->cubic);
            curve.CreateBasisAttr().Set(UsdGeomTokens->catmullRom);
        } else {
            curve.CreateTypeAttr().Set(UsdGeomTokens->linear);
        }
        curve.CreateWrapAttr().Set(UsdGeomTokens->nonperiodic);
        curve.GetPointsAttr().Set(r.wireframePoints);
        curve.GetCurveVertexCountsAttr().Set(r.wireframeCounts);

        {
            VtVec3fArray extent(2);
            if (UsdGeomPointBased::ComputeExtent(r.wireframePoints, &extent)) {
                curve.CreateExtentAttr().Set(extent);
            } else {
                LOG_ERR("Prim: " + protoPath.GetString() + ": WireframePrim::writePrim: ComputeExtent failed at " + protoPath.GetString());
            }
        }
        
        //curve.CreateWidthsAttr().Set(VtArray<float>{0.005f});
        //UsdGeomPrimvarsAPI(curve).CreatePrimvar(TfToken("widths"), SdfValueTypeNames->FloatArray, UsdGeomTokens->constant).Set(VtArray<float>{0.005f});
        
        curve.GetDisplayColorAttr().Set(VtArray<GfVec3f>{{0.8f, 0.8f, 0.8f}});

        if (!r.wireframeContinuity.empty()) {
            UsdGeomPrimvarsAPI api(curve);
            if (UsdGeomPrimvar p = api.GetPrimvar(TfToken("continuityType"))) p.Set(r.wireframeContinuity);
        }

        if (params.wireframeEmbedSurfaceNormals && r.wireframeSurfaceNormals.size() == r.wireframePoints.size() && !r.points.empty()) {
            UsdGeomPrimvarsAPI api(curve);
            if (UsdGeomPrimvar p = api.GetPrimvar(TfToken("surfaceNormal")))
                p.Set(r.wireframeSurfaceNormals);
        }

        if (r.wireframeArcValues.size() == r.wireframePoints.size()) {
            UsdGeomPrimvarsAPI api(curve);
            if (UsdGeomPrimvar p = api.GetPrimvar(TfToken("uArc"))) 
                p.Set(r.wireframeArcValues);
        }
    } else {
        std::vector<CurveChunk> chunks = computeCurveChunks(r.wireframeCounts, params.wireframePointLimit, params.wireframeCombineCurves);

        for (size_t ci = 0; ci < chunks.size(); ++ci) {
            const CurveChunk& chunk = chunks[ci];

            UsdGeomBasisCurves curve(stage->GetPrimAtPath(wireframePath.AppendChild(TfToken("Wireframe_" + std::to_string(ci)))));
            if (!curve) {
                LOG_ERR("Prim: " + protoPath.GetString() + ": WireframePrim::writePrim: missing curve prim Wireframe_" + std::to_string(ci));
                continue;
            }

            if (params.wireframeMode.type == TessParams::CurveType::Cubic) {
                curve.CreateTypeAttr().Set(UsdGeomTokens->cubic);
                curve.CreateBasisAttr().Set(UsdGeomTokens->catmullRom);
            } else {
                curve.CreateTypeAttr().Set(UsdGeomTokens->linear);
            }
            curve.CreateWrapAttr().Set(UsdGeomTokens->nonperiodic);

            VtArray<GfVec3f> pts = gatherChunkValues(r.wireframePoints, chunk);
            curve.GetPointsAttr().Set(pts);
            curve.GetCurveVertexCountsAttr().Set(chunkVertexCounts(chunk));
            curve.GetDisplayColorAttr().Set(VtArray<GfVec3f>{{0.8f, 0.8f, 0.8f}});

            {
                VtVec3fArray extent(2);
                if (UsdGeomPointBased::ComputeExtent(r.points, &extent)) {
                    curve.CreateExtentAttr().Set(extent);
                } else {
                    LOG_ERR("Prim: " + protoPath.GetString() + ": WireframePrim::writePrim: ComputeExtent failed at " + protoPath.GetString());
                }
            }

            UsdGeomPrimvarsAPI api(curve);

            if (!r.wireframeContinuity.empty()) {
                VtIntArray continuity(chunk.pieces.size());
                for (size_t j = 0; j < chunk.pieces.size(); ++j) {
                    int srcIdx = chunk.pieces[j].sourceCurveIdx;
                    continuity[j] = (srcIdx < (int)r.wireframeContinuity.size()) ? r.wireframeContinuity[srcIdx] : 0;
                }
                if (UsdGeomPrimvar p = api.GetPrimvar(TfToken("continuityType")))
                    p.Set(continuity);
            }

            if (!r.wireframeArcValues.empty()) {
                VtArray<float> arcUs = gatherChunkValues(r.wireframeArcValues, chunk);
                if (UsdGeomPrimvar p = api.GetPrimvar(TfToken("uArc")))
                    p.Set(arcUs);
            }

            if (params.wireframeEmbedSurfaceNormals && !r.wireframeSurfaceNormals.empty()) {
                VtArray<GfVec3f> normals = gatherChunkValues(r.wireframeSurfaceNormals, chunk);
                if (UsdGeomPrimvar p = api.GetPrimvar(TfToken("surfaceNormal")))
                    p.Set(normals);
            }
        }
    }
    return true;
}