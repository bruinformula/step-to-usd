
#include <stddef.h>

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
#include <pxr/usd/usdGeom/pointBased.h>

#include <pxr/base/vt/array.h>
#include <pxr/base/vt/types.h>
#include <pxr/base/tf/staticData.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/gf/vec3f.h>


#pragma pop_macro("Handle")

#include "CadUSD/Logger.h"

#include "CadUSD/Tessellation/TessellationUtils.h"
#include "CadUSD/Tessellation/TessellationRoutine.h"

PXR_NAMESPACE_USING_DIRECTIVE

bool MeshTessellationRoutine::defineWireframePrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessParams& params
) const {
    SdfPath wireframePath = protoPath.AppendChild(TfToken("Wireframe"));
    UsdGeomXform::Define(stage, wireframePath);

    if (params.wireframePointLimit <= 0) {
        LOG_ERR("Prim: " + protoPath.GetString() + ": MeshTessellationRoutine::defineWireframePrim: invalid wireframePointLimit " + std::to_string(params.wireframePointLimit) + ", must be > 0.");
        return false;
    }

    uint64_t numSubCurves = (wireframePoints.size() + params.wireframePointLimit - 1) / params.wireframePointLimit;

    if (params.wireframeCombineCurves && numSubCurves == 1) {
        UsdGeomBasisCurves curve = UsdGeomBasisCurves::Define(
            stage, wireframePath.AppendChild(TfToken("Curves"))
        );
        if (!wireframeContinuity.empty()) {
            UsdGeomPrimvarsAPI(curve).CreatePrimvar(TfToken("continuityType"), SdfValueTypeNames->IntArray, UsdGeomTokens->uniform);
        }
        if (params.wireframeEmbedSurfaceNormals && wireframeSurfaceNormals.size() == wireframePoints.size()) {
            UsdGeomPrimvarsAPI(curve).CreatePrimvar(TfToken("surfaceNormal"), SdfValueTypeNames->Normal3fArray, UsdGeomTokens->vertex);
        }

        if (params.wireframeEmbedSurfaceNormals && wireframeSurfaceNormals.size() == wireframePoints.size()) {
            LOG_DEBUG("MeshTessellationRoutine::defineWireframePrim: Created surfaceNormal primvar for wireframe");
        }
        if (wireframeArcValues.size() == wireframePoints.size()) {
            UsdGeomPrimvarsAPI(curve).CreatePrimvar(TfToken("uArc"), SdfValueTypeNames->FloatArray, UsdGeomTokens->vertex);
        }
    } else {
        std::vector<CurveChunk> chunks = computeCurveChunks(wireframeCounts, params.wireframePointLimit, params.wireframeCombineCurves);

        for (size_t ci = 0; ci < chunks.size(); ++ci) {
            UsdGeomBasisCurves curve = UsdGeomBasisCurves::Define(
                stage, wireframePath.AppendChild(TfToken("Wireframe_" + std::to_string(ci)))
            );

            if (!wireframeContinuity.empty()) {
                UsdGeomPrimvarsAPI(curve).CreatePrimvar(TfToken("continuityType"), SdfValueTypeNames->IntArray, UsdGeomTokens->uniform);
            }
            if (params.wireframeEmbedSurfaceNormals && wireframeSurfaceNormals.size() == wireframePoints.size()) {
                UsdGeomPrimvarsAPI(curve).CreatePrimvar(TfToken("surfaceNormal"), SdfValueTypeNames->Normal3fArray, UsdGeomTokens->vertex);
            }
            if (!wireframeArcValues.empty()) {
                UsdGeomPrimvarsAPI(curve).CreatePrimvar(TfToken("uArc"), SdfValueTypeNames->FloatArray, UsdGeomTokens->vertex);
            }
        }
    }
    return true;
}

bool MeshTessellationRoutine::writeWireframePrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessParams& params
) const {
    SdfPath wireframePath = protoPath.AppendChild(TfToken("Wireframe"));

    if (params.wireframePointLimit <= 0) {
        LOG_ERR("Prim: " + protoPath.GetString() + ": MeshTessellationRoutine::writeWireframePrim: invalid wireframePointLimit " + std::to_string(params.wireframePointLimit) + ", must be > 0.");
        return false;
    }
    
    uint64_t numSubCurves = (wireframePoints.size() + params.wireframePointLimit - 1) / params.wireframePointLimit;

    if (params.wireframeCombineCurves && numSubCurves == 1) {
        UsdGeomBasisCurves curve(stage->GetPrimAtPath(wireframePath.AppendChild(TfToken("Curves"))));

        if (!curve) {
            LOG_ERR("Prim: " + protoPath.GetString() + ": MeshTessellationRoutine::writeWireframePrim: missing curve prim");
            return false;
        }

        if (params.wireframeMode.type == TessParams::CurveType::Cubic) {
            curve.CreateTypeAttr().Set(UsdGeomTokens->cubic);
            curve.CreateBasisAttr().Set(UsdGeomTokens->catmullRom);
        } else {
            curve.CreateTypeAttr().Set(UsdGeomTokens->linear);
        }
        curve.CreateWrapAttr().Set(UsdGeomTokens->nonperiodic);
        curve.GetPointsAttr().Set(wireframePoints);
        curve.GetCurveVertexCountsAttr().Set(wireframeCounts);

        {
            VtVec3fArray extent(2);
            if (UsdGeomPointBased::ComputeExtent(wireframePoints, &extent)) {
                curve.CreateExtentAttr().Set(extent);
            } else {
                LOG_ERR("Prim: " + protoPath.GetString() + ": MeshTessellationRoutine::writeWireframePrim: ComputeExtent failed at " + protoPath.GetString());
            }
        }
        
        curve.CreateWidthsAttr().Set(VtArray<float>{0.005f});
        UsdGeomPrimvarsAPI(curve).CreatePrimvar(TfToken("widths"), SdfValueTypeNames->FloatArray, UsdGeomTokens->constant).Set(VtArray<float>{0.005f});
        
        curve.GetDisplayColorAttr().Set(VtArray<GfVec3f>{{0.8f, 0.8f, 0.8f}});

        if (!wireframeContinuity.empty()) {
            UsdGeomPrimvarsAPI api(curve);
            if (UsdGeomPrimvar p = api.GetPrimvar(TfToken("continuityType"))) p.Set(wireframeContinuity);
        }

        if (params.wireframeEmbedSurfaceNormals && wireframeSurfaceNormals.size() == wireframePoints.size() && !points.empty()) {
            UsdGeomPrimvarsAPI api(curve);
            if (UsdGeomPrimvar p = api.GetPrimvar(TfToken("surfaceNormal")))
                p.Set(wireframeSurfaceNormals);
        }

        if (wireframeArcValues.size() == wireframePoints.size()) {
            UsdGeomPrimvarsAPI api(curve);
            if (UsdGeomPrimvar p = api.GetPrimvar(TfToken("uArc"))) 
                p.Set(wireframeArcValues);
        }
    } else {
        std::vector<CurveChunk> chunks = computeCurveChunks(wireframeCounts, params.wireframePointLimit, params.wireframeCombineCurves);

        for (size_t ci = 0; ci < chunks.size(); ++ci) {
            const CurveChunk& chunk = chunks[ci];

            UsdGeomBasisCurves curve(stage->GetPrimAtPath(wireframePath.AppendChild(TfToken("Wireframe_" + std::to_string(ci)))));
            if (!curve) {
                LOG_ERR("Prim: " + protoPath.GetString() + ": MeshTessellationRoutine::writeWireframePrim: missing curve prim Wireframe_" + std::to_string(ci));
                continue;
            }

            if (params.wireframeMode.type == TessParams::CurveType::Cubic) {
                curve.CreateTypeAttr().Set(UsdGeomTokens->cubic);
                curve.CreateBasisAttr().Set(UsdGeomTokens->catmullRom);
            } else {
                curve.CreateTypeAttr().Set(UsdGeomTokens->linear);
            }
            curve.CreateWrapAttr().Set(UsdGeomTokens->nonperiodic);

            VtArray<GfVec3f> pts = gatherChunkValues(wireframePoints, chunk);
            curve.GetPointsAttr().Set(pts);
            curve.GetCurveVertexCountsAttr().Set(chunkVertexCounts(chunk));
            curve.GetDisplayColorAttr().Set(VtArray<GfVec3f>{{0.8f, 0.8f, 0.8f}});

            {
                VtVec3fArray extent(2);
                if (UsdGeomPointBased::ComputeExtent(points, &extent)) {
                    curve.CreateExtentAttr().Set(extent);
                } else {
                    LOG_ERR("Prim: " + protoPath.GetString() + ": MeshTessellationRoutine::writeWireframePrim: ComputeExtent failed at " + protoPath.GetString());
                }
            }

            UsdGeomPrimvarsAPI api(curve);

            if (!wireframeContinuity.empty()) {
                VtIntArray continuity(chunk.pieces.size());
                for (size_t j = 0; j < chunk.pieces.size(); ++j) {
                    int srcIdx = chunk.pieces[j].sourceCurveIdx;
                    continuity[j] = (srcIdx < (int)wireframeContinuity.size()) ? wireframeContinuity[srcIdx] : 0;
                }
                if (UsdGeomPrimvar p = api.GetPrimvar(TfToken("continuityType")))
                    p.Set(continuity);
            }

            if (!wireframeArcValues.empty()) {
                VtArray<float> arcUs = gatherChunkValues(wireframeArcValues, chunk);
                if (UsdGeomPrimvar p = api.GetPrimvar(TfToken("uArc")))
                    p.Set(arcUs);
            }

            if (params.wireframeEmbedSurfaceNormals && !wireframeSurfaceNormals.empty()) {
                VtArray<GfVec3f> normals = gatherChunkValues(wireframeSurfaceNormals, chunk);
                if (UsdGeomPrimvar p = api.GetPrimvar(TfToken("surfaceNormal")))
                    p.Set(normals);
            }
        }
    }
    return true;
}