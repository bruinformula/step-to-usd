#include <string>
#include <numeric>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/pxr.h>
#include <pxr/usd/usd/common.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>
#include <pxr/usd/usdGeom/subset.h>
#include <pxr/usd/usdGeom/primvar.h>
#include <pxr/base/vt/array.h>
#include <pxr/base/vt/types.h>
#include <pxr/base/tf/staticData.h>
#include <pxr/base/tf/token.h>
#include <pxr/usd/usdGeom/pointBased.h>

#pragma pop_macro("Handle")

#include "StepUSD/StepUsdPipeline.h"
#include "StepUSD/Logger.h"
#include "StepUSD/Routine/PrototypeRoutines.h"

PXR_NAMESPACE_USING_DIRECTIVE

bool MeshRoutine::definePrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessResult& r,
    const TessParams& params
) {
    UsdGeomMesh protoMesh = UsdGeomMesh::Define(stage, protoPath.AppendChild(TfToken("Mesh")));

    if (params.meshEnableSurfaceSubsets) {
        for (const auto& surfaceIDBounds : r.surfaceIDBounds) {
            UsdGeomSubset::Define(
                stage,
                protoMesh.GetPath().AppendChild(TfToken("SurfaceSubset_" + std::to_string(surfaceIDBounds.surfaceID)))
            );
        }
    }
    
    UsdGeomPrimvarsAPI api(protoMesh);
    api.CreatePrimvar(TfToken("st"), SdfValueTypeNames->TexCoord2fArray, UsdGeomTokens->faceVarying);
    api.CreatePrimvar(TfToken("isBoundaryVertex"), SdfValueTypeNames->BoolArray, UsdGeomTokens->vertex);
    api.CreatePrimvar(TfToken("boundaryTangent"), SdfValueTypeNames->Normal3fArray, UsdGeomTokens->vertex);
    return true;
}

bool MeshRoutine::writePrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessResult& r,
    const TessParams& params
) {
    UsdGeomMesh protoMesh(stage->GetPrimAtPath(protoPath.AppendChild(TfToken("Mesh"))));

    if (params.meshEnableSurfaceSubsets) {
        for (const auto& surfaceIDBounds : r.surfaceIDBounds) {
            int count = surfaceIDBounds.endIdx - surfaceIDBounds.startIdx;

            VtIntArray indices(count);
            std::iota(indices.begin(), indices.end(), surfaceIDBounds.startIdx);
            
            UsdGeomSubset subset(stage->GetPrimAtPath(protoMesh.GetPath().AppendChild(TfToken("SurfaceSubset_" + std::to_string(surfaceIDBounds.surfaceID)))));

            if (!subset) {
                LOG_ERR("writeMeshGeometry: missing subset prim SurfaceSubset_" + std::to_string(surfaceIDBounds.surfaceID));
                continue;
            }

            subset.CreateElementTypeAttr().Set(UsdGeomTokens->face);
            subset.CreateIndicesAttr().Set(indices);
            subset.CreateFamilyNameAttr().Set(TfToken("materialBind"));
        }
        
        UsdGeomSubset::SetFamilyType(protoMesh, TfToken("materialBind"), UsdGeomTokens->partition);
    }

    protoMesh.GetPointsAttr().Set(r.points);
    protoMesh.GetFaceVertexCountsAttr().Set(r.faceVertexCounts);
    protoMesh.GetFaceVertexIndicesAttr().Set(r.faceVertexIndices);
    protoMesh.GetSubdivisionSchemeAttr().Set(UsdGeomTokens->none);
    protoMesh.SetNormalsInterpolation(UsdGeomTokens->faceVarying);
    protoMesh.GetNormalsAttr().Set(r.normals);

    {
        VtVec3fArray extent(2);
        if (UsdGeomPointBased::ComputeExtent(r.points, &extent)) {
            protoMesh.CreateExtentAttr().Set(extent);
        } else {
            LOG_ERR("writeMeshGeometry: ComputeExtent failed at " + protoPath.GetString());
        }
    }

    UsdGeomPrimvarsAPI api(protoMesh);
    if (params.meshEnableUVs) if (UsdGeomPrimvar p = api.GetPrimvar(TfToken("st"))) p.Set(r.perSurfaceUVs);
    if (params.meshEnableIsBoundaryVertex) if (UsdGeomPrimvar p = api.GetPrimvar(TfToken("isBoundaryVertex"))) p.Set(r.isBoundaryVertex);
    if (!r.boundaryTangents.empty() && r.boundaryTangents.size() == r.points.size()) {
        if (UsdGeomPrimvar p = api.GetPrimvar(TfToken("boundaryTangent")))
            p.Set(r.boundaryTangents);
    }
    return true;
}