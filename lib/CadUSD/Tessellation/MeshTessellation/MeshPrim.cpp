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

#include "CadUSD/Logger.h"
#include "CadUSD/Tessellation/TessellationRoutine.h"


PXR_NAMESPACE_USING_DIRECTIVE

bool MeshTessellationRoutine::defineMeshPrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessParams& params
) const {
    UsdGeomMesh protoMesh = UsdGeomMesh::Define(stage, protoPath.AppendChild(TfToken("Mesh")));

    if (params.meshEnableSurfaceSubsets) {
        for (const auto& surfaceIDBounds : surfaceIDBounds) {
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

bool MeshTessellationRoutine::writeMeshPrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessParams& params
) const {
    UsdGeomMesh protoMesh(stage->GetPrimAtPath(protoPath.AppendChild(TfToken("Mesh"))));

    if (params.meshEnableSurfaceSubsets) {
        for (const auto& surfaceIDBounds : surfaceIDBounds) {
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

    protoMesh.GetPointsAttr().Set(points);
    protoMesh.GetFaceVertexCountsAttr().Set(faceVertexCounts);
    protoMesh.GetFaceVertexIndicesAttr().Set(faceVertexIndices);
    protoMesh.GetSubdivisionSchemeAttr().Set(UsdGeomTokens->none);
    protoMesh.SetNormalsInterpolation(UsdGeomTokens->faceVarying);
    protoMesh.GetNormalsAttr().Set(normals);

    {
        VtVec3fArray extent(2);
        if (UsdGeomPointBased::ComputeExtent(points, &extent)) {
            protoMesh.CreateExtentAttr().Set(extent);
        } else {
            LOG_ERR("writeMeshGeometry: ComputeExtent failed at " + protoPath.GetString());
        }
    }

    UsdGeomPrimvarsAPI api(protoMesh);
    if (params.meshEnableUVs) if (UsdGeomPrimvar p = api.GetPrimvar(TfToken("st"))) p.Set(perSurfaceUVs);
    if (params.meshEnableIsBoundaryVertex) if (UsdGeomPrimvar p = api.GetPrimvar(TfToken("isBoundaryVertex"))) p.Set(isBoundaryVertex);
    if (!boundaryTangents.empty() && boundaryTangents.size() == points.size()) {
        if (UsdGeomPrimvar p = api.GetPrimvar(TfToken("boundaryTangent")))
            p.Set(boundaryTangents);
    }
    return true;
}