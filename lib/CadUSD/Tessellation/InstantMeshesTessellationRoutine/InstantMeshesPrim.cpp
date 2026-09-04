#include <pxr/base/vt/value.h>
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
#include <pxr/usd/usdGeom/points.h>

#include <pxr/base/vt/array.h>
#include <pxr/base/vt/types.h>

#include <pxr/base/tf/staticData.h>
#include <pxr/base/tf/token.h>


#pragma pop_macro("Handle")

#include "CadUSD/Logger.h"
#include "CadUSD/Tessellation/TessellationRoutine.h"


PXR_NAMESPACE_USING_DIRECTIVE

bool InstantMeshesTessellationRoutine::definePrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessParams& params
) const {
    if (points.empty()) return true;
    UsdGeomMesh protoMesh = UsdGeomMesh::Define(stage, protoPath.AppendChild(TfToken("InstantMesh")));
    UsdGeomPoints protoPoints = UsdGeomPoints::Define(stage, protoPath.AppendChild(TfToken("Points")));
    return true;
}

bool InstantMeshesTessellationRoutine::writePrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessParams& params
) const {
    if (points.empty()) return true;

    // Just for viewing sample points
    UsdGeomPoints protoPoints(stage->GetPrimAtPath(protoPath.AppendChild(TfToken("Points"))));
    protoPoints.GetPointsAttr().Set(samplePoints);
    protoPoints.GetNormalsAttr().Set(sampleNormals);
    protoPoints.GetWidthsAttr().Set(VtArray{0.05f});
    protoPoints.SetWidthsInterpolation(UsdGeomTokens->constant);
    
    UsdGeomMesh protoMesh(stage->GetPrimAtPath(protoPath.AppendChild(TfToken("InstantMesh"))));

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


    return true;
}

void InstantMeshesTessellationRoutine::clearPrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath
) const {
    stage->RemovePrim(protoPath.AppendChild(TfToken("InstantMesh")));
    stage->RemovePrim(protoPath.AppendChild(TfToken("Points")));
}

size_t InstantMeshesTessellationRoutine::size() const {
    size_t meshSize = points.size();
    return meshSize;
}