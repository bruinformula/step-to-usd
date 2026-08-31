#include <utility>
#include <unordered_map>
#include <string>
#include <vector>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/pxr.h>
#include <pxr/usd/usd/common.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/scope.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/base/vt/array.h>
#include <pxr/base/tf/staticData.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/gf/vec3f.h>
#include <stddef.h>

#pragma pop_macro("Handle")

#include "CadUSD/Tessellation/TessellationRoutine.h"

PXR_NAMESPACE_USING_DIRECTIVE

bool SketchTessellationRoutine::defineSketchPlanePrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessParams& params
) const {
    SdfPath sketchPlanesPath = protoPath.AppendChild(TfToken("SketchPlanes"));
    UsdGeomScope::Define(stage, sketchPlanesPath);
    for (size_t pi = 0; pi < sketchPlaneBounds.size(); ++pi) {
        UsdGeomMesh::Define(stage, sketchPlanesPath.AppendChild(TfToken("Plane_" + std::to_string(pi))));
    }
    return true;
}

bool SketchTessellationRoutine::writeSketchPlanePrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessParams& params
) const {
    SdfPath sketchPlanesPath = protoPath.AppendChild(TfToken("SketchPlanes"));

    for (size_t pi = 0; pi < sketchPlaneBounds.size(); ++pi) {
        const SketchPlaneBounds& b = sketchPlaneBounds[pi];

        if (b.pointStart < 0 || b.pointCount <= 0 ||
            b.faceCountStart < 0 || b.faceCountCount <= 0 ||
            b.faceIndexStart < 0 || b.faceIndexCount <= 0 ||
            b.normalStart < 0 || b.normalCount <= 0) {
            continue;
        }

        if (static_cast<size_t>(b.pointStart + b.pointCount) > sketchPlanePoints.size() ||
            static_cast<size_t>(b.faceCountStart + b.faceCountCount) > sketchPlaneFaceVertexCounts.size() ||
            static_cast<size_t>(b.faceIndexStart + b.faceIndexCount) > sketchPlaneFaceVertexIndices.size() ||
            static_cast<size_t>(b.normalStart + b.normalCount) > sketchPlaneNormals.size()) {
            continue;
        }

        // Build a compact point array and remap indices in one pass.
        // The stored indices are absolute into sketchPlanePoints; we need
        // them rebased to a local [0, N) range for this plane's mesh prim.
        std::unordered_map<int, int> globalToLocal;
        VtArray<GfVec3f> points;
        VtArray<GfVec3f> normals;

        VtArray<int> counts(
            sketchPlaneFaceVertexCounts.begin() + b.faceCountStart,
            sketchPlaneFaceVertexCounts.begin() + b.faceCountStart + b.faceCountCount
        );

        VtArray<int> indices;
        indices.reserve(b.faceIndexCount);

        bool corruptPlane = false;
        for (int fi = b.faceIndexStart; fi < b.faceIndexStart + b.faceIndexCount; ++fi) {
            int globalIdx = sketchPlaneFaceVertexIndices[fi];

            auto [it, inserted] = globalToLocal.emplace(globalIdx, (int)points.size());
            if (inserted) {
                if (static_cast<size_t>(globalIdx) >= sketchPlanePoints.size()) {
                    // Corrupt index — skip entire plane
                    corruptPlane = true;
                    continue;
                }
                points.push_back(sketchPlanePoints[globalIdx]);
            }
            indices.push_back(it->second);
        }

        if (corruptPlane) continue; 

        // Normals are faceVarying (one per face-vertex), slice directly
        normals = VtArray<GfVec3f>(
            sketchPlaneNormals.begin() + b.normalStart,
            sketchPlaneNormals.begin() + b.normalStart + b.normalCount
        );

        {
            UsdGeomMesh mesh(stage->GetPrimAtPath(
                sketchPlanesPath.AppendChild(TfToken("Plane_" + std::to_string(pi)))
            ));
            mesh.GetPointsAttr().Set(points);
            mesh.GetFaceVertexCountsAttr().Set(counts);
            mesh.GetFaceVertexIndicesAttr().Set(indices);
            mesh.GetSubdivisionSchemeAttr().Set(UsdGeomTokens->none);
            mesh.SetNormalsInterpolation(UsdGeomTokens->faceVarying);
            mesh.GetNormalsAttr().Set(normals);
            mesh.GetDisplayColorAttr().Set(VtArray<GfVec3f>{{0.55f, 0.8f, 1.0f}});
        }
    }

    return true;
}