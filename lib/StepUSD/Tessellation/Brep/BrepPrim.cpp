
#include <sstream>
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

#include <pxr/usd/usdSolid/brepArray.h>
#include <pxr/usd/usdSolid/brepCurve3dCircleAPI.h>
#include <pxr/usd/usdSolid/brepCurve3dEllipseAPI.h>
#include <pxr/usd/usdSolid/brepCurve3dLineAPI.h>
#include <pxr/usd/usdSolid/brepCurve3dNurbAPI.h>
#include <pxr/usd/usdSolid/brepCurveUvNurbAPI.h>
#include <pxr/usd/usdSolid/brepPointAPI.h>
#include <pxr/usd/usdSolid/brepSurfaceConeAPI.h>
#include <pxr/usd/usdSolid/brepSurfaceCylinderAPI.h>
#include <pxr/usd/usdSolid/brepSurfaceNurbAPI.h>
#include <pxr/usd/usdSolid/brepSurfacePlaneAPI.h>
#include <pxr/usd/usdSolid/brepSurfaceSphereAPI.h>
#include <pxr/usd/usdSolid/brepSurfaceTorusAPI.h>
#include <pxr/usd/usdSolid/tokens.h>

#pragma pop_macro("Handle")

#include "StepUSD/Logger.h"

#include "StepUSD/Tessellation/TessellationUtils.h"
#include "StepUSD/Tessellation/TessellationRoutine.h"

PXR_NAMESPACE_USING_DIRECTIVE

static bool HasFlag(Kind value, Kind flag) {
    return (value & flag) != Kind::None;
};

static void unionizeSurfaceKind(Kind& value, Kind flag) {
    value = static_cast<Kind>(static_cast<int>(value) | static_cast<int>(flag));
}

static void addApis(const AnalyticSurface& surf, UsdPrim prim) {

    UsdSolidBrepPointAPI::Apply(prim, TfToken("vertexPoint"));
    UsdSolidBrepCurve3dNurbAPI::Apply(prim, TfToken("edge3dNurb"));
    UsdSolidBrepCurveUvNurbAPI::Apply(prim);

    if (HasFlag(surf.kind, Kind::Nurb))
        UsdSolidBrepSurfaceNurbAPI::Apply(prim);

    if (HasFlag(surf.kind, Kind::Plane))
        UsdSolidBrepSurfacePlaneAPI::Apply(prim);

    if (HasFlag(surf.kind, Kind::Cylinder))
        UsdSolidBrepSurfaceCylinderAPI::Apply(prim);

    if (HasFlag(surf.kind, Kind::Cone))
        UsdSolidBrepSurfaceConeAPI::Apply(prim);

    if (HasFlag(surf.kind, Kind::Sphere))
        UsdSolidBrepSurfaceSphereAPI::Apply(prim);

    if (HasFlag(surf.kind, Kind::Torus))
        UsdSolidBrepSurfaceTorusAPI::Apply(prim);
}

static void addFaceSurfaceTypes(const AnalyticSurface& surf, VtArray<TfToken>& faceSurfaceType) {

    if (HasFlag(surf.kind, Kind::Nurb))
        faceSurfaceType.push_back(TfToken("BrepSurfaceNurbAPI"));

    if (HasFlag(surf.kind, Kind::Plane))
        faceSurfaceType.push_back(TfToken("BrepSurfacePlaneAPI"));

    if (HasFlag(surf.kind, Kind::Cylinder))
        faceSurfaceType.push_back(TfToken("BrepSurfaceCylinderAPI"));

    if (HasFlag(surf.kind, Kind::Cone))
        faceSurfaceType.push_back(TfToken("BrepSurfaceConeAPI"));

    if (HasFlag(surf.kind, Kind::Sphere))
        faceSurfaceType.push_back(TfToken("BrepSurfaceSphereAPI"));

    if (HasFlag(surf.kind, Kind::Torus))
        faceSurfaceType.push_back(TfToken("BrepSurfaceTorusAPI"));
}

bool BrepRoutine::definePrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessParams& params
) const {
    SdfPath brepPath = protoPath.AppendChild(TfToken("Brep"));
    UsdSolidBrepArray brepArray = UsdSolidBrepArray::Define(stage, brepPath);

    for (const auto& surf : faceSurf) {
        addApis(surf, brepArray.GetPrim());
    }

    return true;
}

bool BrepRoutine::writePrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessParams& params
) const {
    SdfPath brepPath = protoPath.AppendChild(TfToken("Brep"));
    UsdSolidBrepArray protoBrep(stage->GetPrimAtPath(brepPath));

    size_t nFaces = faceLoopCount.size();
    size_t nEU    = edgeuseEdgeIndex.size();
    size_t nEdges = edge3d.size();
    size_t nVerts = verts.size();

    // Intersect Tolerance
    VtArray<double> tolArray{1e-4};
    protoBrep.CreateBrepIntersectTol3dAttr().Set(tolArray);

    // Extent Calculation
    std::array<double,3> mn{ std::numeric_limits<double>::max(),
                             std::numeric_limits<double>::max(),
                             std::numeric_limits<double>::max() };
    std::array<double,3> mx{ -std::numeric_limits<double>::max(),
                             -std::numeric_limits<double>::max(),
                             -std::numeric_limits<double>::max() };

    auto grow = [&](const std::array<double,3>& p){
        for (int a = 0; a < 3; ++a) { 
            mn[a] = std::min(mn[a], p[a]); 
            mx[a] = std::max(mx[a], p[a]);
        } 
    };
    for (auto& p : verts) grow(p);
    for (auto& s : surfaces) { for (auto& p : s.cp) grow(p); }
    for (auto& c : edge3d) { for (auto& p : c.cp) grow(p); }

    if (verts.empty() && surfaces.empty() && edge3d.empty()) { 
        mn = {0,0,0}; mx = {0,0,0}; 
    }

    // Topology Setup
    if (isSolid) {
        VtArray<uint> brepRegionCount{2};
        VtArray<uint> shellCount{1,1};
        VtArray<TfToken> regionType{TfToken("voidRegion"), TfToken("solidRegion")};
        VtArray<uint> faceusdCount{(uint)nFaces, (uint)nFaces};
        VtArray<uint> shellWireEdgeCount{0,0};
        VtArray<TfToken> shellPointType{TfToken("none"), TfToken("none")};

        protoBrep.CreateBrepRegionCountAttr().Set(brepRegionCount);
        protoBrep.CreateRegionShellCountAttr().Set(shellCount);
        protoBrep.CreateRegionTypeAttr().Set(regionType);
        protoBrep.CreateShellFaceuseCountAttr().Set(faceusdCount);
        protoBrep.CreateShellWireEdgeCountAttr().Set(shellWireEdgeCount);
        protoBrep.CreateShellPointTypeAttr().Set(shellPointType);
    } else {
        VtArray<uint> brepRegionCount{1};
        VtArray<uint> shellCount{1};
        VtArray<TfToken> regionType{TfToken("voidRegion")};
        VtArray<uint> faceusdCount{(uint)(2*nFaces)};
        VtArray<uint> shellWireEdgeCount{0};
        VtArray<TfToken> shellPointType{TfToken("none")};

        protoBrep.CreateBrepRegionCountAttr().Set(brepRegionCount);
        protoBrep.CreateRegionShellCountAttr().Set(shellCount);
        protoBrep.CreateRegionTypeAttr().Set(regionType);
        protoBrep.CreateShellFaceuseCountAttr().Set(faceusdCount);
        protoBrep.CreateShellWireEdgeCountAttr().Set(shellWireEdgeCount);
        protoBrep.CreateShellPointTypeAttr().Set(shellPointType);
    }

    // Faces & Loops
    VtArray<TfToken> faceSurfaceType;
    for (const auto& surf : faceSurf) {
        addFaceSurfaceTypes(surf, faceSurfaceType);
    }
    protoBrep.CreateFaceLoopCountAttr().Set(faceLoopCount);
    protoBrep.CreateFaceSurfaceTypeAttr().Set(faceSurfaceType);
    protoBrep.CreateFaceTrimTypeAttr().Set(faceTrimType);

    VtArray<GfVec2d> patches;
    for (size_t i = 0; i < faceRange.size(); ++i){ 
        auto& r = faceRange[i];
        patches.push_back(GfVec2d(r[0], r[2]));
        patches.push_back(GfVec2d(r[1], r[3]));
    }
    protoBrep.CreateFaceRangeAttr().Set(patches);

    protoBrep.CreateLoopEdgeuseCountAttr().Set(loopEdgeuseCount);
    protoBrep.CreateLoopVertexIndexAttr().Set(VtArray<uint>(loopEdgeuseCount.size(), 0));

    protoBrep.CreateEdgeuseEdgeIndexAttr().Set(edgeuseEdgeIndex);
    protoBrep.CreateEdgeuseOrientationTypeAttr().Set(edgeuseOrient);

    // Faceuses
    VtArray<TfToken> fo; 
    VtArray<uint> fidx;
    for (size_t fi = 0; fi < nFaces; ++fi) fo.push_back(TfToken(faceuseOrient[2*fi]));
    for (size_t fi = 0; fi < nFaces; ++fi) fo.push_back(TfToken(faceuseOrient[2*fi+1]));
    for (int pass = 0; pass < 2; ++pass) {
        for (size_t fi = 0; fi < nFaces; ++fi) fidx.push_back(fi);
    }
    protoBrep.CreateFaceuseOrientationTypeAttr().Set(fo);
    protoBrep.CreateFaceuseFaceIndexAttr().Set(fidx);

    protoBrep.CreateEdgeuseNextRadialEUIndexAttr().Set(edgeuseNextRadial);
    protoBrep.CreateEdgeuseThisRadialEntryTypeAttr().Set(edgeuseRadialEntry);

    // Edges Topology
    VtArray<TfToken> ct(nEdges, TfToken("BrepCurve3dNurbAPI"));
    protoBrep.CreateEdgeCurveTypeAttr().Set(ct);

    VtArray<double> er; 
    for(auto& r: edgeRange) {
        er.push_back(r[0]);
        er.push_back(r[1]);
    }
    protoBrep.CreateEdgeRangeAttr().Set(er);

    VtArray<GfVec2i> ev;
    for(size_t i = 0; i < edgeVtx.size(); ++i) { 
        ev.push_back(GfVec2i(edgeVtx[i][0], edgeVtx[i][1]));
    }
    protoBrep.CreateEdgeVertexIndicesAttr().Set(ev);

    // Vertices
    VtArray<TfToken> vt(nVerts, TfToken("BrepPointAPI"));
    protoBrep.CreateVertexPointTypeAttr().Set(vt);

    VtArray<GfVec3d> vp;
    for(auto& p: verts) {
        vp.push_back({p[0], p[1], p[2]});
    }
    UsdSolidBrepPointAPI pointPrim = UsdSolidBrepPointAPI::Apply(protoBrep.GetPrim(), TfToken("vertexPoint"));
    pointPrim.CreatePointPositionAttr().Set(vp);

    // 3D Edge NURBS Geometry Export
    if (!edge3d.empty()) {
        auto edgeNurbApi = UsdSolidBrepCurve3dNurbAPI::Apply(protoBrep.GetPrim(), TfToken("edge3dNurb"));

        VtArray<uint> edgeOrders;
        VtArray<uint> edgeVertexCounts;
        VtArray<double> edgeKnots;
        VtArray<GfVec3d> edgeControlVertices;
        VtArray<double> edgeWeights;

        edgeOrders.reserve(edge3d.size());
        edgeVertexCounts.reserve(edge3d.size());

        for (const Crv3& c : edge3d) {
            edgeOrders.push_back(c.order); // ensure your curve struct holds order (degree + 1)
            edgeVertexCounts.push_back(c.cp.size());

            edgeKnots.insert(edgeKnots.end(), c.k.begin(), c.k.end());
            if (!c.w.empty()) {
                edgeWeights.insert(edgeWeights.end(), c.w.begin(), c.w.end());
            }

            for (const auto& p : c.cp) {
                edgeControlVertices.emplace_back(p[0], p[1], p[2]);
            }
        }
        edgeNurbApi.CreateCurve3dOrderAttr().Set(edgeOrders);
        edgeNurbApi.CreateCurve3dVertexCountAttr().Set(edgeVertexCounts);
        edgeNurbApi.CreateCurve3dKnotsAttr().Set(edgeKnots);
        edgeNurbApi.CreateCurve3dControlVerticesAttr().Set(edgeControlVertices);
        if (!edgeWeights.empty()) {
            edgeNurbApi.CreateCurve3dWeightsAttr().Set(edgeWeights);
        }
    }

    // Surface NURBS Geometry Export
    Kind overallKind = Kind::None;
    for (const auto& surf : faceSurf) {
        unionizeSurfaceKind(overallKind, surf.kind);
    }

    if (HasFlag(overallKind, Kind::Nurb)) {
        auto api = UsdSolidBrepSurfaceNurbAPI::Apply(protoBrep.GetPrim());

        VtArray<uint> uOrders, vOrders, uVertexCounts, vVertexCounts;
        VtArray<double> uKnots, vKnots, weights;
        VtArray<GfVec3d> controlVertices;

        for (const auto& s : surfaces) {
            uOrders.push_back(s.uo);
            vOrders.push_back(s.vo);
            uVertexCounts.push_back(s.un);
            vVertexCounts.push_back(s.vn);

            uKnots.insert(uKnots.end(), s.uk.begin(), s.uk.end());
            vKnots.insert(vKnots.end(), s.vk.begin(), s.vk.end());
            weights.insert(weights.end(), s.w.begin(), s.w.end());

            for (const auto& p : s.cp)
                controlVertices.emplace_back(p[0], p[1], p[2]);
        }

        api.CreateSurfaceUOrderAttr().Set(uOrders);
        api.CreateSurfaceVOrderAttr().Set(vOrders);
        api.CreateSurfaceUVertexCountAttr().Set(uVertexCounts);
        api.CreateSurfaceVVertexCountAttr().Set(vVertexCounts);
        api.CreateSurfaceUKnotsAttr().Set(uKnots);
        api.CreateSurfaceVKnotsAttr().Set(vKnots);
        api.CreateSurfaceControlVerticesAttr().Set(controlVertices);
        if (!weights.empty()) {
            api.CreateSurfaceWeightsAttr().Set(weights);
        }
    }
    {
        VtArray<GfVec3d> planeOrigins, planeAxes, planeRefDirs;

        VtArray<GfVec3d> cylOrigins, cylAxes, cylRefDirs;
        VtArray<double> cylRadii;

        VtArray<GfVec3d> coneOrigins, coneAxes, coneRefDirs;
        VtArray<double> coneRadii, coneSemiAngles;

        VtArray<GfVec3d> sphereCenters, sphereAxes, sphereRefDirs;
        VtArray<double> sphereRadii;

        VtArray<GfVec3d> torusOrigins, torusAxes, torusRefDirs;
        VtArray<double> torusMajorRadii, torusMinorRadii;

        for (const auto& s : faceSurf) {
            switch (s.kind) {
            case Kind::Plane:
                planeOrigins.emplace_back(s.origin[0], s.origin[1], s.origin[2]);
                planeAxes.emplace_back(s.axis[0], s.axis[1], s.axis[2]);
                planeRefDirs.emplace_back(s.refDir[0], s.refDir[1], s.refDir[2]);
                break;

            case Kind::Cylinder:
                cylOrigins.emplace_back(s.origin[0], s.origin[1], s.origin[2]);
                cylAxes.emplace_back(s.axis[0], s.axis[1], s.axis[2]);
                cylRefDirs.emplace_back(s.refDir[0], s.refDir[1], s.refDir[2]);
                cylRadii.push_back(s.radius);
                break;

            case Kind::Cone:
                coneOrigins.emplace_back(s.origin[0], s.origin[1], s.origin[2]);
                coneAxes.emplace_back(s.axis[0], s.axis[1], s.axis[2]);
                coneRefDirs.emplace_back(s.refDir[0], s.refDir[1], s.refDir[2]);
                coneRadii.push_back(s.radius);
                coneSemiAngles.push_back(s.semiAngle);
                break;

            case Kind::Sphere:
                sphereCenters.emplace_back(s.origin[0], s.origin[1], s.origin[2]);
                sphereAxes.emplace_back(s.axis[0], s.axis[1], s.axis[2]);
                sphereRefDirs.emplace_back(s.refDir[0], s.refDir[1], s.refDir[2]);
                sphereRadii.push_back(s.radius);
                break;

            case Kind::Torus:
                torusOrigins.emplace_back(s.origin[0], s.origin[1], s.origin[2]);
                torusAxes.emplace_back(s.axis[0], s.axis[1], s.axis[2]);
                torusRefDirs.emplace_back(s.refDir[0], s.refDir[1], s.refDir[2]);
                torusMajorRadii.push_back(s.majorRadius);
                torusMinorRadii.push_back(s.minorRadius);
                break;

            default:
                break;
            }
        }

        if (HasFlag(overallKind, Kind::Plane)) {
            auto api = UsdSolidBrepSurfacePlaneAPI::Apply(protoBrep.GetPrim());
            api.CreateSurfacePlaneOriginAttr().Set(planeOrigins);
            api.CreateSurfacePlaneAxisAttr().Set(planeAxes);
            api.CreateSurfacePlaneRefDirectionAttr().Set(planeRefDirs);
        }

        if (HasFlag(overallKind, Kind::Cylinder)) {
            auto api = UsdSolidBrepSurfaceCylinderAPI::Apply(protoBrep.GetPrim());
            api.CreateSurfaceCylinderOriginAttr().Set(cylOrigins);
            api.CreateSurfaceCylinderAxisAttr().Set(cylAxes);
            api.CreateSurfaceCylinderRefDirectionAttr().Set(cylRefDirs);
            api.CreateSurfaceCylinderRadiusAttr().Set(cylRadii);
        }

        if (HasFlag(overallKind, Kind::Cone)) {
            auto api = UsdSolidBrepSurfaceConeAPI::Apply(protoBrep.GetPrim());
            api.CreateSurfaceConeOriginAttr().Set(coneOrigins);
            api.CreateSurfaceConeAxisAttr().Set(coneAxes);
            api.CreateSurfaceConeRefDirectionAttr().Set(coneRefDirs);
            api.CreateSurfaceConeRadiusAttr().Set(coneRadii);
            api.CreateSurfaceConeSemiAngleAttr().Set(coneSemiAngles);
        }

        if (HasFlag(overallKind, Kind::Sphere)) {
            auto api = UsdSolidBrepSurfaceSphereAPI::Apply(protoBrep.GetPrim());
            api.CreateSurfaceSphereCenterAttr().Set(sphereCenters);
            api.CreateSurfaceSphereAxisAttr().Set(sphereAxes);
            api.CreateSurfaceSphereRefDirectionAttr().Set(sphereRefDirs);
            api.CreateSurfaceSphereRadiusAttr().Set(sphereRadii);
        }

        if (HasFlag(overallKind, Kind::Torus)) {
            auto api = UsdSolidBrepSurfaceTorusAPI::Apply(protoBrep.GetPrim());
            api.CreateSurfaceTorusOriginAttr().Set(torusOrigins);
            api.CreateSurfaceTorusAxisAttr().Set(torusAxes);
            api.CreateSurfaceTorusRefDirectionAttr().Set(torusRefDirs);
            api.CreateSurfaceTorusMajorRadiusAttr().Set(torusMajorRadii);
            api.CreateSurfaceTorusMinorRadiusAttr().Set(torusMinorRadii);
        }
    }

    {
        VtArray<uint> uvOrders;
        VtArray<uint> uvVertexCounts;
        VtArray<double> uvKnots;
        VtArray<GfVec2d> uvControlVertices;
        VtArray<double> uvWeights;

        for (const auto& c : curveUv)
        {
            uvOrders.push_back(c.order);
            uvVertexCounts.push_back(c.n);

            uvKnots.insert(uvKnots.end(), c.k.begin(), c.k.end());
            uvWeights.insert(uvWeights.end(), c.w.begin(), c.w.end());

            for (const auto& p : c.cp)
                uvControlVertices.emplace_back(p[0], p[1]);
        }

        auto uvApi = UsdSolidBrepCurveUvNurbAPI::Apply(protoBrep.GetPrim());
        uvApi.CreateCurveUvOrderAttr().Set(uvOrders);
        uvApi.CreateCurveUvVertexCountAttr().Set(uvVertexCounts);
        uvApi.CreateCurveUvKnotsAttr().Set(uvKnots);
        uvApi.CreateCurveUvControlVerticesAttr().Set(uvControlVertices);
        uvApi.CreateCurveUvWeightsAttr().Set(uvWeights);
    }

    VtArray<double> intersectTol3d = {1e-6};
    protoBrep.CreateBrepIntersectTol3dAttr().Set(intersectTol3d);

    GfVec3d minExtent(mn[0], mn[1], mn[2]);
    GfVec3d maxExtent(mx[0], mx[1], mx[2]);
    VtArray<GfVec3d> extent{minExtent, maxExtent};
    protoBrep.CreateBrepExtentAttr().Set(extent);

    return true;
}

void BrepRoutine::clearPrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath
) const {
    stage->RemovePrim(protoPath.AppendChild(TfToken("Brep")));
}

size_t BrepRoutine::size() const {
    size_t vertsSize = verts.size();
    return vertsSize;
}