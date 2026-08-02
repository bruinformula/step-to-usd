#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <string>
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>


#include <ShapeAnalysis_Surface.hxx>
#include <ShapeCustom_Surface.hxx>
#include <BRep_Tool.hxx>
#include <GeomConvert.hxx>
#include <Geom_BSplineSurface.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_NurbsConvert.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRep_Builder.hxx>
#include <ShapeFix_Shape.hxx>
#include <ShapeUpgrade_ShapeDivideClosed.hxx>
#include <BRepLib.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <TopAbs_Orientation.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom_Plane.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <Standard_Failure.hxx>
#include <Geom2dConvert.hxx>
#include <GeomConvert.hxx>
#include <GeomProjLib.hxx>
#include <gp_Ax3.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Pln.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Cone.hxx>
#include <gp_Sphere.hxx>
#include <gp_Torus.hxx>
#include <gp_Circ.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>

#include <TColStd_Array1OfReal.hxx>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/pxr.h>
#include <pxr/base/work/loops.h>
#include <pxr/base/work/workTBB/loops_impl.h>

#include <pxr/base/vt/array.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/gf/vec2f.h>
#include <pxr/usd/sdf/path.h>

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
#include "StepUSD/Tessellation/TessellationRoutine.h"

PXR_NAMESPACE_USING_DIRECTIVE

using Clock = std::chrono::high_resolution_clock;
using Seconds = std::chrono::duration<double>;

// When true, author degenerate (null-3D-curve) pole/apex edgeuses instead of
// skipping them. A pole/apex edge has no 3D curve but a VALID pcurve (the pole
// line at v=+-pi/2, or the apex line): authoring it closes the face's UV loop
// so MakeFace(surface, wire, Inside=true) bounds the finite region. Needed by
// the NURBS-solid gallery (cone-nurbs/sphere-nurbs) whose seam-split faces are
// bounded only by meridians meeting at degenerate poles; WITHOUT the closing
// pole edges one hemisphere trims to the wrong (empty/complement) region.
// Default false: the cube/filleted/plane/box producer paths must stay
// byte-identical (the filleted fixture has degenerate seam edges too, and
// including them there perturbs its tessellation).
static bool gIncludeDegenerateEdges = false;

// Normalize a gp_Dir-derived vector to a unit std::array (validator requires
// unit-length axis/refDirection).
static std::array<double,3> unitv(const gp_Dir& d) {
    return { d.X(), d.Y(), d.Z() };   // gp_Dir is already unit length
}

// Detect a face's analytic surface type and extract schema params. Returns an
// AnalyticSurface with kind=="" when the surface is not one of the analytic primitives
// (caller then falls back to NURBS extraction).
static AnalyticSurface detectAnalytic(const TopoDS_Face& face) {
    AnalyticSurface a{};
    Handle(Geom_Surface) gs = BRep_Tool::Surface(face);
    if (gs.IsNull()) return a;
    // ShapeDivideClosed wraps a split periodic face in a
    // Geom_RectangularTrimmedSurface; unwrap to the analytic basis surface (the
    // trim range is carried by the face's UVBounds / edges, so params are the
    // basis surface's global frame).
    Handle(Geom_RectangularTrimmedSurface) rt =
        Handle(Geom_RectangularTrimmedSurface)::DownCast(gs);
    if (!rt.IsNull()) gs = rt->BasisSurface();
    const Handle(Standard_Type)& ty = gs->DynamicType();
    if (ty == STANDARD_TYPE(Geom_Plane)) {
        gp_Pln pln = Handle(Geom_Plane)::DownCast(gs)->Pln();
        const gp_Ax3& ax = pln.Position();
        gp_Pnt o = ax.Location();
        a.kind = Kind::Plane;
        a.origin = {o.X(),o.Y(),o.Z()};
        a.axis = unitv(ax.Direction());
        a.refDir = unitv(ax.XDirection());
    } else if (ty == STANDARD_TYPE(Geom_CylindricalSurface)) {
        gp_Cylinder cy = Handle(Geom_CylindricalSurface)::DownCast(gs)->Cylinder();
        const gp_Ax3& ax = cy.Position();
        gp_Pnt o = ax.Location();
        a.kind = Kind::Cylinder;
        a.origin = {o.X(),o.Y(),o.Z()};
        a.axis = unitv(ax.Direction());
        a.refDir = unitv(ax.XDirection());
        a.radius = cy.Radius();
    } else if (ty == STANDARD_TYPE(Geom_ConicalSurface)) {
        gp_Cone co = Handle(Geom_ConicalSurface)::DownCast(gs)->Cone();
        const gp_Ax3& ax = co.Position();
        gp_Pnt o = ax.Location();
        // Author OCCT's native cone frame + SIGNED semiAngle so the builder
        // reconstructs the exact same Geom_ConicalSurface the native pcurves and
        // face:range were parametrized against. NOTE: for an apex-up cone OCCT
        // reports a NEGATIVE semiAngle (the cone narrows along +axis). The
        // schema/validator convention wants semiAngle in (0, pi/2), so this one
        // fixture trips BrepArrayAnalyticSurfaces:InvalidConeSemiAngle. Emitting
        // a positive semiAngle instead (apex-canonical frame) requires
        // reprojecting the pcurves + face:range, which OCCT's GeomProjLib does
        // not do reliably for periodic analytic curves; keeping the native
        // signed value preserves correct tessellation geometry.
        gp_Dir axd = ax.Direction();
        double refR = co.RefRadius();
        double semi = co.SemiAngle();
        // Canonicalize to the validator convention (semiAngle>0, radius 0 at the
        // apex). Apex at w=-refR/tan(semi) along the OCCT axis; if semi<0 flip
        // the axis so the surface's +v (which the reprojected pcurves will use)
        // grows toward the widening base. Cone faces reproject their pcurves +
        // face:range onto this canonical surface (see addFace).
        const double t = std::tan(semi);
        std::array<double,3> apex = { o.X(), o.Y(), o.Z() };
        if (std::fabs(t) > 1e-12) {
            double w = -refR / t;
            apex = { o.X()+w*axd.X(), o.Y()+w*axd.Y(), o.Z()+w*axd.Z() };
        }
        std::array<double,3> uaxis = { axd.X(), axd.Y(), axd.Z() };
        double usemi = semi;
        if (usemi < 0.0) { uaxis = {-uaxis[0],-uaxis[1],-uaxis[2]}; usemi = -usemi; }
        a.kind = Kind::Cone;
        a.origin = apex;
        a.axis = uaxis;
        a.refDir = unitv(ax.XDirection());
        a.radius = 0.0;
        a.semiAngle = usemi;
        a.reproject = true;             // cone needs pcurve reprojection onto canonical frame
        // FIX 2: v is authored as AXIAL distance (schema R(v)=radius+v*tan(semi),
        // + v*axis). OCCT's v is SLANT; scale by cos(semiAngle) on emit.
        a.vScale = std::cos(usemi);
    } else if (ty == STANDARD_TYPE(Geom_SphericalSurface)) {
        gp_Sphere sp = Handle(Geom_SphericalSurface)::DownCast(gs)->Sphere();
        const gp_Ax3& ax = sp.Position();
        gp_Pnt o = ax.Location();
        a.kind = Kind::Sphere;
        a.origin = {o.X(),o.Y(),o.Z()};   // sphere center
        a.axis = unitv(ax.Direction());
        a.refDir = unitv(ax.XDirection());
        a.radius = sp.Radius();
    } else if (ty == STANDARD_TYPE(Geom_ToroidalSurface)) {
        gp_Torus to = Handle(Geom_ToroidalSurface)::DownCast(gs)->Torus();
        const gp_Ax3& ax = to.Position();
        gp_Pnt o = ax.Location();
        a.kind = Kind::Torus;
        a.origin = {o.X(),o.Y(),o.Z()};   // torus center
        a.axis = unitv(ax.Direction());
        a.refDir = unitv(ax.XDirection());
        a.majorRadius = to.MajorRadius();
        a.minorRadius = to.MinorRadius();
    }
    return a;
}

// Rebuild the OCCT Geom_Surface from an AnalyticSurface, in the SAME (possibly
// canonicalized) frame the builder will reconstruct. Used to reproject the
// boundary pcurves so curveUv is consistent with the AUTHORED analytic surface
// (not OCCT's original frame — critical for the apex-canonicalized cone).
static occt::handle<Geom_Surface> canonicalSurface(const AnalyticSurface& a) {
    gp_Pnt o(a.origin[0], a.origin[1], a.origin[2]);
    gp_Dir ax(a.axis[0], a.axis[1], a.axis[2]);
    gp_Dir rx(a.refDir[0], a.refDir[1], a.refDir[2]);
    gp_Ax3 frame(o, ax, rx);
    switch (a.kind) {
        case Kind::Plane:
            return new Geom_Plane(frame);
        case Kind::Cylinder:
            return new Geom_CylindricalSurface(frame, a.radius);
        case Kind::Cone:
            return new Geom_ConicalSurface(frame, a.semiAngle, a.radius);
        case Kind::Sphere:
            return new Geom_SphericalSurface(frame, a.radius);
        case Kind::Torus:
            return new Geom_ToroidalSurface(frame, a.majorRadius, a.minorRadius);
        default:
            return Handle(Geom_Surface)();
    }
    return Handle(Geom_Surface)();
}

static Surf extractSurface(const Handle(Geom_Surface)& gs) {
    Surf s{};
    Handle(Geom_BSplineSurface) bs = Handle(Geom_BSplineSurface)::DownCast(gs);
    if (bs.IsNull()) return s;

    s.un = bs->NbUPoles(); s.vn = bs->NbVPoles();
    s.uo = bs->UDegree()+1; s.vo = bs->VDegree()+1;

    TColStd_Array1OfReal uks(1, s.un + s.uo); bs->UKnotSequence(uks);
    TColStd_Array1OfReal vks(1, s.vn + s.vo); bs->VKnotSequence(vks);
    for (int i = uks.Lower(); i <= uks.Upper(); ++i) s.uk.push_back(uks.Value(i));
    for (int i = vks.Lower(); i <= vks.Upper(); ++i) s.vk.push_back(vks.Value(i));

    bool rational = bs->IsURational() || bs->IsVRational();
    for (int u = 1; u <= s.un; ++u) {
        for (int v = 1; v <= s.vn; ++v) {
            gp_Pnt p = bs->Pole(u, v);
            s.cp.push_back({p.X(), p.Y(), p.Z()});
            s.w.push_back(rational ? bs->Weight(u, v) : 1.0);
        }
    }
    return s;
}

static Surf extractSurface(const TopoDS_Face& face) {
    Surf s{};
    TopLoc_Location loc;
    Handle(Geom_Surface) gs = BRep_Tool::Surface(face, loc);
    if (gs.IsNull()) return s;

    if (!loc.IsIdentity()) {
        gs = Handle(Geom_Surface)::DownCast(gs->Transformed(loc.Transformation()));
    }

    Handle(Geom_BSplineSurface) bs = Handle(Geom_BSplineSurface)::DownCast(gs);
    if (bs.IsNull()) {
        try {
            Standard_Real uMin = 0.0, uMax = 0.0, vMin = 0.0, vMax = 0.0;
            BRepTools::UVBounds(face, uMin, uMax, vMin, vMax);

            if (Precision::IsInfinite(uMin) || Precision::IsInfinite(uMax)) { uMin = 0.0; uMax = 1.0; }
            if (Precision::IsInfinite(vMin) || Precision::IsInfinite(vMax)) { vMin = 0.0; vMax = 1.0; }

            Handle(Geom_RectangularTrimmedSurface) trimmed =
                new Geom_RectangularTrimmedSurface(gs, uMin, uMax, vMin, vMax);

            bs = GeomConvert::SurfaceToBSplineSurface(trimmed);
        } catch (const Standard_Failure& e) {
            LOG_ERR(std::string("Failed to convert surface to B-Spline: ") + e.GetMessageString());
            return s;
        }
    }

    return extractSurface(Handle(Geom_Surface)(bs));
}

static Crv3 extractCurve3d(Handle(Geom_BSplineCurve) bc) {
    Crv3 c{};
    c.order = bc->Degree()+1; c.n = bc->NbPoles();
    TColStd_Array1OfReal ks(1, c.n + c.order); bc->KnotSequence(ks);
    for (int i = ks.Lower(); i <= ks.Upper(); ++i) c.k.push_back(ks.Value(i));
    bool rational = bc->IsRational();
    for (int i = 1; i <= c.n; ++i) {
        gp_Pnt p = bc->Pole(i);
        c.cp.push_back({p.X(), p.Y(), p.Z()});
        c.w.push_back(rational ? bc->Weight(i) : 1.0);
    }
    return c;
}

static Crv2 extractCurve2d(Handle(Geom2d_BSplineCurve) bc) {
    Crv2 c{};
    c.order = bc->Degree()+1; c.n = bc->NbPoles();
    TColStd_Array1OfReal ks(1, c.n + c.order); bc->KnotSequence(ks);
    for (int i = ks.Lower(); i <= ks.Upper(); ++i) c.k.push_back(ks.Value(i));
    bool rational = bc->IsRational();
    for (int i = 1; i <= c.n; ++i) {
        gp_Pnt2d p = bc->Pole(i);
        c.cp.push_back({p.X(), p.Y()});
        c.w.push_back(rational ? bc->Weight(i) : 1.0);
    }
    return c;
}

// Build a 2-pole placeholder line BSpline between two 3D points (for a
// degenerate / null-curve edge), so the per-edge arrays stay aligned.
static Crv3 placeholderLine3d(const std::array<double,3>& a, const std::array<double,3>& b) {
    Crv3 c{};
    c.order = 2; c.n = 2;
    c.k = {0.0, 0.0, 1.0, 1.0};
    c.cp = {a, b};
    c.w = {1.0, 1.0};
    return c;
}

// ---- per-shape sharing context (TopExp maps over the whole shape) ----
struct BrepContext {
    TopTools_IndexedMapOfShape vmap; // unique vertices (orientation-independent)
    TopTools_IndexedMapOfShape emap; // unique edges
    // Per emap edge (0-based): degeneracy (null 3D curve) + raw geometry, kept
    // until the post-pass compacts out unused/degenerate edges.
    std::vector<bool> edgeDegenerate;
    std::vector<Crv3> edgeCrv3;       // 3D curve per emap edge
    std::vector<std::array<double,2>> edgeRng; // range per emap edge
    std::vector<std::array<int,2>> edgeVtx;    // endpoint vertex indices per emap edge
    // For each emap edge index: the global edgeuse indices that reference it,
    // in discovery order. Used to stitch the radial ring after all faces.
    std::map<int, std::vector<int>> edgeToEUs;
};

// Append one face (with all its wires) to the Out arrays.
void BrepRoutine::addFace(BrepContext& ctx, const TopoDS_Face& face) {
    // surface: prefer the native analytic type (routes the builder through its
    // robust analytic path); fall back to NURBS extraction otherwise.
    AnalyticSurface anal = detectAnalytic(face);
    faceSurf.push_back(anal);
    if (anal.kind == Kind::None)
        surfaces.push_back(extractSurface(face));   // NURBS, packed in face order over NURBS faces
    // face range (corner-point form is emitted later in emit()). FIX 2: for a
    // cone, OCCT's v is slant distance; author it as AXIAL (v*cos(semiAngle)) so
    // face:range is consistent with the schema's cone parameterization.
    Standard_Real u0,u1,v0,v1; BRepTools::UVBounds(face, u0,u1,v0,v1);
    faceRange.push_back({u0,u1,v0*anal.vScale,v1*anal.vScale});
    // faceuse orientation: REVERSED -> outward against natural normal
    bool rev = (face.Orientation() == TopAbs_REVERSED);
    faceuseOrient.push_back(rev ? TfToken("opposite") : TfToken("same"));
    faceuseOrient.push_back(rev ? TfToken("same") : TfToken("opposite"));

    // wires: outer first, then holes
    TopoDS_Wire outer = BRepTools::OuterWire(face);
    std::vector<TopoDS_Wire> wires;
    wires.push_back(outer);
    for (TopExp_Explorer we(face, TopAbs_WIRE); we.More(); we.Next()) {
        TopoDS_Wire w = TopoDS::Wire(we.Current());
        if (!w.IsSame(outer)) wires.push_back(w);
    }
    faceLoopCount.push_back((int)wires.size());

    for (size_t wi = 0; wi < wires.size(); ++wi) {
        const TopoDS_Wire& w = wires[wi];
        // Collect this loop's edgeuses, in traversal order. Each parallel array
        // entry describes ONE edgeuse: its 2D pcurve (curveUv), the SHARED
        // unique-edge index, and the orientation token.
        std::vector<Crv2> p2;                   // curveUv per edgeuse
        std::vector<int> euEdgeIdx;                  // shared edge index per eu
        std::vector<std::string> euOrient;           // token per eu
        for (BRepTools_WireExplorer wexp(w, face); wexp.More(); wexp.Next()) {
            const TopoDS_Edge& edge = wexp.Current();
            try {
                int eidx = ctx.emap.FindIndex(edge) - 1; // 0-based SHARED index
                // Skip edgeuses on degenerate (null-3D-curve) seam/pole edges, as
                // the legacy fan path did. They carry no boundary geometry the
                // NURBS trim path needs, and authoring them as extra trim
                // segments perturbs the tessellation. The edge itself is dropped
                // by the post-pass compaction (so no orphan-edge error).
                // Exception (gIncludeDegenerateEdges): the NURBS-solid gallery
                // authors them, using their valid pcurve, to CLOSE pole/apex UV
                // loops so MakeFace(Inside) bounds the finite region.
                if (!gIncludeDegenerateEdges &&
                    eidx >= 0 && eidx < (int)ctx.edgeDegenerate.size() &&
                    ctx.edgeDegenerate[eidx]) {
                    continue;
                }
                Standard_Real f2,l2;
                Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(edge, face, f2, l2);
                if (pc.IsNull()) continue;
                // Trim to the edge's range FIRST (handles full/periodic basis
                // curves), then convert to BSpline.
                Handle(Geom2d_BSplineCurve) bpc = Geom2dConvert::CurveToBSplineCurve(
                    new Geom2d_TrimmedCurve(pc, f2, l2));
                bool reversed = (edge.Orientation() == TopAbs_REVERSED);
                if (reversed) bpc->Reverse();
                Crv2 uv = extractCurve2d(bpc);
                // FIX 2: scale the pcurve's v (second) coordinate from OCCT slant
                // to schema axial for cone faces, so the authored curveUv stays
                // consistent with the axial face:range and cone parameterization.
                // vScale is 1.0 for all non-cone surfaces (no-op).
                if (anal.vScale != 1.0)
                    for (auto& cp : uv.cp) cp[1] *= anal.vScale;
                p2.push_back(uv);
                euEdgeIdx.push_back(eidx);
                // "opposite" when this edgeuse traverses the edge against its
                // natural (FORWARD) sense.
                euOrient.push_back(reversed ? "opposite" : "same");
            } catch (const Standard_Failure& e) {
                std::fprintf(stderr, "  skip edge: %s\n", e.GetMessageString());
            }
        }
        // Author the standard STEP/OCCT loop-winding convention (material on
        // the left): the OUTER loop (wi==0) winds CCW in UV (positive signed
        // area); every INNER (hole) loop (wi>0) winds CW (negative signed area).
        // This matches the SMLib OCCT/PRC->USD converters and OCCT itself, which
        // key off orientationType and expect outer-CCW/inner-CW; hdOcct's builder
        // now consumes the authored orientation directly (it no longer force-
        // reverses inner wires). Shoelace over all 2D control points in order.
        const bool wantCCW = (wi == 0);      // outer CCW, holes CW
        double signedA = 0.0; std::vector<std::array<double,2>> poly;
        for (auto& c : p2) for (auto& cp : c.cp) poly.push_back(cp);
        for (size_t i=0;i+1<poly.size();++i)
            signedA += poly[i][0]*poly[i+1][1] - poly[i+1][0]*poly[i][1];
        if (!poly.empty())
            signedA += poly.back()[0]*poly.front()[1] - poly.front()[0]*poly.back()[1];
        if ((wantCCW && signedA < 0) || (!wantCCW && signedA > 0)) {
            // Reverse the per-edgeuse lists in LOCKSTEP so curveUv[k],
            // edgeuseEdgeIndex[k] and orientation stay aligned, and reverse the
            // pcurve direction (and flip the orientation token) of each.
            std::reverse(p2.begin(), p2.end());
            std::reverse(euEdgeIdx.begin(), euEdgeIdx.end());
            std::reverse(euOrient.begin(), euOrient.end());
            for (auto& c : p2) {
                std::reverse(c.cp.begin(), c.cp.end());
                std::reverse(c.w.begin(), c.w.end());
                // knot vector of a reversed BSpline: reflect about its span
                double a = c.k.front(), b = c.k.back();
                std::vector<double> nk(c.k.size());
                for (size_t i=0;i<c.k.size();++i) nk[i] = a + b - c.k[c.k.size()-1-i];
                c.k.swap(nk);
            }
            for (auto& t : euOrient) t = (t == "same") ? "opposite" : "same";
        }
        int euCount = 0;
        for (size_t i=0;i<p2.size();++i) {
            int euIdx = (int)curveUv.size();
            curveUv.push_back(p2[i]);
            edgeuseEdgeIndex.push_back(euEdgeIdx[i]);
            edgeuseOrient.push_back(TfToken(euOrient[i]));
            ctx.edgeToEUs[euEdgeIdx[i]].push_back(euIdx);
            ++euCount;
        }
        loopEdgeuseCount.push_back(euCount);
    }
}

bool BrepRoutine::tessellate(
    const TopoDS_Shape& defShape, 
    const TessParams& params,
    const SdfPath& protoPath
) {
    auto tessellateStart = Clock::now();

    if (defShape.IsNull()) {
        LOG_ERR("Input shape is null for prototype: " + protoPath.GetString());
        return false;
    }

    // Repair shape
    ShapeFix_Shape shapeHealer(defShape);
    shapeHealer.SetPrecision(1e-6);

    shapeHealer.FixFreeWireMode() = true;
    shapeHealer.FixSolidMode() = true;

    shapeHealer.FixWireTool()->FixAddCurve3dMode() = true;
    shapeHealer.FixWireTool()->FixAddPCurveMode() = true;
    shapeHealer.FixWireTool()->FixSmallMode() = true;
    
    shapeHealer.FixFaceTool()->FixSmallAreaWireMode() = true;
    shapeHealer.FixFaceTool()->FixSplitFaceMode() = true;
    shapeHealer.FixFaceTool()->FixLoopWiresMode() = true;

    shapeHealer.Perform();

    TopoDS_Shape shape = shapeHealer.Shape();
    if (shape.IsNull()) {
        LOG_ERR("Shape became null after ShapeFix_Shape: " + protoPath.GetString());
        return false;
    }

    // Split closed/periodic surfaces (e.g. 360-deg cylinders/toruses)
    ShapeUpgrade_ShapeDivideClosed div(shape); 
    div.Perform(); 
    shape = div.Result();
    if (shape.IsNull()) {
        LOG_ERR("Resulting shape is null after ShapeUpgrade_ShapeDivideClosed.");
        return false;
    }

    // Analyzer check
    BRepCheck_Analyzer analyzer(shape);
    if (!analyzer.IsValid()) {
        LOG_ERR("Shape invalid after ShapeUpgrade_ShapeDivideClosed");
        // Continuing anyway to allow partial recovery
    }

    // Build 3D curves
    if (!BRepLib::BuildCurves3d(shape)) {
        LOG_ERR("Failed rebuilding curves");
    }

    // BRepTools::Write(shape, "rebuilt.brep");

    this->isSolid = TopExp_Explorer(shape, TopAbs_SOLID).More();

    BrepContext ctx;
    // Orientation-independent unique-vertex / unique-edge maps.
    TopExp::MapShapes(shape, TopAbs_VERTEX, ctx.vmap);
    TopExp::MapShapes(shape, TopAbs_EDGE, ctx.emap);

    // Per UNIQUE VERTEX: position.
    for (int i = 1; i <= ctx.vmap.Extent(); ++i) {
        gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(ctx.vmap(i)));
        verts.push_back({p.X(), p.Y(), p.Z()});
    }

    // Per UNIQUE EDGE (emap order, 0-based): 3D curve, range, endpoint vertex
    // indices, degeneracy. Stored in ctx; compacted into Out after the faces are
    // explored (degenerate / unreferenced edges are dropped).
    int degenerateCount = 0;
    const int nEmap = ctx.emap.Extent();
    ctx.edgeDegenerate.assign(nEmap, false);
    ctx.edgeCrv3.resize(nEmap);
    ctx.edgeRng.resize(nEmap);
    ctx.edgeVtx.resize(nEmap);
    for (int i = 1; i <= nEmap; ++i) {
        const TopoDS_Edge& edge = TopoDS::Edge(ctx.emap(i));
        // endpoint vertices (orientation-independent indices into vmap).
        // TopExp::Vertices(CumOri=false) returns (FORWARD-vertex, REVERSED-vertex)
        // of the edge as stored. emap holds edges FORWARD, so v1 sits at the 3D
        // curve's first parameter and v2 at the last -- i.e. already in
        // curve-parametric order. We do NOT reorder here by topology.
        TopoDS_Vertex v1, v2;
        TopExp::Vertices(edge, v1, v2);
        int i1 = v1.IsNull() ? 0 : ctx.vmap.FindIndex(v1) - 1;
        int i2 = v2.IsNull() ? 0 : ctx.vmap.FindIndex(v2) - 1;
        ctx.edgeVtx[i-1] = {i1, i2};

        Standard_Real f3,l3;
        Handle(Geom_Curve) c3 = BRep_Tool::Curve(edge, f3, l3);
        bool degen = c3.IsNull();
        if (!degen) {
            try {
                Handle(Geom_BSplineCurve) bc3 = GeomConvert::CurveToBSplineCurve(
                    new Geom_TrimmedCurve(c3, f3, l3));
                ctx.edgeCrv3[i-1] = extractCurve3d(bc3);
                ctx.edgeRng[i-1] = {f3, l3};
                // FIX 1 (proposal rule 434: "the curve runs from the start vertex
                // to the end vertex"). Author edge:vertexIndices in CURVE-PARAMETRIC
                // order: [0] = vertex at the curve's START (param f3), [1] = vertex
                // at its END (param l3). We enforce this explicitly by matching the
                // authored 3D curve's endpoint control points to the endpoint
                // vertices, rather than relying on TopExp topological order. This
                // makes the convention robust even if an emap edge were ever stored
                // REVERSED (curve start would then be v2, not v1). edge:range stays
                // (f3, l3), consistent with the 3D curve's natural direction.
                if (!ctx.edgeCrv3[i-1].cp.empty() && i1 >= 0 && i2 >= 0 &&
                    i1 < (int)verts.size() && i2 < (int)verts.size()) {
                    const auto& cpFirst = ctx.edgeCrv3[i-1].cp.front();
                    const auto& cpLast  = ctx.edgeCrv3[i-1].cp.back();
                    auto d2 = [](const std::array<double,3>& a,
                                 const std::array<double,3>& b){
                        double s=0; for(int k=0;k<3;++k){double e=a[k]-b[k]; s+=e*e;} return s; };
                    // Distance of the curve's start CP to v1 vs v2: if it is closer
                    // to v2, the stored (v1,v2) is the curve's (end,start) -> swap.
                    double dStartV1 = d2(cpFirst, verts[i1]);
                    double dStartV2 = d2(cpFirst, verts[i2]);
                    double dEndV2   = d2(cpLast,  verts[i2]);
                    double dEndV1   = d2(cpLast,  verts[i1]);
                    if (dStartV2 + dEndV1 < dStartV1 + dEndV2) {
                        ctx.edgeVtx[i-1] = {i2, i1};
                    }
                }
            } catch (const Standard_Failure& e) {
                std::fprintf(stderr, "  edge %d curve convert failed: %s\n",
                             i-1, e.GetMessageString());
                degen = true;
            }
        }
        ctx.edgeDegenerate[i-1] = degen;
        if (degen) {
            ++degenerateCount;
            // When the gallery authors degenerate edges, give each a placeholder
            // 3D line between its endpoint vertices + a unit range, so the
            // per-edge 3D arrays stay valid if a surviving edgeuse references it.
            if (gIncludeDegenerateEdges) {
                std::array<double,3> pa = (i1>=0 && i1<(int)verts.size())
                    ? verts[i1] : std::array<double,3>{{0,0,0}};
                std::array<double,3> pb = (i2>=0 && i2<(int)verts.size())
                    ? verts[i2] : pa;
                ctx.edgeCrv3[i-1] = placeholderLine3d(pa, pb);
                ctx.edgeRng[i-1] = {0.0, 1.0};
            }
        }
    }

    // Faces -> edgeuses (curveUv per edgeuse). edgeuse:edgeIndex temporarily
    // holds the EMAP index; compaction below remaps it to the authored index.
    for (TopExp_Explorer fx(shape, TopAbs_FACE); fx.More(); fx.Next())
        addFace(ctx, TopoDS::Face(fx.Current()));

    // ---- compact edges: author only emap edges actually referenced by a
    // surviving edgeuse, assign 0-based authored indices, remap. ----
    std::vector<int> emap2authored(nEmap, -1);
    for (int eu = 0; eu < (int)edgeuseEdgeIndex.size(); ++eu) {
        int em = edgeuseEdgeIndex[eu];
        if (em < 0 || em >= nEmap) continue;
        if (emap2authored[em] < 0) {
            emap2authored[em] = (int)edge3d.size();
            edge3d.push_back(ctx.edgeCrv3[em]);
            edgeRange.push_back(ctx.edgeRng[em]);
            edgeVtx.push_back(ctx.edgeVtx[em]);
        }
    }
    for (int eu = 0; eu < (int)edgeuseEdgeIndex.size(); ++eu)
        edgeuseEdgeIndex[eu] = emap2authored[edgeuseEdgeIndex[eu]];

    // ---- radial ring: per authored edge, its edgeuses form ONE cycle ----
    size_t nEU = edgeuseEdgeIndex.size();
    edgeuseNextRadial.assign(nEU, 0);
    edgeuseRadialEntry.assign(nEU, TfToken("topEntry"));
    // Group edgeuses by AUTHORED edge index (ctx.edgeToEUs is keyed by emap).
    std::map<int, std::vector<int>> authoredEdgeToEUs;
    for (auto& kv : ctx.edgeToEUs) {
        int au = (kv.first >= 0 && kv.first < nEmap) ? emap2authored[kv.first] : -1;
        if (au < 0) continue;
        auto& dst = authoredEdgeToEUs[au];
        dst.insert(dst.end(), kv.second.begin(), kv.second.end());
    }
    for (auto& kv : authoredEdgeToEUs) {
        const std::vector<int>& eus = kv.second;
        size_t m = eus.size();
        for (size_t k = 0; k < m; ++k) {
            // cycle: eu[k] -> eu[(k+1)%m]  (1 edgeuse -> self-loop)
            edgeuseNextRadial[eus[k]] = eus[(k+1)%m];
            // alternate top/bottom by position in the ring
            edgeuseRadialEntry[eus[k]] = (k % 2 == 0) ? TfToken("topEntry") : TfToken("bottomEntry");
        }
    }

    std::ostringstream text;
    text << "wrote " << " | solid=" << (int)isSolid
         << " faces=" << faceLoopCount.size()
         << " authoredEdges=" << edge3d.size()
         << " edgeuses=" << edgeuseEdgeIndex.size()
         << " verts=" << verts.size()
         << " emapEdges=" << nEmap
         << " degenerateEdges=" << degenerateCount
         << "\n";
        
    LOG_INFO(text.str());


    return true;
}