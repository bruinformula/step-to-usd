

#include <chrono>
#include <exception>
#include <limits>
#include <string>

#include <TDF_Label.hxx>
#include <TopLoc_Location.hxx>
#include <gp_Trsf.hxx>
#include <ShapeFix_Shape.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <Poly_Triangulation.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <GeomLProp_SLProps.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <BRepBndLib.hxx>
#include <TopExp.hxx>
#include <IMeshTools_Parameters.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_QuasiUniformDeflection.hxx>
#include <BRepExtrema_SelfIntersection.hxx>
#include <BRepTools.hxx>
#include <BRepExtrema_MapOfIntegerPackedMapOfInteger.hxx>
#include <Bnd_Box.hxx>
#include <BOPAlgo_Splitter.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRep_Builder.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <GeomAbs_Shape.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_IndexedMap.hxx>
#include <NCollection_List.hxx>
#include <Poly_PolygonOnTriangulation.hxx>
#include <Poly_Triangle.hxx>
#include <ShapeAnalysis_FreeBounds.hxx>
#include <Standard_Failure.hxx>
#include <Standard_Handle.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopTools_HSequenceOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Message_ProgressRange.hxx>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/pxr.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/imageable.h>
#include <pxr/usd/sdf/path.h>

#include <pxr/base/work/loops.h>
#include <pxr/base/work/workTBB/loops_impl.h>
#include <pxr/base/vt/array.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/gf/vec2f.h>

#pragma pop_macro("Handle")

#include "CadUSD/Logger.h"
#include "CadUSD/Tessellation/TessellationRoutine.h"
#include "CadUSD/Tessellation/TessellationUtils.h"
#include "CadUSD/Tessellation/DeadlineProgressIndicator.h"

PXR_NAMESPACE_USING_DIRECTIVE

bool TessellationRoutine::tessellate(
    const TopoDS_Shape& defShape, 
    const TessParams& params,
    const SdfPath& protoPath
) {
    using Clock = std::chrono::high_resolution_clock;
    using Seconds = std::chrono::duration<double>;

    auto tessellateStart = Clock::now();

    std::string protoName = protoPath.GetAsString();

    TopoDS_Shape fixedShape = defShape;
    if (params.meshEnableRepairPass) {
        LOG_DEBUG("  -> tessellatePart: ShapeFix_Shape (Repair pass)");
        ShapeFix_Shape fixer(defShape);
        fixer.SetPrecision(params.meshFixPrecision);
        fixer.SetMaxTolerance(params.meshFixTolerance);
        
        bool fixTimedOut = runWithDeadline(
            std::chrono::milliseconds(params.meshFixTimeout),
            "ShapeFix_Shape for part " + protoName + ": ",
            fixer
        );
    
        if (fixTimedOut) {
            LOG_DEBUG("  -> ShapeFix_Shape timed out, proceeding with partial repair");
        }

        TopoDS_Shape fixedShape = fixer.Shape();
    }

    LOG_DEBUG("  -> tessellatePart: BRepTools::Clean");
    BRepTools::Clean(defShape); // remove previously created tessellations for this part 

    double diagonal = computeBoundingBoxDiagonal(defShape);

    bool renderOnly = params.renderPurposeThreshold != std::numeric_limits<double>::infinity() && diagonal < params.renderPurposeThreshold;

    IMeshTools_Parameters meshParams;
    meshParams.InParallel = false; 
    meshParams.Deflection = diagonal * params.meshLinearDeflection;
    meshParams.Angle = params.meshAngularDeflection; // in radians
    meshParams.MinSize = meshParams.Deflection * params.meshMinSize;
    
    LOG_DEBUG("  -> tessellatePart: BRepMesh_IncrementalMesh");
    BRepMesh_IncrementalMesh mesher;
    mesher.SetShape(defShape);
    mesher.ChangeParameters() = meshParams;
    
    bool meshTimedOut = runWithDeadline(
        std::chrono::milliseconds(params.meshMeshTimeout),
        "BRepMesh_IncrementalMesh for part " + protoName + ": ",
        mesher
    );

    if (meshTimedOut) {
        LOG_DEBUG("  -> BRepMesh_IncrementalMesh timed out"); // Some faces will have null triangulations
    }

    int maxPasses = params.meshMaxNumberRemeshPasses;
    IMeshTools_Parameters repairParams = meshParams;

    // repeat check for self-intersections 
    // if fail, refine the mesh until there are none 
    // or we hit the max pass count.

    // important note this is on the whole shape 
    // not just the intersected shapes, 
    // so the edge walk later works

    LOG_DEBUG("  -> tessellatePart: Starting remesh passes (" + std::to_string(maxPasses) + ")");
    for (int pass = 0; pass < maxPasses; ++pass) {
        LOG_DEBUG("  Running self-intersection check (pass " + std::to_string(pass) + ")");
        BRepExtrema_SelfIntersection checker(defShape, params.meshSelfIntersectionThreshold);
        LOG_DEBUG("  -> checker.Perform()");
        checker.Perform();

        if (!checker.IsDone()) {
            LOG_DEBUG("  -> checker not done, breaking");
            break;
        }

        LOG_DEBUG("  -> checker getting OverlapElements");
        const BRepExtrema_MapOfIntegerPackedMapOfInteger& overlaps = checker.OverlapElements();

        if (overlaps.IsEmpty()) {
            LOG_DEBUG("  No interesections found.");
            break;
        }
        
        LOG_DEBUG("  Found overlaps. Remeshing with finer parameters.");

        repairParams.Deflection *= 0.5;
        repairParams.Angle *= 0.5;

        LOG_DEBUG("  -> BRepTools::Clean (repair)");
        BRepTools::Clean(defShape); 
        BRepMesh_IncrementalMesh remesher;
        remesher.SetShape(defShape);
        remesher.ChangeParameters() = repairParams;
        
        bool remeshTimedOut = runWithDeadline(
            std::chrono::milliseconds(params.meshRemeshTimeout),
            "BRepMesh_IncrementalMesh for part " + protoName + ": ",
            remesher
        );

        if (remeshTimedOut) {
            LOG_DEBUG("  -> BRepMesh_IncrementalMesh (repair) timed out");
            break;
        }
    }
    
    auto meshEnd = Clock::now();
    LOG_DEBUG("  Mesh time: " + std::to_string(Seconds(meshEnd - tessellateStart).count()) + " s");

    bool instSuccess = false;
    try {
        instSuccess = instRoutine.tessellate(fixedShape, params, protoPath);
    } catch (std::exception& e) {
        LOG_ERR(e.what());
    }
    bool meshSuccess = meshRoutine.tessellate(fixedShape, params, protoPath);
    bool sketchSuccess = sketchRoutine.tessellate(fixedShape, params, protoPath);
    
    return instSuccess && meshSuccess && sketchSuccess;
}

bool TessellationRoutine::definePrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessParams& params
) const {
    UsdPrim protoPrim = stage->GetPrimAtPath(protoPath);
    if (renderOnly) UsdGeomImageable(protoPrim).CreatePurposeAttr();

    bool instDefined = instRoutine.definePrim(stage, protoPath, params);
    bool meshDefined = meshRoutine.definePrim(stage, protoPath, params);
    bool sketchDefined = sketchRoutine.definePrim(stage, protoPath, params);
    return instDefined && meshDefined && sketchDefined;
}

bool TessellationRoutine::writePrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessParams& params
) const {
    UsdPrim protoPrim = stage->GetPrimAtPath(protoPath);
    if (renderOnly) UsdGeomImageable(protoPrim).CreatePurposeAttr().Set(UsdGeomTokens->render); 

    bool instWritten = instRoutine.writePrim(stage, protoPath, params);
    bool meshWritten = meshRoutine.writePrim(stage, protoPath, params);
    bool sketchWritten = sketchRoutine.writePrim(stage, protoPath, params);
    return instWritten && meshWritten && sketchWritten;
}

void TessellationRoutine::clearPrim(
    UsdStageRefPtr stage,
    const SdfPath& protoPath
) const {
    instRoutine.clearPrim(stage, protoPath);
    meshRoutine.clearPrim(stage, protoPath);
    sketchRoutine.clearPrim(stage, protoPath);
}

size_t TessellationRoutine::size() const {
    return meshRoutine.size() + sketchRoutine.size();
}