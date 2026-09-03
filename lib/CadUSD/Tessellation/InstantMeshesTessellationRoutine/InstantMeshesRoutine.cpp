#include <chrono>
#include <utility>
#include <algorithm>
#include <cmath>
#include <functional>
#include <initializer_list>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <string>
#include <vector>

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
#include <pxr/base/work/loops.h>
#include <pxr/base/work/workTBB/loops_impl.h>

#include <pxr/base/vt/array.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/gf/vec2f.h>
#include <pxr/usd/sdf/path.h>

#pragma pop_macro("Handle")

#include "CadUSD/Logger.h"
#include "CadUSD/Tessellation/TessellationRoutine.h"
#include "CadUSD/Tessellation/TessellationUtils.h"

class Geom_Surface;

PXR_NAMESPACE_USING_DIRECTIVE

using Clock = std::chrono::high_resolution_clock;
using Seconds = std::chrono::duration<double>;


bool InstantMeshesTessellationRoutine::tessellate(
    const TopoDS_Shape& defShape, 
    const TessParams& params,
    const SdfPath& protoPath
) {
    auto tessellateStart = Clock::now();

    LOG_DEBUG("  -> tessellatePart: Edge walk preparation");



    
    

    auto tessellateEnd = Clock::now();
    LOG_DEBUG("  Total tessellatePart time: " + std::to_string(Seconds(tessellateEnd - tessellateStart).count()) + " s");

    return true;
}