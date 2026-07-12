#pragma once

#include <ostream>
#include <iostream>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>

#include <TDF_Label.hxx>
#include <Quantity_Color.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_XYZ.hxx>
#include <TDF_Tool.hxx>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/pxr.h>
#include <pxr/usd/usd/common.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/modelAPI.h>
#include <pxr/usd/usd/inherits.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/references.h>
#include <pxr/usd/usd/variantSets.h>
#include <pxr/usd/usd/editContext.h>

#include <pxr/usd/sdf/changeBlock.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/variantSpec.h>
#include <pxr/usd/sdf/variantSetSpec.h>

#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdGeom/scope.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>
#include <pxr/usd/usdGeom/basisCurves.h>
#include <pxr/usd/usdGeom/imageable.h>
#include <pxr/usd/usdGeom/subset.h>
#include <pxr/usd/usdGeom/primvar.h>
#include <pxr/usd/usdGeom/xformOp.h>

#include <pxr/usd/sdf/copyUtils.h>
#include <pxr/base/work/loops.h>

#include <pxr/base/vt/array.h>
#include <pxr/base/vt/types.h>
#include <pxr/base/tf/staticData.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/gf/vec3f.h>

#pragma pop_macro("Handle")

PXR_NAMESPACE_USING_DIRECTIVE

struct CurvePiece {
    int sourceCurveIdx; // index into original counts/continuity array
    int pointOffset;    // offset into points array
    int pointCount;     // length of this piece (<= pointLimit)
};

// Split each original curve into pieces no longer than pointLimit.
// For oversized curves, each continuation piece repeats the previous
// piece's endpoint as its first point to avoid visual gaps.
std::vector<CurvePiece> splitCurvesIntoPieces(const VtIntArray& counts, uint64_t pointLimit);

struct CurveChunk {
    std::vector<CurvePiece> pieces;
    int pointOffset; // == pieces.front().pointOffset
    int pointCount;  // sum of piece pointCounts
};

// Group pieces into prim-sized chunks. If combine==false, each piece is its own chunk.
std::vector<CurveChunk> computeCurveChunks(const VtIntArray& counts, uint64_t pointLimit, bool combine);

VtIntArray chunkVertexCounts(const CurveChunk& chunk);

template <typename T>
VtArray<T> gatherChunkValues(const VtArray<T>& source, const CurveChunk& chunk) {
    std::vector<T> values;
    for (const auto& piece : chunk.pieces) {
        if (piece.pointCount <= 0 || piece.pointOffset < 0) {
            continue;
        }

        int end = piece.pointOffset + piece.pointCount;
        if (end > (int)source.size()) {
            continue;
        }

        values.insert(
            values.end(),
            source.begin() + piece.pointOffset,
            source.begin() + end
        );
    }
    return VtArray<T>(values.begin(), values.end());
}