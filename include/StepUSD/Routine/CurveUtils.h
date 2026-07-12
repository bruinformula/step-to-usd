#pragma once

#include <vector>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/pxr.h>
#include <pxr/base/vt/array.h>
#include <pxr/base/vt/types.h>

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