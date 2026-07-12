#include <vector>
#include <algorithm>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/pxr.h>
#include <pxr/base/vt/array.h>
#include <pxr/base/vt/types.h>
#include <stddef.h>
#include <stdint.h>

#pragma pop_macro("Handle")

#include "StepUSD/Routine/CurveUtils.h"

PXR_NAMESPACE_USING_DIRECTIVE

// Split each original curve into pieces no longer than pointLimit.
// For oversized curves, each continuation piece repeats the previous
// piece's endpoint as its first point to avoid visual gaps.
std::vector<CurvePiece> splitCurvesIntoPieces(const VtIntArray& counts, uint64_t pointLimit) {
    std::vector<CurvePiece> pieces;
    int pointOffset = 0;

    if (pointLimit == 0) {
        return pieces;
    }

    for (size_t curveIdx = 0; curveIdx < counts.size(); ++curveIdx) {
        int curveCount = counts[curveIdx];
        if (curveCount <= 0) {
            continue;
        }

        if ((uint64_t)curveCount <= pointLimit || pointLimit == 1) {
            int remaining = curveCount;
            int offset = pointOffset;
            while (remaining > 0) {
                int take = (int)std::min((uint64_t)remaining, pointLimit);
                pieces.push_back({(int)curveIdx, offset, take});
                offset += take;
                remaining -= take;
            }
            pointOffset += curveCount;
            continue;
        }

        int consumed = 0;

        // First piece uses up to pointLimit unique points.
        int firstCount = (int)std::min((uint64_t)curveCount, pointLimit);
        pieces.push_back({(int)curveIdx, pointOffset, firstCount});
        consumed += firstCount;

        // Remaining pieces repeat the previous endpoint, then add up to
        // (pointLimit - 1) new points.
        while (consumed < curveCount) {
            int remainingUnique = curveCount - consumed;
            int takeUnique = (int)std::min((uint64_t)remainingUnique, pointLimit - 1);

            int pieceStart = pointOffset + consumed - 1;
            int pieceCount = 1 + takeUnique;
            pieces.push_back({(int)curveIdx, pieceStart, pieceCount});

            consumed += takeUnique;
        }

        pointOffset += curveCount;
    }
    return pieces;
}

// Group pieces into prim-sized chunks. If combine==false, each piece is its own chunk.
std::vector<CurveChunk> computeCurveChunks(const VtIntArray& counts, uint64_t pointLimit, bool combine) {
    std::vector<CurvePiece> pieces = splitCurvesIntoPieces(counts, pointLimit);
    std::vector<CurveChunk> chunks;

    if (!combine) {
        for (const auto& piece : pieces) {
            chunks.push_back({{piece}, piece.pointOffset, piece.pointCount});
        }
        return chunks;
    }

    size_t i = 0;
    while (i < pieces.size()) {
        CurveChunk chunk;
        chunk.pointOffset = pieces[i].pointOffset;
        chunk.pointCount = 0;
        while (i < pieces.size()) {
            int c = pieces[i].pointCount;
            if (!chunk.pieces.empty() && (uint64_t)(chunk.pointCount + c) > pointLimit) break;
            chunk.pieces.push_back(pieces[i]);
            chunk.pointCount += c;
            i++;
        }
        chunks.push_back(chunk);
    }
    return chunks;
}

VtIntArray chunkVertexCounts(const CurveChunk& chunk) {
    VtIntArray counts(chunk.pieces.size());
    for (size_t j = 0; j < chunk.pieces.size(); ++j) counts[j] = chunk.pieces[j].pointCount;
    return counts;
}
