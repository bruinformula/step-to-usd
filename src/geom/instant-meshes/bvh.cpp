
/*
    bvh.cpp -- bounding volume hierarchy for fast ray-intersection queries

    This file is part of the implementation of

        Instant Field-Aligned Meshes
        Wenzel Jakob, Daniele Panozzo, Marco Tarini, and Olga Sorkine-Hornung
        In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2015)

    All rights reserved. Use of this source code is governed by a
    BSD-style license that can be found in the LICENSE.txt file.
*/

#include <cassert>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/parallel_invoke.h>
#include "bvh.h"

// ---------------------------------------------------------------------------
// Bin structure — unchanged
// ---------------------------------------------------------------------------

struct Bins {
    static const int BIN_COUNT = 8;
    Bins() { memset(counts, 0, sizeof(uint32_t) * BIN_COUNT); }
    uint32_t counts[BIN_COUNT];
    AABB bounds[BIN_COUNT];
};

// ---------------------------------------------------------------------------
// Serial recursive BVH builder
// Replaces BVHBuildTask::execute_serially. Identical logic, now a free
// function so it can be called from both the serial and parallel paths.
// ---------------------------------------------------------------------------

static void build_serially(BVH &bvh, uint32_t node_idx,
                            uint32_t *start, uint32_t *end,
                            uint32_t *temp) {
    uint32_t size = end - start;
    BVHNode &node = bvh.mNodes[node_idx];
    const MatrixXu &F = *bvh.mF;
    const MatrixXf &V = *bvh.mV;
    bool pointcloud = F.size() == 0;

    Float   best_cost  = BVH::T_tri * size;
    int64_t best_index = -1, best_axis = -1;
    float  *left_areas = (float *)temp;

    for (int axis = 0; axis < 3; ++axis) {
        if (pointcloud) {
            std::sort(start, end, [&](uint32_t f1, uint32_t f2) {
                return V(axis, f1) < V(axis, f2);
            });
        } else {
            std::sort(start, end, [&](uint32_t f1, uint32_t f2) {
                return (V(axis, F(0,f1)) + V(axis, F(1,f1)) + V(axis, F(2,f1))) <
                       (V(axis, F(0,f2)) + V(axis, F(1,f2)) + V(axis, F(2,f2)));
            });
        }

        AABB aabb;
        for (uint32_t i = 0; i < size; ++i) {
            uint32_t f = *(start + i);
            if (pointcloud) {
                aabb.expandBy(V.col(f));
            } else {
                aabb.expandBy(V.col(F(0, f)));
                aabb.expandBy(V.col(F(1, f)));
                aabb.expandBy(V.col(F(2, f)));
            }
            left_areas[i] = (float)aabb.surfaceArea();
        }
        if (axis == 0)
            node.aabb = aabb;

        aabb.clear();
        Float tri_factor = BVH::T_tri / node.aabb.surfaceArea();

        for (uint32_t i = size - 1; i >= 1; --i) {
            uint32_t f = *(start + i);
            if (pointcloud) {
                aabb.expandBy(V.col(f));
            } else {
                aabb.expandBy(V.col(F(0, f)));
                aabb.expandBy(V.col(F(1, f)));
                aabb.expandBy(V.col(F(2, f)));
            }
            Float sah_cost = 2.0f * BVH::T_aabb +
                tri_factor * (i * left_areas[i-1] + (size-i) * (float)aabb.surfaceArea());
            if (sah_cost < best_cost) {
                best_cost  = sah_cost;
                best_index = i;
                best_axis  = axis;
            }
        }
    }

    if (best_index == -1) {
        node.leaf.flag  = 1;
        node.leaf.start = start - bvh.mIndices;
        node.leaf.size  = size;
        return;
    }

    if (pointcloud) {
        std::sort(start, end, [&](uint32_t f1, uint32_t f2) {
            return V(best_axis, f1) < V(best_axis, f2);
        });
    } else {
        std::sort(start, end, [&](uint32_t f1, uint32_t f2) {
            return (V(best_axis, F(0,f1)) + V(best_axis, F(1,f1)) + V(best_axis, F(2,f1))) <
                   (V(best_axis, F(0,f2)) + V(best_axis, F(1,f2)) + V(best_axis, F(2,f2)));
        });
    }

    uint32_t left_count    = (uint32_t)best_index;
    int      node_idx_left  = node_idx + 1;
    int      node_idx_right = node_idx + 2 * left_count;
    node.inner.rightChild = node_idx_right;
    node.inner.unused = 0;

    build_serially(bvh, node_idx_left,  start,              start + left_count, temp);
    build_serially(bvh, node_idx_right, start + left_count, end,                temp + left_count);
}

// ---------------------------------------------------------------------------
// Parallel recursive BVH builder  (replaces BVHBuildTask)
//
// Three-tier strategy:
//   >= PARALLEL_THRESHOLD  parallel binning + parallel partition +
//                          tbb::parallel_invoke for children
//   >= SERIAL_THRESHOLD    parallel binning + parallel partition +
//                          serial child recursion (avoids spawn overhead)
//   <  SERIAL_THRESHOLD    fall straight to build_serially
//
// The left and right halves of both the index buffer (start[]) and the
// scratch buffer (temp[]) are always disjoint, so parallel_invoke is safe.
// ---------------------------------------------------------------------------

static constexpr uint32_t SERIAL_THRESHOLD   = 32;
static constexpr uint32_t PARALLEL_THRESHOLD = 1024;

static void build_recursive(BVH &bvh, uint32_t node_idx,
                             uint32_t *start, uint32_t *end,
                             uint32_t *temp,
                             uint32_t  total_size) {
    const MatrixXu &F = *bvh.mF;
    const MatrixXf &V = *bvh.mV;
    bool     pointcloud = F.size() == 0;
    uint32_t size       = end - start;
    BVHNode &node       = bvh.mNodes[node_idx];

    // ---- Leaf: go fully serial and report progress -------------------------
    if (size < SERIAL_THRESHOLD) {
        tbb::blocked_range<uint32_t> range(
            (uint32_t)(start - bvh.mIndices),
            (uint32_t)(end   - bvh.mIndices));
        const ProgressCallback &progress = bvh.mProgress;
        SHOW_PROGRESS_RANGE(range, total_size,
                            "Constructing Bounding Volume Hierarchy");
        build_serially(bvh, node_idx, start, end, temp);
        return;
    }

    // ---- Parallel bin counting (SAH) ---------------------------------------
    int   axis         = node.aabb.largestAxis();
    Float mn           = node.aabb.min[axis];
    Float mx           = node.aabb.max[axis];
    Float inv_bin_size = Bins::BIN_COUNT / (mx - mn);

    Bins bins = tbb::parallel_reduce(
        tbb::blocked_range<uint32_t>(0u, size, GRAIN_SIZE),
        Bins(),
        [&](const tbb::blocked_range<uint32_t> &range, Bins result) {
            for (uint32_t i = range.begin(); i != range.end(); ++i) {
                uint32_t f = start[i];
                Float centroid = pointcloud
                    ? V(axis, f)
                    : (1.0f/3.0f)*(V(axis,F(0,f))+V(axis,F(1,f))+V(axis,F(2,f)));
                int idx = std::min(std::max(
                    (int)((centroid - mn) * inv_bin_size), 0),
                    Bins::BIN_COUNT - 1);
                result.counts[idx]++;
                AABB &bb = result.bounds[idx];
                if (!pointcloud) {
                    bb.expandBy(V.col(F(0,f)));
                    bb.expandBy(V.col(F(1,f)));
                    bb.expandBy(V.col(F(2,f)));
                } else {
                    bb.expandBy(V.col(f));
                }
            }
            return result;
        },
        [](const Bins &b1, const Bins &b2) {
            Bins r;
            for (int i = 0; i < Bins::BIN_COUNT; ++i) {
                r.counts[i] = b1.counts[i] + b2.counts[i];
                r.bounds[i] = AABB::merge(b1.bounds[i], b2.bounds[i]);
            }
            return r;
        }
    );

    // ---- SAH cost sweep ----------------------------------------------------
    AABB bounds_left[Bins::BIN_COUNT];
    bounds_left[0] = bins.bounds[0];
    for (int i = 1; i < Bins::BIN_COUNT; ++i) {
        bins.counts[i] += bins.counts[i-1];            // prefix sum
        bounds_left[i]  = AABB::merge(bounds_left[i-1], bins.bounds[i]);
    }

    AABB    bounds_right = bins.bounds[Bins::BIN_COUNT-1];
    AABB    best_bounds_right;
    int64_t best_index = -1;
    Float   best_cost  = BVH::T_tri * size;
    Float   tri_factor = BVH::T_tri / node.aabb.surfaceArea();

    for (int i = Bins::BIN_COUNT - 2; i >= 0; --i) {
        uint32_t nl = bins.counts[i], nr = size - bins.counts[i];
        Float cost  = 2.0f * BVH::T_aabb +
            tri_factor * (nl * bounds_left[i].surfaceArea() +
                          nr * bounds_right.surfaceArea());
        if (cost < best_cost) {
            best_cost         = cost;
            best_index        = i;
            best_bounds_right = bounds_right;
        }
        bounds_right = AABB::merge(bounds_right, bins.bounds[i]);
    }

    if (best_index == -1) {
        // No beneficial parallel split — fall back to careful serial builder
        build_serially(bvh, node_idx, start, end, temp);
        return;
    }

    // ---- Parallel two-pass partition ---------------------------------------
    uint32_t left_count    = bins.counts[best_index];
    int      node_idx_left  = node_idx + 1;
    int      node_idx_right = node_idx + 2 * left_count;

    bvh.mNodes[node_idx_left ].aabb = bounds_left[best_index];
    bvh.mNodes[node_idx_right].aabb = best_bounds_right;
    node.inner.rightChild = node_idx_right;
    node.inner.unused     = 0;

    std::atomic<uint32_t> offset_left(0), offset_right(left_count);

    tbb::parallel_for(
        tbb::blocked_range<uint32_t>(0u, size, GRAIN_SIZE),
        [&](const tbb::blocked_range<uint32_t> &range) {
            uint32_t cl = 0, cr = 0;
            for (uint32_t i = range.begin(); i != range.end(); ++i) {
                uint32_t f = start[i];
                Float centroid = pointcloud
                    ? V(axis, f)
                    : (1.0f/3.0f)*(V(axis,F(0,f))+V(axis,F(1,f))+V(axis,F(2,f)));
                ((int)((centroid-mn)*inv_bin_size) <= best_index ? cl : cr)++;
            }
            uint32_t il = offset_left .fetch_add(cl);
            uint32_t ir = offset_right.fetch_add(cr);
            for (uint32_t i = range.begin(); i != range.end(); ++i) {
                uint32_t f = start[i];
                Float centroid = pointcloud
                    ? V(axis, f)
                    : (1.0f/3.0f)*(V(axis,F(0,f))+V(axis,F(1,f))+V(axis,F(2,f)));
                if ((int)((centroid-mn)*inv_bin_size) <= best_index)
                    temp[il++] = f;
                else
                    temp[ir++] = f;
            }
        }
    );
    memcpy(start, temp, size * sizeof(uint32_t));
    assert(offset_left == left_count && offset_right == size);

    // ---- Recurse -----------------------------------------------------------
    // Left half:  start[0 .. left_count-1] / temp[0 .. left_count-1]
    // Right half: start[left_count .. size-1] / temp[left_count .. size-1]
    // The two halves are always disjoint so parallel_invoke is race-free.
    if (size >= PARALLEL_THRESHOLD) {
        tbb::parallel_invoke(
            [&]{ build_recursive(bvh, node_idx_left,
                                 start,              start + left_count,
                                 temp,               total_size); },
            [&]{ build_recursive(bvh, node_idx_right,
                                 start + left_count, end,
                                 temp  + left_count, total_size); }
        );
    } else {
        build_recursive(bvh, node_idx_left,
                        start,              start + left_count,
                        temp,               total_size);
        build_recursive(bvh, node_idx_right,
                        start + left_count, end,
                        temp  + left_count, total_size);
    }
}

// ---------------------------------------------------------------------------
// BVH::build  (public, unchanged signature)
// ---------------------------------------------------------------------------

BVH::BVH(
    const MatrixXu *F, 
    const MatrixXf *V, 
    const MatrixXf *N, 
    const AABB &aabb
) : 
    mIndices(nullptr), 
    mF(F), 
    mV(V), 
    mN(N), 
    mDiskRadius(0.f) 
{
    if (mF->size() > 0) {
        mNodes.resize(2*mF->cols());
        memset(mNodes.data(), 0, sizeof(BVHNode) * mNodes.size());
        mNodes[0].aabb = aabb;
        mIndices = new uint32_t[mF->cols()];
    } else if (mV->size() > 0) {
        mNodes.resize(2*mV->cols());
        memset(mNodes.data(), 0, sizeof(BVHNode) * mNodes.size());
        mNodes[0].aabb = aabb;
        mIndices = new uint32_t[mV->cols()];
    }
}
 
void BVH::build(const ProgressCallback &progress) {
    if (mF->cols() == 0 && mV->cols() == 0)
        return;
    mProgress = progress;

#if defined(SINGLE_PRECISION)
    if (sizeof(BVHNode) != 32)
        throw std::runtime_error(
            "BVH Node is not packed! Investigate compiler settings.");
#endif

    cout << "Constructing Bounding Volume Hierarchy .. ";
    cout.flush();

    bool     pointcloud = mF->size() == 0;
    uint32_t total_size = pointcloud ? mV->cols() : mF->cols();

    for (uint32_t i = 0; i < total_size; ++i)
        mIndices[i] = i;

    Timer<> timer;
    uint32_t *temp = new uint32_t[total_size];
    build_recursive(*this, 0u, mIndices, mIndices + total_size, temp, total_size);
    delete[] temp;

    // ---- Compress sparse node storage --------------------------------------
    std::pair<Float, uint32_t> stats = statistics();
    cout << "done. ("
         << "SAH cost = " << stats.first  << ", "
         << "nodes = "    << stats.second << ", "
         << "took "       << timeString(timer.reset()) << ")" << endl;

    cout.precision(4);
    cout << "Compressing BVH node storage to "
         << 100 * stats.second / (float)mNodes.size()
         << "% of its original size .. ";
    cout.flush();

    std::vector<BVHNode>  compressed(stats.second);
    std::vector<uint32_t> skipped_accum(mNodes.size());

    for (int64_t i = (int64_t)stats.second - 1,
                 j = (int64_t)mNodes.size(),
                 skipped = 0;
         i >= 0; --i) {
        while (mNodes[--j].isUnused())
            skipped++;
        BVHNode &nn = compressed[i];
        nn = mNodes[j];
        skipped_accum[j] = skipped;
        if (nn.isInner()) {
            nn.inner.rightChild =
                i + nn.inner.rightChild - j -
                (skipped - skipped_accum[nn.inner.rightChild]);
        }
    }
    mNodes = std::move(compressed);
    cout << "done. (took " << timeString(timer.value()) << ")" << endl;

    // Point-cloud: assign disk radius 
    if (pointcloud) {
        cout << "Assigning disk radius .. ";
        cout.flush();

        tbb::blocked_range<uint32_t> range(0u, (uint32_t)mV->cols(), GRAIN_SIZE);
        double total = tbb::parallel_deterministic_reduce(
            range,
            0.0,
            [&](const tbb::blocked_range<uint32_t> &r, double sum) -> double {
                for (uint32_t i = r.begin(); i < r.end(); ++i) {
                    Float radius = std::numeric_limits<Float>::infinity();
                    if (findNearest(mV->col(i), radius) != (uint32_t)-1)
                        sum += (double)radius;
                }
                SHOW_PROGRESS_RANGE(r, mV->cols(), "Assigning disk radius");
                return sum;
            },
            [](double a, double b) { return a + b; }
        );
        mDiskRadius = total / (double)range.size() * 3.0;
        refitBoundingBoxes();
        cout << "done. (took " << timeString(timer.value()) << ")" << endl;
    }

    mProgress = nullptr;
}

bool BVH::rayIntersect(Ray ray, uint32_t &idx, Float &t, Vector2f *uv) const {
    if (mNodes.empty())
        return false;

    uint32_t node_idx = 0, stack[64];
    uint32_t stack_idx = 0;
    bool hit = false;
    t = std::numeric_limits<Float>::infinity();

    if (mF->size() > 0) {
        while (true) {
            const BVHNode &node = mNodes[node_idx];

            if (!node.aabb.rayIntersect(ray)) {
                if (stack_idx == 0)
                    break;
                node_idx = stack[--stack_idx];
                continue;
            }

            if (node.isInner()) {
                stack[stack_idx++] = node.inner.rightChild;
                node_idx++;
                assert(stack_idx<64);
            } else {
                Float _t;
                Vector2f _uv;
                for (uint32_t i = node.start(), end = node.end(); i < end; ++i) {
                    if (rayIntersectTri(ray, mIndices[i], _t, _uv)) {
                        idx = mIndices[i];
                        t = ray.maxt = _t;
                        hit = true;
                        if (uv)
                            *uv = _uv;
                    }
                }
                if (stack_idx == 0)
                    break;
                node_idx = stack[--stack_idx];
                continue;
            }
        }
    } else {
        if (uv)
            *uv = Vector2f::Zero();
        while (true) {
            const BVHNode &node = mNodes[node_idx];

            if (!node.aabb.rayIntersect(ray)) {
                if (stack_idx == 0)
                    break;
                node_idx = stack[--stack_idx];
                continue;
            }

            if (node.isInner()) {
                stack[stack_idx++] = node.inner.rightChild;
                node_idx++;
                assert(stack_idx<64);
            } else {
                Float _t;
                for (uint32_t i = node.start(), end = node.end(); i < end; ++i) {
                    if (rayIntersectDisk(ray, mIndices[i], _t)) {
                        idx = mIndices[i];
                        t = ray.maxt = _t;
                        hit = true;
                    }
                }
                if (stack_idx == 0)
                    break;
                node_idx = stack[--stack_idx];
                continue;
            }
        }
    }

    return hit;
}

bool BVH::rayIntersect(Ray ray) const {
    if (mNodes.empty())
        return false;

    uint32_t node_idx = 0, stack[64];
    uint32_t stack_idx = 0;

    if (mF->size() > 0) {
        while (true) {
            const BVHNode &node = mNodes[node_idx];

            if (!node.aabb.rayIntersect(ray)) {
                if (stack_idx == 0)
                    break;
                node_idx = stack[--stack_idx];
                continue;
            }

            if (node.isInner()) {
                stack[stack_idx++] = node.inner.rightChild;
                node_idx++;
                assert(stack_idx<64);
            } else {
                Float t;
                Vector2f uv;
                for (uint32_t i = node.start(), end = node.end(); i < end; ++i)
                    if (rayIntersectTri(ray, mIndices[i], t, uv))
                        return true;
                if (stack_idx == 0)
                    break;
                node_idx = stack[--stack_idx];
                continue;
            }
        }
    } else {
        while (true) {
            const BVHNode &node = mNodes[node_idx];

            if (!node.aabb.rayIntersect(ray)) {
                if (stack_idx == 0)
                    break;
                node_idx = stack[--stack_idx];
                continue;
            }

            if (node.isInner()) {
                stack[stack_idx++] = node.inner.rightChild;
                node_idx++;
                assert(stack_idx<64);
            } else {
                Float t;
                for (uint32_t i = node.start(), end = node.end(); i < end; ++i)
                    if (rayIntersectDisk(ray, mIndices[i], t))
                        return true;
                if (stack_idx == 0)
                    break;
                node_idx = stack[--stack_idx];
                continue;
            }
        }
    }

    return false;
}

void BVH::findNearestWithRadius(
    const Vector3f &p, 
    Float radius,
    std::vector<uint32_t> &result,
    bool includeSelf
) const {
    result.clear();

    uint32_t node_idx = 0, stack[64];
    uint32_t stack_idx = 0;
    Float radius2 = radius*radius;

    while (true) {
        const BVHNode &node = mNodes[node_idx];
        if (node.aabb.squaredDistanceTo(p) > radius2) {
            if (stack_idx == 0)
                break;
            node_idx = stack[--stack_idx];
            continue;
        }

        if (node.isInner()) {
            stack[stack_idx++] = node.inner.rightChild;
            node_idx++;
            assert(stack_idx<64);
        } else {
            uint32_t start = node.leaf.start, end = start + node.leaf.size;
            for (uint32_t i = start; i < end; ++i) {
                uint32_t f = mIndices[i];
                Vector3f pointPos = Vector3f::Zero();
                if (mF->size() > 0) {
                    for (int j=0; j<3; ++j)
                        pointPos += mV->col((*mF)(j, f));
                    pointPos *= 1.0f / 3.0f;
                } else {
                    pointPos = mV->col(f);
                }
                Float pointDist2 = (pointPos-p).squaredNorm();
                if (pointDist2 < radius2 && (pointDist2 != 0 || includeSelf))
                    result.push_back(f);
            }
            if (stack_idx == 0)
                break;
            node_idx = stack[--stack_idx];
            continue;
        }
    }
}

uint32_t BVH::findNearest(
    const Vector3f &p, 
    Float &radius, 
    bool includeSelf
) const {
    uint32_t node_idx = 0, stack[64];
    uint32_t stack_idx = 0;
    Float radius2 = radius*radius;
    uint32_t result = (uint32_t) -1;

    while (true) {
        const BVHNode &node = mNodes[node_idx];
        if (node.aabb.squaredDistanceTo(p) > radius2) {
            if (stack_idx == 0)
                break;
            node_idx = stack[--stack_idx];
            continue;
        }

        if (node.isInner()) {
            uint32_t left = node_idx + 1, right = node.inner.rightChild;
            Float distLeft = mNodes[left].aabb.squaredDistanceTo(p);
            Float distRight = mNodes[right].aabb.squaredDistanceTo(p);
            if (distLeft < distRight) {
                node_idx = left;
                if (distRight < radius2)
                    stack[stack_idx++] = right;
            } else {
                node_idx = right;
                if (distLeft < radius2)
                    stack[stack_idx++] = left;
            }
            assert(stack_idx<64);
        } else {
            uint32_t start = node.leaf.start, end = start + node.leaf.size;
            for (uint32_t i = start; i < end; ++i) {
                uint32_t f = mIndices[i];
                Vector3f pointPos = Vector3f::Zero();
                if (mF->size() > 0) {
                    for (int j=0; j<3; ++j)
                        pointPos += mV->col((*mF)(j, f));
                    pointPos *= 1.0f / 3.0f;
                } else {
                    pointPos = mV->col(f);
                }
                Float pointDist2 = (pointPos-p).squaredNorm();

                if (pointDist2 < radius2 && (pointDist2 != 0 || includeSelf)) {
                    radius2 = pointDist2;
                    result = f;
                }
            }
            if (stack_idx == 0)
                break;
            node_idx = stack[--stack_idx];
            continue;
        }
    }
    radius = std::sqrt(radius2);
    return result;
}

void BVH::findKNearest(
    const Vector3f &p, 
    uint32_t k, 
    Float &radius,
    std::vector<std::pair<Float, uint32_t>> &result,
    bool includeSelf
) const {
    result.clear();

    uint32_t node_idx = 0, stack[64];
    uint32_t stack_idx = 0;
    Float radius2 = radius*radius;
    bool isHeap = false;
    auto comp = [](const std::pair<Float, uint32_t> &v1, const std::pair<Float, uint32_t> &v2) {
        return v1.first < v2.first;
    };

    while (true) {
        const BVHNode &node = mNodes[node_idx];
        if (node.aabb.squaredDistanceTo(p) > radius2) {
            if (stack_idx == 0)
                break;
            node_idx = stack[--stack_idx];
            continue;
        }

        if (node.isInner()) {
            uint32_t left = node_idx + 1, right = node.inner.rightChild;
            Float distLeft = mNodes[left].aabb.squaredDistanceTo(p);
            Float distRight = mNodes[right].aabb.squaredDistanceTo(p);
            if (distLeft < distRight) {
                node_idx = left;
                if (distRight < radius2)
                    stack[stack_idx++] = right;
            } else {
                node_idx = right;
                if (distLeft < radius2)
                    stack[stack_idx++] = left;
            }
            assert(stack_idx<64);
        } else {
            uint32_t start = node.leaf.start, end = start + node.leaf.size;
            for (uint32_t i = start; i < end; ++i) {
                uint32_t f = mIndices[i];
                Vector3f pointPos = Vector3f::Zero();
                if (mF->size() > 0) {
                    for (int j=0; j<3; ++j)
                        pointPos += mV->col((*mF)(j, f));
                    pointPos *= 1.0f / 3.0f;
                } else {
                    pointPos = mV->col(f);
                }
                Float pointDist2 = (pointPos-p).squaredNorm();

                if (pointDist2 < radius2 && (pointDist2 != 0 || includeSelf)) {
                    if (result.size() < k) {
                        result.push_back(std::make_pair(pointDist2, f));
                    } else {
                        if (!isHeap) {
                            /* Establish the max-heap property */
                            std::make_heap(result.begin(), result.end(), comp);
                            isHeap = true;
                        }

                        result.push_back(std::make_pair(pointDist2, f));
                        std::push_heap(result.begin(), result.end(), comp);
                        std::pop_heap(result.begin(), result.end(), comp);
                        result.pop_back();

                        /* Reduce the search radius accordingly */
                        radius2 = result[0].first;
                    }
                }
            }
            if (stack_idx == 0)
                break;
            node_idx = stack[--stack_idx];
            continue;
        }
    }
    radius = std::sqrt(radius2);
}

void BVH::findKNearest(
    const Vector3f &p, const Vector3f &n, uint32_t k,
    Float &radius,
    std::vector<std::pair<Float, uint32_t> > &result,
    Float angleThresh, bool includeSelf
) const {
    result.clear();

    uint32_t node_idx = 0, stack[64];
    uint32_t stack_idx = 0;
    Float radius2 = radius*radius;
    bool isHeap = false;
    angleThresh = std::cos(angleThresh * M_PI/180);
    auto comp = [](const std::pair<Float, uint32_t> &v1, const std::pair<Float, uint32_t> &v2) {
        return v1.first < v2.first;
    };

    while (true) {
        const BVHNode &node = mNodes[node_idx];
        if (node.aabb.squaredDistanceTo(p) > radius2) {
            if (stack_idx == 0)
                break;
            node_idx = stack[--stack_idx];
            continue;
        }

        if (node.isInner()) {
            uint32_t left = node_idx + 1, right = node.inner.rightChild;
            Float distLeft = mNodes[left].aabb.squaredDistanceTo(p);
            Float distRight = mNodes[right].aabb.squaredDistanceTo(p);
            if (distLeft < distRight) {
                node_idx = left;
                if (distRight < radius2)
                    stack[stack_idx++] = right;
            } else {
                node_idx = right;
                if (distLeft < radius2)
                    stack[stack_idx++] = left;
            }
            assert(stack_idx<64);
        } else {
            uint32_t start = node.leaf.start, end = start + node.leaf.size;
            for (uint32_t i = start; i < end; ++i) {
                uint32_t f = mIndices[i];
                Vector3f pointPos = Vector3f::Zero();
                if (mF->size() > 0) {
                    for (int j=0; j<3; ++j)
                        pointPos += mV->col((*mF)(j, f));
                    pointPos *= 1.0f / 3.0f;
                } else {
                    pointPos = mV->col(f);
                }
                Vector3f pointNormal = Vector3f::Zero();
                if (mF->size() > 0) {
                    for (int j=0; j<3; ++j)
                        pointNormal += mN->col((*mF)(j, f));
                } else {
                    pointNormal = mN->col(f);
                }
                Float pointDist2 = (pointPos-p).squaredNorm();

                if (pointDist2 < radius2 && (pointDist2 != 0 || includeSelf) && pointNormal.dot(n) > angleThresh) {
                    if (result.size() < k) {
                        result.push_back(std::make_pair(pointDist2, f));
                    } else {
                        if (!isHeap) {
                            /* Establish the max-heap property */
                            std::make_heap(result.begin(), result.end(), comp);
                            isHeap = true;
                        }

                        result.push_back(std::make_pair(pointDist2, f));
                        std::push_heap(result.begin(), result.end(), comp);
                        std::pop_heap(result.begin(), result.end(), comp);
                        result.pop_back();

                        /* Reduce the search radius accordingly */
                        radius2 = result[0].first;
                    }
                }
            }
            if (stack_idx == 0)
                break;
            node_idx = stack[--stack_idx];
            continue;
        }
    }
    radius = std::sqrt(radius2);
}

bool BVH::rayIntersectTri(const Ray &ray, uint32_t i, Float &t, Vector2f &uv) const {
    const Vector3f &p0 = mV->col((*mF)(0, i)),
                   &p1 = mV->col((*mF)(1, i)),
                   &p2 = mV->col((*mF)(2, i));

    Vector3f edge1 = p1 - p0, edge2 = p2 - p0;
    Vector3f pvec = ray.d.cross(edge2);

    Float det = edge1.dot(pvec);
    if (det == 0.0f)
        return false;
    Float inv_det = 1.0f / det;

    Vector3f tvec = ray.o - p0;
    Float u = tvec.dot(pvec) * inv_det;
    if (u < 0.0f || u > 1.0f)
        return false;

    Vector3f qvec = tvec.cross(edge1);
    Float v = ray.d.dot(qvec) * inv_det;

    if (v < 0.0f || u + v > 1.0f)
        return false;

    Float tempT = edge2.dot(qvec) * inv_det;
    if (tempT < ray.mint || tempT > ray.maxt)
        return false;

    t = tempT;
    uv << u, v;
    return true;
}

bool BVH::rayIntersectDisk(const Ray &ray, uint32_t i, Float &t) const {
    Vector3f v = mV->col(i), n = mN->col(i);
    Float dp = ray.d.dot(n);

    if (std::abs(dp) < RCPOVERFLOW)
        return false;

    t = (n.dot(v) - n.dot(ray.o)) / dp;

    return (ray(t)-v).squaredNorm() < mDiskRadius*mDiskRadius;
}

void BVH::printStatistics() const {
    cout << endl;
    cout << "Bounding Volume Hierarchy statistics:" << endl;
    cout << "    Tree nodes         : " << memString(sizeof(BVHNode) * mNodes.size()) << endl;
    cout << "    Index buffer       : " << memString(sizeof(uint32_t) * mF->size()) << endl;
    cout << "    Total              : "
         << memString(sizeof(BVHNode) * mNodes.size() + sizeof(uint32_t) * mF->size()) << endl;
}

std::pair<Float, uint32_t> BVH::statistics(uint32_t node_idx) const {
    const BVHNode &node = mNodes[node_idx];
    if (node.isLeaf()) {
        return std::make_pair(T_tri * node.leaf.size, 1u);
    } else {
        std::pair<Float, uint32_t> stats_left = statistics(node_idx + 1u);
        std::pair<Float, uint32_t> stats_right = statistics(node.inner.rightChild);
        Float saLeft = mNodes[node_idx + 1u].aabb.surfaceArea();
        Float saRight = mNodes[node.inner.rightChild].aabb.surfaceArea();
        Float saCur = node.aabb.surfaceArea();
        Float sahCost = 2 * BVH::T_aabb + (saLeft * stats_left.first +
                                           saRight * stats_right.first) / saCur;

        return std::make_pair(
            sahCost,
            stats_left.second + stats_right.second + 1u
        );
    }
}

BVH::~BVH() {
    delete[] mIndices;
}

void BVH::refitBoundingBoxes(uint32_t node_idx) {
    BVHNode &node = mNodes[node_idx];
    if (node.isLeaf()) {
        for (uint32_t i=node.start(); i<node.end(); ++i) {
            uint32_t j = mIndices[i];
            const Vector3f &p = mV->col(j), &n = mN->col(j);
            Vector3f s, t;
            coordinate_system(n, s, t);
            AABB aabb;
            for (int k=0; k<4; ++k)
                aabb.expandBy(p + mDiskRadius *  (
                    ((k&1)*2 - 1) * s +
                    ((k&2) - 1) * t));
            node.aabb = aabb;
        }
    } else {
        uint32_t left = node_idx + 1u, right = node.inner.rightChild;
        refitBoundingBoxes(left);
        refitBoundingBoxes(right);
        node.aabb = AABB::merge(mNodes[left].aabb, mNodes[right].aabb);
    }
}