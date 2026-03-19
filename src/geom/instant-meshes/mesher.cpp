/*
    mesher.cpp: Bare-minimum meshing pipeline extracted from Instant Meshes

    Original work:
        Instant Field-Aligned Meshes
        Wenzel Jakob, Daniele Panozzo, Marco Tarini, and Olga Sorkine-Hornung
        In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2015)

    All rights reserved. Use of this source code is governed by a
    BSD-style license that can be found in the LICENSE.txt file.
*/

#include "mesher.h"
#include "meshio.h"
#include "dedge.h"
#include "normal.h"
#include "extract.h"
#include "subdivide.h"
#include "adjacency.h"

#include <iostream>
#include <stdexcept>
#include <thread>
#include <chrono>

// Construction
Mesher::Mesher(bool deterministic)
    : mDeterministic(deterministic)
    , mOptimizer(mRes, false)
    , mBVH(nullptr)
    , mCreaseAngle(-1.0f)
    , mSmoothIterations(0)
    , mPureQuad(false)
    , mAlignToBoundaries(false)
    , mProgress(nullptr)
{
}

Mesher::~Mesher() {
    delete mBVH;
    mOptimizer.shutdown();
    mRes.free();
}

// Configuration
void Mesher::setSymmetry(int rosy, int posy) {
    if (!((rosy == 6 && posy == 3) ||
          (rosy == 2 && posy == 4) ||
          (rosy == 4 && posy == 4)))
        throw std::runtime_error("Unsupported RoSy/PoSy combination");

    mOptimizer.setRoSy(rosy);
    mOptimizer.setPoSy(posy);
}

void Mesher::setExtrinsic(bool extrinsic) {
    mOptimizer.setExtrinsic(extrinsic);
}

void Mesher::setTargetScale(Float scale) {
    if (!mRes.levels())
        return;

    int posy = mOptimizer.posy();
    Float face_area = (posy == 4)
        ? (scale * scale)
        : (std::sqrt(3.f) / 4.f * scale * scale);

    uint32_t face_count   = (uint32_t)(mMeshStats.mSurfaceArea / face_area);
    uint32_t vertex_count = (posy == 4) ? face_count : (face_count / 2);
    (void)vertex_count;

    std::lock_guard<ordered_lock> lock(mRes.mutex());
    mRes.setScale(scale);
}

void Mesher::setTargetVertexCount(uint32_t v) {
    if (!mRes.levels())
        return;

    int   posy       = mOptimizer.posy();
    int   face_count = (posy == 4) ? (int)v : (int)(v * 2);
    Float face_area  = mMeshStats.mSurfaceArea / face_count;
    Float scale      = (posy == 4)
        ? std::sqrt(face_area)
        : (2.f * std::sqrt(face_area * std::sqrt(1.f / 3.f)));

    std::lock_guard<ordered_lock> lock(mRes.mutex());
    mRes.setScale(scale);
}

// Internal
void Mesher::buildHierarchy(MatrixXu &F, MatrixXf &V, MatrixXf &N, Float scale, int rosy, int posy) {
    bool pointcloud = (F.size() == 0);

    VectorXf A;
    AdjacencyMatrix adj = nullptr;

    if (!pointcloud) {
        VectorXu V2E, E2E;

        // Subdivide if the mesh is too coarse for the desired edge length
        if (mMeshStats.mMaximumEdgeLength * 2 > scale ||
            mMeshStats.mMaximumEdgeLength > mMeshStats.mAverageEdgeLength * 2) {

            std::cout << "Input mesh is too coarse (max edge length="
                      << mMeshStats.mMaximumEdgeLength
                      << "), subdividing .." << std::endl;

            build_dedge(
                F, V, 
                V2E, 
                E2E,
                mBoundaryVertices, 
                mNonmanifoldVertices, 
                mProgress
            );
            subdivide(
                F, V, 
                V2E, 
                E2E,
                mBoundaryVertices, 
                mNonmanifoldVertices,
                std::min(scale / 2.f,
                (Float)mMeshStats.mAverageEdgeLength * 2.f),
                mDeterministic, 
                mProgress
            );
            mMeshStats = compute_mesh_stats(F, V, mDeterministic, mProgress);
        }

        // Rebuild directed edges on (possibly subdivided) mesh
        build_dedge(F, V, V2E, E2E, mBoundaryVertices, mNonmanifoldVertices, mProgress);

        adj = generate_adjacency_matrix_uniform(
            F, V2E, E2E, mNonmanifoldVertices, mProgress);

        // Normals
        MatrixXf N_crease;
        MatrixXu F_crease;
        MatrixXf V_crease;

        mCreaseMap.clear();
        mCreaseSet.clear();

        if (mCreaseAngle >= 0.f) {
            V_crease = V;
            F_crease = F;
            generate_crease_normals(F_crease, V_crease, V2E, E2E,
                                    mBoundaryVertices, mNonmanifoldVertices,
                                    mCreaseAngle, N_crease,
                                    mCreaseMap, mProgress);
            N = N_crease.topLeftCorner(3, V.cols());
        } else {
            generate_smooth_normals(F, V, V2E, E2E,
                                    mNonmanifoldVertices, N, mProgress);
        }

        for (auto const &kv : mCreaseMap)
            mCreaseSet.insert(kv.second);

        compute_dual_vertex_areas(F, V, V2E, E2E,
                                  mNonmanifoldVertices, A);

        mRes.setE2E(std::move(E2E));

    } else {
        // Point cloud path
        mBoundaryVertices.resize(V.cols());
        mNonmanifoldVertices.resize(V.cols());
        mBoundaryVertices.setConstant(false);
        mNonmanifoldVertices.setConstant(false);

        adj = generate_adjacency_matrix_pointcloud(
            V, N, mBVH, mMeshStats,
            10, mDeterministic, mProgress);

        A.resize(V.cols());
        A.setConstant(1.0f);
    }

    mRes.setF(std::move(F));
    mRes.setV(std::move(V));
    mRes.setN(std::move(N));
    mRes.setA(std::move(A));
    mRes.setAdj(std::move(adj));

    {
        std::lock_guard<ordered_lock> lock(mRes.mutex());
        mRes.setScale(scale);
    }

    mRes.build(mDeterministic, mProgress);
    mRes.resetSolution();

    // Apply boundary alignment constraints after the hierarchy is built
    if (mAlignToBoundaries && !pointcloud) {
        const MatrixXu &F_ref  = mRes.F();
        const MatrixXf &V_ref  = mRes.V();
        const VectorXu &E2E_ref = mRes.E2E();

        for (uint32_t i = 0; i < 3 * (uint32_t)F_ref.cols(); ++i) {
            if (E2E_ref[i] == INVALID) {
                uint32_t i0 = F_ref(i % 3, i / 3);
                uint32_t i1 = F_ref((i + 1) % 3, i / 3);
                Vector3f edge = V_ref.col(i1) - V_ref.col(i0);
                if (edge.squaredNorm() > 0) {
                    edge.normalize();
                    mRes.CQ().col(i0) = mRes.CQ().col(i1) = edge;
                    mRes.CO().col(i0) = V_ref.col(i0);
                    mRes.CO().col(i1) = V_ref.col(i1);
                    mRes.CQw()[i0] = mRes.CQw()[i1] = 1.f;
                    mRes.COw()[i0] = mRes.COw()[i1] = 1.f;
                }
            }
        }
        mRes.propagateConstraints(mOptimizer.rosy(), mOptimizer.posy());
    }
}

void Mesher::loadInput(const std::string &filename,
                       Float creaseAngle,
                       Float scale,
                       int   face_count,
                       int   vertex_count,
                       int   rosy,
                       int   posy,
                       int   knn_points) {

    mFilename = filename;

    // Read 
    MatrixXu F;
    MatrixXf V, N;

    load_mesh_or_pointcloud(filename, F, V, N, mProgress);

    bool pointcloud = (F.size() == 0);

    // Stop any running optimiser, tear down old BVH
    {
        std::lock_guard<ordered_lock> lock(mRes.mutex());
        mOptimizer.stop();
    }
    delete mBVH;
    mBVH = nullptr;

    // Mesh statistics 
    mMeshStats = compute_mesh_stats(F, V, mDeterministic, mProgress);

    // For point clouds, build BVH first 
    if (pointcloud) {
        mBVH = new BVH(&F, &V, &N, mMeshStats.mAABB);
        mBVH->build(mProgress);
    }

    // Resolve crease angle 
    if (!std::isfinite(creaseAngle)) {
        if (filename.find("fandisk")    != std::string::npos ||
            filename.find("cube_twist") != std::string::npos)
            creaseAngle = 20.f;
        else
            creaseAngle = -1.f;
    }
    mCreaseAngle = creaseAngle;

    // Resolve target scale 
    if (scale < 0 && vertex_count < 0 && face_count < 0) {
        std::cout << "No target specified; defaulting to 1/16 of input vertex count.\n";
        vertex_count = (int)(V.cols() / 16);
    }

    if (scale > 0) {
        Float face_area = (posy == 4)
            ? (scale * scale)
            : (std::sqrt(3.f) / 4.f * scale * scale);
        face_count   = (int)(mMeshStats.mSurfaceArea / face_area);
        vertex_count = (posy == 4) ? face_count : (face_count / 2);
    } else if (face_count > 0) {
        Float face_area = mMeshStats.mSurfaceArea / face_count;
        vertex_count    = (posy == 4) ? face_count : (face_count / 2);
        scale           = (posy == 4)
            ? std::sqrt(face_area)
            : (2.f * std::sqrt(face_area * std::sqrt(1.f / 3.f)));
    } else if (vertex_count > 0) {
        face_count      = (posy == 4) ? vertex_count : (vertex_count * 2);
        Float face_area = mMeshStats.mSurfaceArea / face_count;
        scale           = (posy == 4)
            ? std::sqrt(face_area)
            : (2.f * std::sqrt(face_area * std::sqrt(1.f / 3.f)));
    }

    std::cout << "Output mesh goals (approximate)\n"
              << "   Vertex count   = " << vertex_count  << "\n"
              << "   Face count     = " << face_count    << "\n"
              << "   Edge length    = " << scale         << "\n";

    // Set symmetry before building 
    setSymmetry(rosy, posy);

    // Build hierarchy 
    mRes.free();
    buildHierarchy(F, V, N, scale, rosy, posy);

    // Build / update BVH on the final mesh 
    if (!mBVH) {
        mBVH = new BVH(&mRes.F(), &mRes.V(), &mRes.N(), mMeshStats.mAABB);
        mBVH->build(mProgress);
    } else {
        mBVH->setData(&mRes.F(), &mRes.V(), &mRes.N());
    }

    mRes.printStatistics();
    mBVH->printStatistics();
}

// Field solvers
void Mesher::solveOrientation() {
    if (mRes.levels() == 0)
        throw std::runtime_error("No mesh loaded");

    {
        std::lock_guard<ordered_lock> lock(mRes.mutex());
        mOptimizer.optimizeOrientations(-1);
        mOptimizer.notify();
    }

    while (mOptimizer.active())
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void Mesher::solvePosition() {
    if (mRes.levels() == 0 || mRes.iterationsQ() < 0)
        throw std::runtime_error("Run solveOrientation() first");

    {
        std::lock_guard<ordered_lock> lock(mRes.mutex());
        mOptimizer.optimizePositions(-1);
        mOptimizer.notify();
    }

    while (mOptimizer.active())
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

// Extraction
void Mesher::extractMesh() {
    if (mRes.levels() == 0 || mRes.iterationsO() < 0)
        throw std::runtime_error("Run solvePosition() first");

    int  rosy      = mOptimizer.rosy();
    int  posy      = mOptimizer.posy();
    bool extrinsic = mOptimizer.extrinsic();

    std::vector<std::vector<TaggedLink>> adj_extracted;
    std::set<uint32_t> creaseOut;

    extract_graph(mRes, extrinsic, rosy, posy,
                  adj_extracted,
                  mV_extracted, mN_extracted,
                  mCreaseSet, creaseOut,
                  mDeterministic);

    extract_faces(adj_extracted,
                  mV_extracted, mN_extracted, mNf_extracted, mF_extracted,
                  posy, mRes.scale(), creaseOut,
                  true,
                  mPureQuad,
                  mBVH,
                  mSmoothIterations);

    std::cout << "Extraction complete: "
              << mF_extracted.cols() << " faces, "
              << mV_extracted.cols() << " vertices.\n";
}

// Output
void Mesher::saveOutput(const std::string &filename) {
    if (mF_extracted.size() == 0 || mV_extracted.size() == 0)
        throw std::runtime_error("No extracted mesh — run extractMesh() first");

    write_mesh(filename, mF_extracted, mV_extracted, MatrixXf(), mNf_extracted);

    std::cout << "Saved: " << filename << "\n";
}