/*
    mesher.h: Bare-minimum meshing pipeline extracted from Instant Meshes
    Original work:
        Instant Field-Aligned Meshes
        Wenzel Jakob, Daniele Panozzo, Marco Tarini, and Olga Sorkine-Hornung
        In ACM Transactions on Graphics (Proc. SIGGRAPH Asia 2015)
    All rights reserved. Use of this source code is governed by a
    BSD-style license that can be found in the LICENSE.txt file.
*/
#pragma once

#include <string>
#include <set>
#include <map>
#include <functional>
#include <filesystem>

#include "hierarchy.h"
#include "field.h"
#include "bvh.h"
#include "meshstats.h"

struct MeshParams {
    std::filesystem::path inputPath;
    std::filesystem::path outputPath;

    int rosy = 4;
    int posy = 4;
    bool extrinsic = true;
    bool alignToBoundaries = false;
    bool dominant = false;

    Float scale = -1.f;
    int faceCount = -1;
    int vertexCount = -1;

    Float creaseAngle = -1.f;
    uint32_t smoothIter = 2;
    uint32_t knnPoints = 10;

    bool deterministic = false;
};

class Mesher {
public:
    explicit Mesher(const MeshParams &params);
    ~Mesher();

    void loadInput();
    void solveOrientation();
    void solvePosition();
    void extractMesh();
    void saveOutput();

    using ProgressCallback = std::function<void(const std::string &, Float)>;
    void setProgressCallback(ProgressCallback cb) { mProgress = cb; }

protected:
    void buildHierarchy(MatrixXu &F, MatrixXf &V, MatrixXf &N, Float scale);

private:
    MeshParams mParams;

    MultiResolutionHierarchy mRes;
    Optimizer mOptimizer;
    BVH *mBVH;
    MeshStats mMeshStats;

    std::map<uint32_t, uint32_t> mCreaseMap;
    std::set<uint32_t> mCreaseSet;

    VectorXb mNonmanifoldVertices;
    VectorXb mBoundaryVertices;

    MatrixXu mF_extracted;
    MatrixXf mV_extracted;
    MatrixXf mN_extracted;
    MatrixXf mNf_extracted;

    ProgressCallback mProgress;
};