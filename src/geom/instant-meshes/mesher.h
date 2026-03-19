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

#include "hierarchy.h"
#include "field.h"
#include "bvh.h"
#include "meshstats.h"
#include <string>
#include <set>
#include <map>
#include <functional>
#include <limits>

class Mesher {
public:
    Mesher(bool deterministic = false);
    ~Mesher();

    // Input
    void loadInput(
        const std::string &filename,
        Float creaseAngle     = std::numeric_limits<Float>::infinity(),
        Float scale           = -1,
        int   face_count      = -1,
        int   vertex_count    = -1,
        int   rosy            = 4,
        int   posy            = 4,
        int   knn_points      = 10
    );

    // Field solve
    void solveOrientation();
    void solvePosition();

    // Output 
    void extractMesh();
    void saveOutput(const std::string &filename);

    // Config 
    void setSymmetry(int rosy, int posy);
    void setExtrinsic(bool extrinsic);
    void setTargetScale(Float scale);
    void setTargetVertexCount(uint32_t v);

    void setSmoothIterations(int n)       { mSmoothIterations   = n;    }
    void setPureQuad(bool pq)             { mPureQuad           = pq;   }
    void setAlignToBoundaries(bool align) { mAlignToBoundaries  = align; }

    // Optional progress callback
    using ProgressCallback = std::function<void(const std::string &, Float)>;
    void setProgressCallback(ProgressCallback cb) { mProgress = cb; }

protected:
    void buildHierarchy(
        MatrixXu &F, MatrixXf &V, MatrixXf &N,
        Float scale, int rosy, int posy
    );

private:
    bool mDeterministic;

    /* Core data */
    MultiResolutionHierarchy mRes;
    Optimizer                mOptimizer;
    BVH                     *mBVH;
    MeshStats                mMeshStats;

    /* Crease handling */
    std::map<uint32_t, uint32_t> mCreaseMap;
    std::set<uint32_t>           mCreaseSet;
    Float                        mCreaseAngle;

    /* Topology masks */
    VectorXb mNonmanifoldVertices;
    VectorXb mBoundaryVertices;

    /* Extraction result */
    MatrixXu mF_extracted;
    MatrixXf mV_extracted;
    MatrixXf mN_extracted;
    MatrixXf mNf_extracted;

    /* Source filename */
    std::string mFilename;

    /* Extraction options */
    int  mSmoothIterations;
    bool mPureQuad;
    bool mAlignToBoundaries;

    /* Progress */
    ProgressCallback mProgress;
};