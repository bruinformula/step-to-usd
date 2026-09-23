#include <chrono>
#include <string>
#include <random>
#include <algorithm>

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
#include <BRepClass_FaceClassifier.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

#include <Eigen/Dense>

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

//#include "InstantMeshes/mesher.h"
//#include "InstantMeshes/common.h"

class Geom_Surface;

PXR_NAMESPACE_USING_DIRECTIVE

using namespace Eigen;

using Clock = std::chrono::high_resolution_clock;
using Seconds = std::chrono::duration<double>;

bool InstantMeshesTessellationRoutine::tessellate(
    const TopoDS_Shape& defShape, 
    const TessParams& params,
    const SdfPath& protoPath
) {
    auto tessellateStart = Clock::now();

    LOG_DEBUG("  -> tessellatePart: Edge walk preparation");

    static_assert(sizeof(gp_Pnt) == 3 * sizeof(double),
                  "gp_Pnt layout assumption broken - Eigen::Map reinterpret_cast is unsafe");
    static_assert(sizeof(gp_Vec) == 3 * sizeof(double),
                  "gp_Vec layout assumption broken - Eigen::Map reinterpret_cast is unsafe");
    static_assert(sizeof(GfVec3f) == 3 * sizeof(float),
                  "GfVec3f layout assumption broken - Eigen::Map reinterpret_cast is unsafe");

    std::random_device rd;
    std::mt19937 gen(rd());

    // This is just a test. We're just generating random 
    // surface positions unformally atop each one of the 
    // faces of the part 
    std::vector<gp_Pnt> inputPoints;
    std::vector<gp_Vec> inputNormals;
    std::vector<gp_Vec> inputTangents;
    std::vector<float>  inputTangentWeights;

    const double samplesPerUnitArea = 1;
    const int minSamplesPerFace = 50;
    const int maxSamplesPerFace = 1000000;

    for (TopExp_Explorer faceExp(defShape, TopAbs_FACE);
         faceExp.More();
         faceExp.Next())
    {
        const TopoDS_Face& face = TopoDS::Face(faceExp.Current());
    
        BRepAdaptor_Surface adapter(face);
    
        const double uMin = adapter.FirstUParameter();
        const double uMax = adapter.LastUParameter();
        const double vMin = adapter.FirstVParameter();
        const double vMax = adapter.LastVParameter();
    
        // Sample count scaled to face area rather than a fixed constant,
        // so density-per-unit-area is roughly uniform across the part.
        GProp_GProps massProps;
        BRepGProp::SurfaceProperties(face, massProps);
        const double faceArea = massProps.Mass();

        int samplesPerFace = static_cast<int>(std::round(faceArea * samplesPerUnitArea));
        samplesPerFace = std::clamp(samplesPerFace, minSamplesPerFace, maxSamplesPerFace);

        // Instead: classify + weight each grid cell ONCE up front, then draw
        // samples directly from the resulting discrete distribution. Cost becomes
        // O(gridRes^2) setup + O(samplesPerFace) sampling, independent of
        // acceptance rate - a sliver face now costs the same as a well-behaved one.
        constexpr int gridRes = 64; // cells per axis; raise for finely trimmed faces
        std::vector<double> cellWeights(static_cast<size_t>(gridRes) * gridRes, 0.0);

        const double duCell = (uMax - uMin) / gridRes;
        const double dvCell = (vMax - vMin) / gridRes;

        BRepTopAdaptor_FClass2d fastClassifier(face, Precision::Confusion());

        for (int gi = 0; gi < gridRes; ++gi) {
            const double guCenter = uMin + duCell * (gi + 0.5);
            for (int gj = 0; gj < gridRes; ++gj) {
                const double gvCenter = vMin + dvCell * (gj + 0.5);

                const TopAbs_State state = fastClassifier.Perform(gp_Pnt2d(guCenter, gvCenter));
                if (state != TopAbs_IN && state != TopAbs_ON) {
                    continue; // weight stays 0.0 - cell excluded from sampling
                }

                gp_Pnt cPos;
                gp_Vec cdU, cdV;
                adapter.D1(guCenter, gvCenter, cPos, cdU, cdV);
                // Weight by the local area element so density comes out uniform
                // per unit surface area (this is what keeps poles from clustering).
                cellWeights[static_cast<size_t>(gi) * gridRes + gj] = cdU.Crossed(cdV).Magnitude();
            }
        }

        // Shared point-emission logic, used by both the fast grid path below and
        // the sliver fallback path further down - keeps them in sync.
        auto emitSample = [&](double u, double v) {
            gp_Pnt position;
            gp_Vec dU;
            gp_Vec dV;

            adapter.D1(u, v, position, dU, dV);
            gp_Vec normal = dU.Crossed(dV);
            const double jacobian = normal.Magnitude();

            if (face.Orientation() == TopAbs_REVERSED)
                normal.Reverse();

            gp_Vec unitNormal = normal.Normalized();

            // Use the U-parametric tangent as the guide direction, re-orthogonalized
            // against the normal to remove numerical drift.
            // This is subject to change
            gp_Vec tangent = dU - unitNormal * unitNormal.Dot(dU);
            double tangentLen = tangent.Magnitude();

            bool tangentValid = tangentLen > Precision::Confusion()
                              && jacobian > Precision::Confusion();

            if (tangentValid) {
                tangent.Normalize();
            } else {
                tangent = gp_Vec(0.0, 0.0, 0.0);
            }

            inputPoints.push_back(position);
            inputNormals.push_back(unitNormal);
            inputTangents.push_back(tangent);
            inputTangentWeights.push_back(tangentValid ? 1.0f : 0.0f);
        };

        const double totalWeight = std::accumulate(cellWeights.begin(), cellWeights.end(), 0.0);
        if (totalWeight <= Precision::Confusion()) {
            // The coarse grid's cell-center tests missed this face's trimmed
            // region entirely - typical of a sliver or a small face relative to
            // its underlying surface's UV extent. Rather than silently producing
            // zero points, fall back to exact rejection sampling just for this
            // one face; it's the slow path, but it only triggers for faces the
            // fast path can't see at all.
            std::uniform_real_distribution<double> uDist(uMin, uMax);
            std::uniform_real_distribution<double> vDist(vMin, vMax);

            int samples = 0;
            long attempts = 0;
            const long maxAttempts = static_cast<long>(samplesPerFace) * 500;

            while (samples < samplesPerFace && attempts < maxAttempts) {
                ++attempts;
                const double u = uDist(gen);
                const double v = vDist(gen);
                const TopAbs_State state = fastClassifier.Perform(gp_Pnt2d(u, v));
                if (state != TopAbs_IN && state != TopAbs_ON) {
                    continue;
                }
                emitSample(u, v);
                ++samples;
            }

            if (samples < samplesPerFace) {
                LOG_DEBUG("  -> sliver face: only got " + std::to_string(samples) + "/" +
                           std::to_string(samplesPerFace) + " samples via fallback sampling");
            }
            continue; // done with this face
        }

        // Flag cells adjacent (8-connected) to an excluded cell. The grid only
        // knows a cell's CENTER is in/out, so wherever the real trim boundary
        // cuts through one of these cells, blindly trusting the grid produces
        // the axis-aligned staircase along trim edges. Interior cells (the vast
        // majority) skip this and keep full direct-sampling speed.
        std::vector<uint8_t> isBoundaryCell(cellWeights.size(), 0);
        auto weightAt = [&](int gi, int gj) -> double {
            if (gi < 0 || gi >= gridRes || gj < 0 || gj >= gridRes) return 0.0;
            return cellWeights[static_cast<size_t>(gi) * gridRes + gj];
        };
        for (int gi = 0; gi < gridRes; ++gi) {
            for (int gj = 0; gj < gridRes; ++gj) {
                const size_t idx = static_cast<size_t>(gi) * gridRes + gj;
                if (cellWeights[idx] <= 0.0) continue; // only included cells matter here
                bool edge = false;
                for (int dgi = -1; dgi <= 1 && !edge; ++dgi)
                    for (int dgj = -1; dgj <= 1 && !edge; ++dgj)
                        if ((dgi || dgj) && weightAt(gi + dgi, gj + dgj) <= 0.0)
                            edge = true;
                isBoundaryCell[idx] = edge ? 1 : 0;
            }
        }

        // Builds its own cumulative table once; each draw below is then a single
        // O(log gridRes^2) pick - no rejection, no further classifier calls,
        // except for the boundary-cell verification noted above.
        std::discrete_distribution<int> cellDist(cellWeights.begin(), cellWeights.end());
        std::uniform_real_distribution<double> cellLocalDist(0.0, 1.0);

        int samples = 0;
        while (samples < samplesPerFace) {
            const int cellIdx = cellDist(gen);
            const int gi = cellIdx / gridRes;
            const int gj = cellIdx % gridRes;

            double u = uMin + duCell * (gi + cellLocalDist(gen));
            double v = vMin + dvCell * (gj + cellLocalDist(gen));

            if (isBoundaryCell[cellIdx]) {
                // Verify against the real trim curve instead of trusting the
                // grid; retry a few sub-positions within the same cell before
                // giving up and redrawing a different cell. This is what makes
                // the boundary follow the true (possibly curved/diagonal) trim
                // edge instead of the grid's staircase.
                bool inside = false;
                for (int tries = 0; tries < 8; ++tries) {
                    const TopAbs_State state = fastClassifier.Perform(gp_Pnt2d(u, v));
                    if (state == TopAbs_IN || state == TopAbs_ON) {
                        inside = true;
                        break;
                    }
                    u = uMin + duCell * (gi + cellLocalDist(gen));
                    v = vMin + dvCell * (gj + cellLocalDist(gen));
                }
                if (!inside) {
                    continue; // this slice of the cell is outside - redraw a cell
                }
            }

            emitSample(u, v);
            ++samples;
        }
    }
    
    if (inputPoints.empty() || inputNormals.empty()) {
        return false;
    }

    for (int i = 0; i < inputPoints.size(); ++i) {
        float px = static_cast<float>(inputPoints[i].X());
        float py = static_cast<float>(inputPoints[i].Y());
        float pz = static_cast<float>(inputPoints[i].Z());
        samplePoints.emplace_back(px, py, pz);

        float nx = static_cast<float>(inputNormals[i].X());
        float ny = static_cast<float>(inputNormals[i].Y());
        float nz = static_cast<float>(inputNormals[i].Z());
        sampleNormals.emplace_back(nx, ny, nz);        
    }
    
    MatrixXf P, N, Tt;
    VectorXf Wt;

    // Vectorized AoS -> SoA conversion. gp_Pnt/gp_Vec are POD wrappers around
    // 3 contiguous doubles, so we can Eigen::Map the raw buffers directly
    // instead of looping element-by-element through .X()/.Y()/.Z() accessors.
    {
        Eigen::Map<const Eigen::Matrix3Xd> Pd(
            reinterpret_cast<const double*>(inputPoints.data()), 3, inputPoints.size());
        P = Pd.cast<float>();

        Eigen::Map<const Eigen::Matrix3Xd> Nd(
            reinterpret_cast<const double*>(inputNormals.data()), 3, inputNormals.size());
        N = Nd.cast<float>();

        Eigen::Map<const Eigen::Matrix3Xd> Td(
            reinterpret_cast<const double*>(inputTangents.data()), 3, inputTangents.size());
        Tt = Td.cast<float>();

        Wt = Eigen::Map<const Eigen::VectorXf>(
            inputTangentWeights.data(), inputTangentWeights.size());
    }

    InstantMeshes::MeshParams mesherParams;

    mesherParams.scale = 10.0f;
    mesherParams.vertexCount = -1;
    mesherParams.faceCount = -1;
    mesherParams.alignToBoundaries = true;
    mesherParams.smoothIter = 0;

    std::cout
        << "Mesher params:"
        << " rosy=" << mesherParams.rosy
        << " posy=" << mesherParams.posy
        << " extrinsic=" << mesherParams.extrinsic
        << " knnPoints=" << mesherParams.knnPoints
        << " vertexCount=" << mesherParams.vertexCount
        << '\n';
    
    InstantMeshes::Mesher mesher(mesherParams);
    
    mesher.loadInput(P, N, Tt, Wt);
    //mesher.loadInput(P, N);

    std::cout << "Instant Meshes input: "
            << P.cols() << " points, "
            << N.cols() << " normals\n";

    std::cout << "P:\n"
            << "  cols = " << P.cols() << '\n'
            << "  min  = " << P.rowwise().minCoeff().transpose() << '\n'
            << "  max  = " << P.rowwise().maxCoeff().transpose() << '\n'
            << "  mean = " << P.rowwise().mean().transpose() << '\n';
    
    std::cout << "N:\n"
            << "  cols = " << N.cols() << '\n'
            << "  min  = " << N.rowwise().minCoeff().transpose() << '\n'
            << "  max  = " << N.rowwise().maxCoeff().transpose() << '\n';

    std::cout << "Solving orientation field ...\n";
    mesher.solveOrientation();

    std::cout << "Solving position field ...\n";
    mesher.solvePosition();

    std::cout << "Extracting mesh ...\n";
    mesher.extractMesh();
    
    if (mesher.mF_extracted.size() == 0 || mesher.mV_extracted.size() == 0)
        throw std::runtime_error("No extracted mesh — run extractMesh() first");

    // Populate Points
    const int vertexOffset = static_cast<int>(points.size());
    const size_t newVertexCount = static_cast<size_t>(mesher.mV_extracted.cols());

    // Vectorized SoA -> AoS write-out. GfVec3f is 3 contiguous floats with no
    // padding, so this is a single block assignment instead of a per-vertex
    // constructor loop.
    {
        points.resize(points.size() + newVertexCount);
        Eigen::Map<Eigen::Matrix3Xf> outPoints(
            reinterpret_cast<float*>(points.data() + vertexOffset), 3, newVertexCount);
        outPoints = mesher.mV_extracted;
    }

    // Process Faces and collect irregular n-gon edges
    std::map<uint32_t, std::pair<uint32_t, std::map<uint32_t, uint32_t>>> irregular;
    
    bool hasFaceNormals = (mesher.mNf_extracted.size() > 0);
    bool hasVertexNormals = (mesher.mN_extracted.size() > 0);

    faceVertexCounts.reserve(faceVertexCounts.size() + mesher.mF_extracted.cols());
    faceVertexIndices.reserve(faceVertexIndices.size() + mesher.mF_extracted.size());

    for (uint32_t f = 0; f < mesher.mF_extracted.cols(); ++f) {
        // Check for irregular face segments (Instant Meshes n-gon encoding)
        if (mesher.mF_extracted.rows() == 4) {
            if (mesher.mF_extracted(2, f) == mesher.mF_extracted(3, f)) {
                auto &value = irregular[mesher.mF_extracted(2, f)];
                value.first = f; // Face index used for normal lookup
                value.second[mesher.mF_extracted(0, f)] = mesher.mF_extracted(1, f);
                continue;
            }
        }

        // Regular face (Triangle or Quad)
        uint32_t numVerts = mesher.mF_extracted.rows();
        faceVertexCounts.push_back(static_cast<int>(numVerts));

        for (uint32_t j = 0; j < numVerts; ++j) {
            faceVertexIndices.push_back(vertexOffset + static_cast<int>(mesher.mF_extracted(j, f)));
        }

        if (hasFaceNormals) {
            normals.push_back(GfVec3f(mesher.mNf_extracted(0, f),
                                    mesher.mNf_extracted(1, f),
                                    mesher.mNf_extracted(2, f)));
        }
    }

    // Reconstruct Irregular N-Gons
    for (const auto &item : irregular) {
        const auto &face = item.second;
        uint32_t v = face.second.begin()->first;
        uint32_t first = v;
        uint32_t count = 0;

        std::vector<int> ngonIndices;
        while (true) {
            ngonIndices.push_back(static_cast<int>(v));
            v = face.second.at(v);
            if (v == first || ++count == face.second.size())
                break;
        }

        faceVertexCounts.push_back(static_cast<int>(ngonIndices.size()));
        for (int idx : ngonIndices) {
            faceVertexIndices.push_back(vertexOffset + idx);
        }

        if (hasFaceNormals) {
            uint32_t faceIdx = face.first;
            normals.push_back(GfVec3f(mesher.mNf_extracted(0, faceIdx),
                                    mesher.mNf_extracted(1, faceIdx),
                                    mesher.mNf_extracted(2, faceIdx)));
        }
    }

    //  Vertex Normals (if per-vertex normals are used instead of face normals)
    if (hasVertexNormals && !hasFaceNormals) {
        normals.reserve(normals.size() + mesher.mN_extracted.cols());
        for (uint32_t i = 0; i < mesher.mN_extracted.cols(); ++i) {
            normals.push_back(GfVec3f(mesher.mN_extracted(0, i),
                                    mesher.mN_extracted(1, i),
                                    mesher.mN_extracted(2, i)));
        }
    }

    auto tessellateEnd = Clock::now();
    LOG_DEBUG("  Total tessellatePart time: " + std::to_string(Seconds(tessellateEnd - tessellateStart).count()) + " s");

    return true;
}