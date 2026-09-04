#include <chrono>
#include <string>
#include <random>

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

#include "InstantMeshes/mesher.h"
#include "InstantMeshes/common.h"

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

    if (!params.meshEnableInstantMeshes)
        return true;
    
    auto tessellateStart = Clock::now();

    LOG_DEBUG("  -> tessellatePart: Edge walk preparation");
    
    std::random_device rd;
    std::mt19937 gen(rd());

    // This is just a test. We're just generating random 
    // surface positions unformally atop each one of the 
    // faces of the part 

    for (TopExp_Explorer faceExp(defShape, TopAbs_FACE);
         faceExp.More();
         faceExp.Next())
    {

        std::vector<gp_Pnt> inputPoints;
        std::vector<gp_Vec> inputNormals;
    
        const TopoDS_Face& face = TopoDS::Face(faceExp.Current());
    
        BRepAdaptor_Surface adapter(face);
    
        const double uMin = adapter.FirstUParameter();
        const double uMax = adapter.LastUParameter();
        const double vMin = adapter.FirstVParameter();
        const double vMax = adapter.LastVParameter();
    
        std::uniform_real_distribution<double> uDist(uMin, uMax);
        std::uniform_real_distribution<double> vDist(vMin, vMax);
        
        const int samplesPerFace = 100000; 
        BRepClass_FaceClassifier classifier;

        // Because of trim samples aren't guranteed 
        // to be inside the UV bounds given 
        int samples = 0;
        while (samples < samplesPerFace) {
            const double u = uDist(gen);
            const double v = vDist(gen);
    
            // Is the given coordinate
            // inside the face boundary.
            classifier.Perform(
                face,
                gp_Pnt2d(u, v),
                Precision::Confusion()
            );
    
            const TopAbs_State state = classifier.State();
            if (state != TopAbs_IN && state != TopAbs_ON) {
                continue; // outside throw out
            }

            gp_Pnt position;
            gp_Vec dU;
            gp_Vec dV;

            adapter.D1(u, v, position, dU, dV);
            gp_Vec normal = dU.Crossed(dV);

            if (face.Orientation() == TopAbs_REVERSED)
                normal.Reverse();
    
            inputPoints.push_back(position);
            inputNormals.push_back(normal.Normalized());
            ++samples;
        }

        samplePoints.reserve(inputPoints.size());
        sampleNormals.reserve(inputNormals.size());
    
        for (int i = 0; i < inputPoints.size(); ++i) {
            float px = static_cast<float>(inputPoints[i].X());
            float py = static_cast<float>(inputPoints[i].Y());
            float pz = static_cast<float>(inputPoints[i].Z());
            samplePoints.emplace_back(px, py, pz);
    
            float nx = static_cast<float>(inputPoints[i].X());
            float ny = static_cast<float>(inputPoints[i].Y());
            float nz = static_cast<float>(inputPoints[i].Z());
            sampleNormals.emplace_back(nx, ny, nz);        
        }
    
        if (inputPoints.empty() || inputNormals.empty()) {
            return false;
        }
        
        MatrixXf P;
        MatrixXf N;
        
        P.resize(3, inputPoints.size());
        N.resize(3, inputNormals.size());
        
        for (size_t i = 0; i < inputPoints.size(); ++i) {
            P(0, i) = static_cast<float>(inputPoints[i].X());
            P(1, i) = static_cast<float>(inputPoints[i].Y());
            P(2, i) = static_cast<float>(inputPoints[i].Z());
        
            N(0, i) = static_cast<float>(inputNormals[i].X());
            N(1, i) = static_cast<float>(inputNormals[i].Y());
            N(2, i) = static_cast<float>(inputNormals[i].Z());
        } 
    
        InstantMeshes::MeshParams mesherParams;

        mesherParams.scale = 0.4f;
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
        
        mesher.loadInput(P, N);
    
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
    
        // Clear output arrays
        //points.clear();
        //normals.clear();
        //faceVertexCounts.clear();
        //faceVertexIndices.clear();
    
        // Populate Points
        const int vertexOffset = static_cast<int>(points.size());
        
        points.reserve(points.size() + mesher.mV_extracted.cols());
        for (uint32_t i = 0; i < mesher.mV_extracted.cols(); ++i) {
            points.push_back(GfVec3f(mesher.mV_extracted(0, i),
                                      mesher.mV_extracted(1, i),
                                      mesher.mV_extracted(2, i)));
        }
    
        // Process Faces and collect irregular n-gon edges
        std::map<uint32_t, std::pair<uint32_t, std::map<uint32_t, uint32_t>>> irregular;
        
        bool hasFaceNormals = (mesher.mNf_extracted.size() > 0);
        bool hasVertexNormals = (mesher.mN_extracted.size() > 0);
    
        faceVertexCounts.reserve(mesher.mF_extracted.cols());
        faceVertexIndices.reserve(mesher.mF_extracted.size());
    
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
            normals.reserve(mesher.mN_extracted.cols());
            for (uint32_t i = 0; i < mesher.mN_extracted.cols(); ++i) {
                normals.push_back(GfVec3f(mesher.mN_extracted(0, i),
                                        mesher.mN_extracted(1, i),
                                        mesher.mN_extracted(2, i)));
            }
        }
    }

    auto tessellateEnd = Clock::now();
    LOG_DEBUG("  Total tessellatePart time: " + std::to_string(Seconds(tessellateEnd - tessellateStart).count()) + " s");

    return true;
}
