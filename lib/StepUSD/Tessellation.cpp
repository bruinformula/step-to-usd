
#include <iostream>
#include <chrono>
#include <utility>
#include <algorithm>
#include <cmath>
#include <functional>
#include <initializer_list>
#include <unordered_map>
#include <unordered_set>
#include <atomic>
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

#include "StepUSD/StepUsdPipeline.h"
#include "StepUSD/Logger.h"
#include "StepUSD/Tessellation/TessellationRoutine.h"
#include "StepUSD/Tessellation/TessellationUtils.h"
#include "StepUSD/Tessellation/DeadlineProgressIndicator.h"

class Geom_Surface;

PXR_NAMESPACE_USING_DIRECTIVE

bool StepUsdPipeline::tessellatePart(
    TessResult& result, 
    const TopoDS_Shape& defShape, 
    const TessParams& params,
    const SdfPath& protoPath
) {
    using Clock = std::chrono::high_resolution_clock;
    using Seconds = std::chrono::duration<double>;

    auto tessellateStart = Clock::now();

    std::string protoName = protoPath.GetAsString();

    LOG_DEBUG("  -> tessellatePart: ShapeFix_Shape (Repair pass)");
    ShapeFix_Shape fixer(defShape);
    fixer.SetPrecision(params.meshFixPrecision);
    fixer.SetMaxTolerance(params.meshFixTolerance);
    
    bool fixTimedOut = runWithDeadline(
        std::chrono::milliseconds(params.meshFixTimeout),
        "ShapeFix_Shape for part " + protoName + ": ",
        fixer
    );

    if (fixTimedOut) {
        LOG_DEBUG("  -> ShapeFix_Shape timed out, proceeding with partial repair");
    }
    
    TopoDS_Shape fixedShape = fixer.Shape();

    LOG_DEBUG("  -> tessellatePart: BRepTools::Clean");
    BRepTools::Clean(defShape); // remove previously created tessellations for this part 

    double diagonal = computeBoundingBoxDiagonal(defShape);

    result.renderOnly = params.renderPurposeThreshold != std::numeric_limits<double>::infinity() && diagonal < params.renderPurposeThreshold;

    IMeshTools_Parameters meshParams;
    meshParams.InParallel = false; 
    meshParams.Deflection = diagonal * params.meshLinearDeflection;
    meshParams.Angle = params.meshAngularDeflection; // in radians
    meshParams.MinSize = meshParams.Deflection * params.meshMinSize;
    
    LOG_DEBUG("  -> tessellatePart: BRepMesh_IncrementalMesh");
    BRepMesh_IncrementalMesh mesher(defShape, meshParams);

    bool meshTimedOut = runWithDeadline(
        std::chrono::milliseconds(params.meshMeshTimeout),
        "BRepMesh_IncrementalMesh for part " + protoName + ": ",
        mesher
    );

    if (meshTimedOut) {
        LOG_DEBUG("  -> BRepMesh_IncrementalMesh timed out"); // Some faces will have null triangulations
    }

    int maxPasses = params.meshMaxNumberRemeshPasses;
    IMeshTools_Parameters repairParams = meshParams;

    // repeat check for self-intersections 
    // if fail, refine the mesh until there are none 
    // or we hit the max pass count.

    // important note this is on the whole shape 
    // not just the intersected shapes, 
    // so the edge walk later works

    LOG_DEBUG("  -> tessellatePart: Starting remesh passes (" + std::to_string(maxPasses) + ")");
    for (int pass = 0; pass < maxPasses; ++pass) {
        LOG_DEBUG("  Running self-intersection check (pass " + std::to_string(pass) + ")");
        BRepExtrema_SelfIntersection checker(defShape, params.meshSelfIntersectionThreshold);
        LOG_DEBUG("  -> checker.Perform()");
        checker.Perform();

        if (!checker.IsDone()) {
            LOG_DEBUG("  -> checker not done, breaking");
            break;
        }

        LOG_DEBUG("  -> checker getting OverlapElements");
        const BRepExtrema_MapOfIntegerPackedMapOfInteger& overlaps = checker.OverlapElements();

        if (overlaps.IsEmpty()) {
            LOG_DEBUG("  No interesections found.");
            break;
        }
        
        LOG_DEBUG("  Found overlaps. Remeshing with finer parameters.");

        repairParams.Deflection *= 0.5;
        repairParams.Angle *= 0.5;

        LOG_DEBUG("  -> BRepTools::Clean (repair)");
        BRepTools::Clean(defShape); 
        LOG_DEBUG("  -> BRepMesh_IncrementalMesh (repair)");
        BRepMesh_IncrementalMesh remesher(defShape, repairParams);
        
        bool remeshTimedOut = runWithDeadline(
            std::chrono::milliseconds(params.meshRemeshTimeout),
            "BRepMesh_IncrementalMesh for part " + protoName + ": ",
            remesher
        );

        if (remeshTimedOut) {
            LOG_DEBUG("  -> BRepMesh_IncrementalMesh (repair) timed out");
            break;
        }
    }
    
    auto meshEnd = Clock::now();
    LOG_DEBUG("  Mesh time: " + std::to_string(Seconds(meshEnd - tessellateStart).count()) + " s");


    MeshTessellationRoutine meshTessellationRoutine;
    SketchTessellationRoutine sketchTessellationRoutine;

    bool meshSuccess = meshTessellationRoutine.tessellate(fixedShape, params, protoPath, result);
    bool sketchSuccess = sketchTessellationRoutine.tessellate(fixedShape, params, protoPath, result);

    return meshSuccess && sketchSuccess;
}

struct ShapeKey {
    const void* tshape;

    bool operator==(const ShapeKey& other) const {
        return tshape == other.tshape;
    }
    struct Hash {
        size_t operator()(const ShapeKey& k) const {
            return std::hash<const void*>{}(k.tshape);
        }
    };
};

static bool tessParamsEqual(const TessParams& a, const TessParams& b) {
    return std::memcmp(&a, &b, sizeof(TessParams)) == 0;
}

struct ParamSubgroup {
    TessParams params;
    std::vector<TessellationJob*> jobs;
};

void StepUsdPipeline::tessellateGeometry(
    std::vector<TessellationJob>& tessJobs,
    const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths
) {
    std::unordered_map<ShapeKey, std::vector<TessellationJob*>, ShapeKey::Hash> jobsByShape;
    for (TessellationJob& job : tessJobs) {
        const auto& defs = job.proto->model->getDefinitionShapes();
        const TopoDS_Shape& shape = defs[job.defIndex].second;
        jobsByShape[ShapeKey{shape.TShape().get()}].push_back(&job);
    }

    // Pre-calculate complexity (face count) per unique shape for load balancing.
    std::vector<ShapeKey> shapeKeys;
    shapeKeys.reserve(jobsByShape.size());
    std::unordered_map<ShapeKey, int, ShapeKey::Hash> shapeComplexity;
    shapeComplexity.reserve(jobsByShape.size());

    for (const auto& [key, jobs] : jobsByShape) {
        shapeKeys.push_back(key);
        const TessellationJob* rep = jobs.front();
        const auto& defs = rep->proto->model->getDefinitionShapes();
        const TopoDS_Shape& shape = defs[rep->defIndex].second;
        int complexity = 0;
        for (TopExp_Explorer faceExp(shape, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
            complexity++;
        }
        shapeComplexity[key] = complexity;
    }

    // Sort descending by complexity so heaviest jobs start first
    std::sort(shapeKeys.begin(), shapeKeys.end(), [&shapeComplexity](const ShapeKey& a, const ShapeKey& b) {
        return shapeComplexity.at(a) > shapeComplexity.at(b);
    });

    {
        LOG_SCOPED_TIMER("Parallel Tessellation of " + std::to_string(shapeKeys.size()) + " unique shapes.");
        std::atomic<int> completedJobs{0};
        const int totalJobs = static_cast<int>(tessJobs.size());

        WorkParallelForEach(shapeKeys.begin(), shapeKeys.end(), [&](const ShapeKey& key) {
            const std::vector<TessellationJob*>& jobs = jobsByShape.at(key);

            const TessellationJob* rep = jobs.front();
            const auto& defs = rep->proto->model->getDefinitionShapes();
            const TopoDS_Shape& shape = defs[rep->defIndex].second;

            std::vector<ParamSubgroup> subgroups;

            for (TessellationJob* jobPtr : jobs) {
                TessellationJob& job = *jobPtr;

                bool bTessellate = isPrototypeActiveInFilter(
                    selectedPaths, job.prototypePath, job.proto->variantSetName, job.proto->variantName);

                if (!bTessellate) {
                    int currentCount = ++completedJobs;
                    LOG_PROGRESS(currentCount, totalJobs, "Tessellating Geometry");
                    continue;
                }

                bool merged = false;
                for (ParamSubgroup& sg : subgroups) {
                    if (tessParamsEqual(sg.params, job.params)) {
                        sg.jobs.push_back(jobPtr);
                        merged = true;
                        break;
                    }
                }
                if (!merged) {
                    subgroups.push_back(ParamSubgroup{job.params, {jobPtr}});
                }
            }

            for (ParamSubgroup& sg : subgroups) {
                TessellationJob& primary = *sg.jobs.front();

                LOG_DEBUG("Tessellating part: " + primary.prototypePath.GetString() +
                          " (def index " + std::to_string(primary.defIndex) +
                          ", shared by " + std::to_string(sg.jobs.size()) + " prototype path(s))");

                try {
                    tessellatePart(primary.result, shape, sg.params, primary.prototypePath);

                    // Fan the computed result out to every other job that needs
                    // this exact (shape, params) pair.
                    for (size_t i = 1; i < sg.jobs.size(); ++i) {
                        sg.jobs[i]->result = primary.result;
                    }

                    for (TessellationJob* jobPtr : sg.jobs) {
                        int currentCount = ++completedJobs;
                        LOG_DEBUG("Finished tessellating: " + jobPtr->prototypePath.GetString() +
                                  " | Faces: " + std::to_string(jobPtr->result.faceVertexCounts.size()) +
                                  " (" + std::to_string(currentCount) + "/" + std::to_string(totalJobs) + " jobs completed globally)");
                        LOG_PROGRESS(currentCount, totalJobs, "Tessellating Geometry");
                    }
                } catch (const Standard_Failure& e) {
                    LOG_PROGRESS_DONE();
                    for (TessellationJob* jobPtr : sg.jobs) {
                        LOG_ERR("OCC exception on " + jobPtr->prototypePath.GetString() +
                                " (def index " + std::to_string(jobPtr->defIndex) + "): " + e.GetMessageString());
                        int currentCount = ++completedJobs;
                        LOG_PROGRESS(currentCount, totalJobs, "Tessellating Geometry");
                    }
                } catch (const std::exception& e) {
                    LOG_PROGRESS_DONE();
                    for (TessellationJob* jobPtr : sg.jobs) {
                        LOG_ERR("std exception on " + jobPtr->prototypePath.GetString() +
                                " (def index " + std::to_string(jobPtr->defIndex) + "): " + e.what());
                        int currentCount = ++completedJobs;
                        LOG_PROGRESS(currentCount, totalJobs, "Tessellating Geometry");
                    }
                } catch (...) {
                    LOG_PROGRESS_DONE();
                    for (TessellationJob* jobPtr : sg.jobs) {
                        LOG_ERR("Unknown exception on " + jobPtr->prototypePath.GetString() +
                                " (def index " + std::to_string(jobPtr->defIndex) + ")");
                        int currentCount = ++completedJobs;
                        LOG_PROGRESS(currentCount, totalJobs, "Tessellating Geometry");
                    }
                }
            }
        });

        LOG_PROGRESS_DONE();
    }
}