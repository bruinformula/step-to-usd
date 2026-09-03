#include <utility>
#include <algorithm>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <atomic>
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

#include "CadUSD/CadUsdPipeline.h"
#include "CadUSD/Logger.h"
#include "CadUSD/Tessellation/TessellationRoutine.h"

class Geom_Surface;

PXR_NAMESPACE_USING_DIRECTIVE

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

void CadUsdPipeline::tessellateGeometry(
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
    std::unordered_map<ShapeKey, int, ShapeKey::Hash> shapeComplexity;
    shapeComplexity.reserve(jobsByShape.size());

    std::vector<std::pair<ShapeKey, int>> shapeKeysWithComplexity;
    shapeKeysWithComplexity.reserve(jobsByShape.size());

    for (const auto& [key, jobs] : jobsByShape) {
        const TessellationJob* rep = jobs.front();
        const auto& defs = rep->proto->model->getDefinitionShapes();
        const TopoDS_Shape& shape = defs[rep->defIndex].second;
        int complexity = 0;
        for (TopExp_Explorer faceExp(shape, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
            complexity++;
        }
        shapeComplexity[key] = complexity;
        shapeKeysWithComplexity.emplace_back(key, complexity);
    }

    // Sort descending by complexity so heaviest jobs start first
    std::sort(
        shapeKeysWithComplexity.begin(), 
        shapeKeysWithComplexity.end(),
        [](const auto& a, const auto& b) { 
            return a.second > b.second; 
        }
    );
    
    std::vector<ShapeKey> shapeKeys;
    shapeKeys.reserve(shapeKeysWithComplexity.size());
    for (const auto& [key, complexity] : shapeKeysWithComplexity) {
        shapeKeys.push_back(key);
    }

    // Slowest-job report, printed once at the very end.
    struct JobTiming {
        SdfPath path;
        int defIndex;
        size_t sharedCount;
        double milliseconds;
    };
    std::mutex timingMutex;
    std::vector<JobTiming> jobTimings;

    // Bookkeeping for the near-completion tally report. Unlike a simple
    // "currently running" pointer, this tracks every job that hasn't
    // completed yet -- including ones still queued behind other work --
    // so once we're down to the last few jobs we can print exactly what's
    // left and how long each has been running, instead of just one path.
    struct PendingJobInfo {
        int defIndex;
        int complexity;              // face count of the underlying shape
        size_t sharedCount = 1;      // filled in once its subgroup is known
        bool active = false;
        std::chrono::steady_clock::time_point startTime;
    };
    std::mutex pendingJobsMutex;
    std::unordered_map<SdfPath, PendingJobInfo, SdfPath::Hash> pendingJobs;
    pendingJobs.reserve(tessJobs.size());
    for (TessellationJob& job : tessJobs) {
        const auto& defs = job.proto->model->getDefinitionShapes();
        const TopoDS_Shape& shape = defs[job.defIndex].second;
        ShapeKey key{shape.TShape().get()};
        pendingJobs[job.protoPath] = PendingJobInfo{
            job.defIndex, shapeComplexity.at(key)
        };
    }

    {
        LOG_SCOPED_TIMER("Parallel Tessellation of " + std::to_string(shapeKeys.size()) + " unique shapes.");
        std::atomic<int> completedJobs{0};
        const int totalJobs = static_cast<int>(tessJobs.size());

        // Once we're down to the last kNearEndThreshold jobs, periodically
        // print a tally of exactly what's still outstanding -- running or
        // merely queued -- so a long tail of one or two heavy shapes doesn't
        // look like a hang.
        constexpr int kNearEndThreshold = 10;
        constexpr auto kMonitorInterval = std::chrono::seconds(5);

        std::mutex monitorMutex;
        std::condition_variable monitorCv;
        std::atomic<bool> monitorRunning{true};
        
        std::thread monitorThread([&]() {
            std::unique_lock<std::mutex> monitorLock(monitorMutex);
            while (!monitorCv.wait_for(monitorLock, kMonitorInterval,
                                        [&] { return !monitorRunning.load(); })) {
                const int remaining = totalJobs - completedJobs.load();
                if (remaining <= 0 || remaining > kNearEndThreshold) {
                    continue;
                }
                std::vector<std::pair<SdfPath, PendingJobInfo>> snapshot;
                {
                    std::lock_guard<std::mutex> lock(pendingJobsMutex);
                    snapshot.assign(pendingJobs.begin(), pendingJobs.end());
                }

                int activeCount = 0;
                for (const auto& entry : snapshot) {
                    if (entry.second.active) activeCount++;
                }

                // Running jobs first (longest-running first), then queued
                // jobs (heaviest first) -- puts the likely bottleneck at
                // the top of the list.
                std::sort(snapshot.begin(), snapshot.end(),
                    [](const auto& a, const auto& b) {
                        if (a.second.active != b.second.active) return a.second.active > b.second.active;
                        if (a.second.active) return a.second.startTime < b.second.startTime;
                        return a.second.complexity > b.second.complexity;
                    });

                LOG_DEBUG("Tessellation tally: " + std::to_string(snapshot.size()) + " job(s) outstanding ("
                          + std::to_string(activeCount) + " running, "
                          + std::to_string(snapshot.size() - activeCount) + " queued):");

                const auto now = std::chrono::steady_clock::now();
                const size_t printCount = std::min(snapshot.size(), static_cast<size_t>(kNearEndThreshold));
                for (size_t i = 0; i < printCount; ++i) {
                    const auto& [path, info] = snapshot[i];
                    std::string status;
                    if (info.active) {
                        double elapsedSec = std::chrono::duration<double>(now - info.startTime).count();
                        status = std::to_string(elapsedSec) + "s elapsed";
                    } else {
                        status = "queued, not yet started";
                    }
                    LOG_DEBUG("  - " + path.GetString() +
                              " (def index " + std::to_string(info.defIndex) +
                              ", " + std::to_string(info.complexity) + " faces" +
                              ", shared by " + std::to_string(info.sharedCount) + " prototype path(s))" +
                              " - " + status);
                }
            }
        });

        WorkParallelForEach(shapeKeys.begin(), shapeKeys.end(), [&](const ShapeKey& key) {
            const std::vector<TessellationJob*>& jobs = jobsByShape.at(key);

            const TessellationJob* rep = jobs.front();
            const auto& defs = rep->proto->model->getDefinitionShapes();
            const TopoDS_Shape& shape = defs[rep->defIndex].second;

            std::vector<ParamSubgroup> subgroups;

            for (TessellationJob* jobPtr : jobs) {
                TessellationJob& job = *jobPtr;

                bool bTessellate = isPrototypeActiveInFilter(
                    selectedPaths, job.protoPath, job.proto->variantSetName, job.proto->variantName);

                if (!bTessellate) {
                    {
                        std::lock_guard<std::mutex> lock(pendingJobsMutex);
                        pendingJobs.erase(job.protoPath);
                    }
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

                LOG_DEBUG("Tessellating part: " + primary.protoPath.GetString() +
                          " (def index " + std::to_string(primary.defIndex) +
                          ", shared by " + std::to_string(sg.jobs.size()) + " prototype path(s))");

                {
                    const auto startTime = std::chrono::steady_clock::now();
                    std::lock_guard<std::mutex> lock(pendingJobsMutex);
                    for (TessellationJob* jobPtr : sg.jobs) {
                        auto it = pendingJobs.find(jobPtr->protoPath);
                        if (it != pendingJobs.end()) {
                            it->second.active = true;
                            it->second.startTime = startTime;
                            it->second.sharedCount = sg.jobs.size();
                        }
                    }
                }

                try {
                    const auto tStart = std::chrono::steady_clock::now();
                    primary.routine->tessellate(shape, sg.params, primary.protoPath);
                    const auto tEnd = std::chrono::steady_clock::now();
                    const double elapsedMs =
                        std::chrono::duration<double, std::milli>(tEnd - tStart).count();

                    {
                        std::lock_guard<std::mutex> lock(timingMutex);
                        jobTimings.push_back(JobTiming{
                            primary.protoPath, primary.defIndex, sg.jobs.size(), elapsedMs});
                    }

                    // Fan the computed result out to every other job that needs
                    // this (shape, params) pair.
                    for (size_t i = 1; i < sg.jobs.size(); ++i) {
                        sg.jobs[i]->routine = primary.routine;
                    }

                    {
                        std::lock_guard<std::mutex> lock(pendingJobsMutex);
                        for (TessellationJob* jobPtr : sg.jobs) {
                            pendingJobs.erase(jobPtr->protoPath);
                        }
                    }

                    for (TessellationJob* jobPtr : sg.jobs) {
                        int currentCount = ++completedJobs;
                        LOG_DEBUG("Finished tessellating: " + jobPtr->protoPath.GetString() +
                                  " (" + std::to_string(currentCount) + "/" + std::to_string(totalJobs) + " jobs completed globally)");
                        LOG_PROGRESS(currentCount, totalJobs, "Tessellating Geometry");
                    }
                } catch (const Standard_Failure& e) {
                    LOG_PROGRESS_DONE();
                    {
                        std::lock_guard<std::mutex> lock(pendingJobsMutex);
                        for (TessellationJob* jobPtr : sg.jobs) {
                            pendingJobs.erase(jobPtr->protoPath);
                        }
                    }
                    for (TessellationJob* jobPtr : sg.jobs) {
                        LOG_ERR("OCC exception on " + jobPtr->protoPath.GetString() +
                                " (def index " + std::to_string(jobPtr->defIndex) + "): " + e.GetMessageString());
                        int currentCount = ++completedJobs;
                        LOG_PROGRESS(currentCount, totalJobs, "Tessellating Geometry");
                    }
                } catch (const std::exception& e) {
                    LOG_PROGRESS_DONE();
                    {
                        std::lock_guard<std::mutex> lock(pendingJobsMutex);
                        for (TessellationJob* jobPtr : sg.jobs) {
                            pendingJobs.erase(jobPtr->protoPath);
                        }
                    }
                    for (TessellationJob* jobPtr : sg.jobs) {
                        LOG_ERR("std exception on " + jobPtr->protoPath.GetString() +
                                " (def index " + std::to_string(jobPtr->defIndex) + "): " + e.what());
                        int currentCount = ++completedJobs;
                        LOG_PROGRESS(currentCount, totalJobs, "Tessellating Geometry");
                    }
                } catch (...) {
                    LOG_PROGRESS_DONE();
                    {
                        std::lock_guard<std::mutex> lock(pendingJobsMutex);
                        for (TessellationJob* jobPtr : sg.jobs) {
                            pendingJobs.erase(jobPtr->protoPath);
                        }
                    }
                    for (TessellationJob* jobPtr : sg.jobs) {
                        LOG_ERR("Unknown exception on " + jobPtr->protoPath.GetString() +
                                " (def index " + std::to_string(jobPtr->defIndex) + ")");
                        int currentCount = ++completedJobs;
                        LOG_PROGRESS(currentCount, totalJobs, "Tessellating Geometry");
                    }
                }
            }
        });

        {
            std::lock_guard<std::mutex> monitorLock(monitorMutex);
            monitorRunning = false;
        }
        monitorCv.notify_one();
        monitorThread.join();

        LOG_PROGRESS_DONE();
    }

    // Report the slowest tessellation jobs
    {
        constexpr size_t kTopN = 10;
        const size_t reportCount = std::min(kTopN, jobTimings.size());

        std::partial_sort(
            jobTimings.begin(),
            jobTimings.begin() + reportCount,
            jobTimings.end(),
            [](const JobTiming& a, const JobTiming& b) {
                return a.milliseconds > b.milliseconds;
            });

        LOG_DEBUG("Top " + std::to_string(reportCount) + " slowest tessellation jobs:");
        for (size_t i = 0; i < reportCount; ++i) {
            const JobTiming& t = jobTimings[i];
            LOG_DEBUG(
                "  #" + std::to_string(i + 1) + ": " + t.path.GetString() +
                " (def index " + std::to_string(t.defIndex) +
                ", shared by " + std::to_string(t.sharedCount) + " prototype path(s))" +
                " - " + std::to_string(t.milliseconds) + " ms");
        }
    }
}