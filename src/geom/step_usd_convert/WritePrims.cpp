#include <stddef.h>
#include <iostream>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>

#include <opencascade/TDF_Label.hxx>
#include <opencascade/Quantity_Color.hxx>
#include <opencascade/TopoDS_Shape.hxx>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/pxr.h>
#include <pxr/usd/usd/common.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/modelAPI.h>
#include <pxr/usd/usd/inherits.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/references.h>
#include <pxr/usd/usd/variantSets.h>
#include <pxr/usd/usd/editContext.h>

#include <pxr/usd/sdf/changeBlock.h>
#include <pxr/usd/sdf/types.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/variantSpec.h>
#include <pxr/usd/sdf/variantSetSpec.h>

#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdGeom/scope.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>
#include <pxr/usd/usdGeom/basisCurves.h>
#include <pxr/usd/usdGeom/imageable.h>
#include <pxr/usd/usdGeom/subset.h>
#include <pxr/usd/usdGeom/primvar.h>
#include <pxr/usd/usdGeom/xformOp.h>

#include <pxr/usd/sdf/copyUtils.h>
#include <pxr/base/work/loops.h>

#include <pxr/base/vt/array.h>
#include <pxr/base/vt/types.h>
#include <pxr/base/tf/staticData.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/gf/vec3f.h>

#pragma pop_macro("Handle")

#include "stepTessellationAPI.h"

#include "UsdStepExporter.h"
#include "StepModel.h"
#include "Logger.h"

PXR_NAMESPACE_USING_DIRECTIVE

// Write CAD pat
void UsdStepExporter::writeCadPart(
    UsdStageRefPtr prototypesStage,
    const UsdPrim& prototypesPrimOnContainerStage,
    const SdfPath cadPartPath
) {
    std::optional<SdfReference> defaultParamsRef = UsdStepExporter::getPrototypesDefaultParams(prototypesPrimOnContainerStage);

    fs::path containerStagePath = fs::canonical(
        prototypesPrimOnContainerStage.GetStage()->GetRootLayer()->GetResolvedPath().GetPathString()
    );
    fs::path prototypesStagePath = fs::canonical(
        prototypesStage->GetRootLayer()->GetResolvedPath().GetPathString()
    );

    fs::path relativePath = fs::relative(containerStagePath, prototypesStagePath.parent_path());

    SdfPath containerPrimPath = prototypesPrimOnContainerStage.GetPath().GetParentPath();
    SdfPath realCadPartPath = containerPrimPath.AppendChild(cadPartPath.GetNameToken());

    UsdPrim cadPart = prototypesStage->CreateClassPrim(realCadPartPath);

    prototypesStage->GetPrimAtPath(containerPrimPath).SetSpecifier(SdfSpecifierOver);

    UsdGeomImageable(cadPart).CreateVisibilityAttr().Set(UsdGeomTokens->inherited);

    auto makeClassChild = [&](const char* name) {
        SdfPath childPath = realCadPartPath.AppendChild(TfToken(name));
        UsdPrim child = prototypesStage->DefinePrim(childPath);
        UsdGeomImageable(child).CreateVisibilityAttr().Set(UsdGeomTokens->inherited);
    };

    makeClassChild("Mesh");
    makeClassChild("Wireframe");
    makeClassChild("Sketch");
    makeClassChild("SketchPlanes");
    
    if (defaultParamsRef.has_value()) {
        cadPart.GetReferences().AddReference(
            relativePath.string(),
            defaultParamsRef->GetPrimPath(),
            defaultParamsRef->GetLayerOffset()
        );
    }
}

// Prototype Xforms
void UsdStepExporter::writePrototypeXformsInPrototypesStage(
    UsdStageRefPtr prototypesStage,
    const UsdPrim& containerPrim,
    const std::vector<std::pair<TDF_Label, TopoDS_Shape>>& defs,
    const SdfPath& prototypesPath,
    const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
    const SdfPath& containerPrimPath,
    const std::string& variantSetName,
    const std::string& variantName,
    LabelMap<SdfPath>& prototypePaths,
    bool makeFreshStage
) {
    std::unordered_map<std::string, int> protoNameCounts;
    const int total = (int)defs.size();
    int completed = 0;

    std::string logLabel = "";
    if (!variantSetName.empty()) {
        logLabel = " {" + variantSetName + "=" + variantName + "}";
    }

    std::optional<SdfReference> defaultParamsRef;
    fs::path relativePath;
    
    SdfPath cadPartPath = containerPrimPath.AppendChild(TfToken("CADPart"));

    for (int defIdx = 0; defIdx < total; defIdx++) {
        std::string rawName = getLabelName(defs[defIdx].first);
        if (rawName.empty()) {
            rawName = "Def_" + std::to_string(defIdx);
        }

        int protoCount = protoNameCounts[rawName]++;
        std::string name = sanitizeUsdName(rawName, protoCount);

        SdfPath protoPath = prototypesPath.AppendChild(TfToken(name));

        prototypePaths[defs[defIdx].first] = protoPath;

        if (!makeFreshStage && prototypesStage->GetPrimAtPath(protoPath).IsValid()) {
            completed++;
            if (Logger::activeLevel == Logger::DEBUG) {
                LOG_DEBUG("[" + std::to_string(completed) + "/" + std::to_string(total) + "] Skip existing prototype: " + protoPath.GetString());
            } else {
                LOG_PROGRESS(completed, total, "Writing prototypes " + logLabel);
            }
            continue;
        }

        UsdPrim protoPrim = prototypesStage->DefinePrim(protoPath);

        { // SdfChangeBlock
            SdfChangeBlock changeBlock;

            if (!protoPrim.IsValid()) {
                std::cerr << "writePrototypeXform: prim invalid after Define at " << protoPath << "\n";
                continue;
            }

            // Clear existing inherits before adding to avoid duplicates on re-run
            protoPrim.GetInherits().ClearInherits();
            protoPrim.GetInherits().AddInherit(cadPartPath);

            AutolibStepTessellationAPI api(protoPrim);

            api.CreateStepDefIndexAttr().Set(defIdx);
        }
        
        completed++;
        if (Logger::activeLevel == Logger::DEBUG) {
            LOG_DEBUG("[" + std::to_string(completed) + "/" + std::to_string(total) + "] Writing prototype: " + protoPath.GetString());
        } else {
            LOG_PROGRESS(completed, total, "Writing prototypes " + logLabel);
        }
    }
    LOG_PROGRESS_DONE();
}

void UsdStepExporter::writePrototypeOverridesInAssemblyStage(
    UsdStageRefPtr assemblyStage,
    const UsdPrim& containerPrim,
    LabelMap<SdfPath>& prototypePaths
) {
    LOG_SCOPED_TIMER("writePrototypeOverridesInAssemblyStage");
    const int total = (int)prototypePaths.size();
    int completed = 0;
    { // SdfChangeBlock
        SdfChangeBlock changeBlock;

        for (auto protoIter = prototypePaths.begin(); protoIter != prototypePaths.end(); ++protoIter) {
            const SdfPath& protoPath = protoIter->second;
            SdfPath assemblyProtoPath = protoPath.ReplacePrefix(SdfPath::AbsoluteRootPath(), containerPrim.GetPath());
            assemblyStage->OverridePrim(assemblyProtoPath);

            completed++;
            if (Logger::activeLevel == Logger::DEBUG) {
                LOG_DEBUG("[" + std::to_string(completed) + "/" + std::to_string(total) + "] Writing Assembly Override: " + assemblyProtoPath.GetString());
            } else {
                LOG_PROGRESS(completed, total, "Writing Prototype Overrides");
            }
        }
    }

    LOG_PROGRESS_DONE();
}

// Prototype Geometry
static void defineMeshGeometry(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessResult& r,
    const TessParams& params
) {
    UsdGeomMesh proto = UsdGeomMesh::Define(stage, protoPath.AppendChild(TfToken("Mesh")));

    if (params.enableSurfaceSubsets) {
        for (const auto& surfaceIDBounds : r.surfaceIDBounds) {
            UsdGeomSubset::Define(
                stage,
                proto.GetPath().AppendChild(TfToken("SurfaceSubset_" + std::to_string(surfaceIDBounds.surfaceID)))
            );
        }
    }
    
    UsdGeomPrimvarsAPI api(proto);
    api.CreatePrimvar(TfToken("st"), SdfValueTypeNames->TexCoord2fArray, UsdGeomTokens->faceVarying);
    api.CreatePrimvar(TfToken("surfaceID"), SdfValueTypeNames->IntArray, UsdGeomTokens->uniform);
    api.CreatePrimvar(TfToken("isBoundaryVertex"), SdfValueTypeNames->BoolArray, UsdGeomTokens->vertex);
}

static void writeMeshGeometry(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessResult& r,
    const TessParams& params
) {
    UsdGeomMesh proto(stage->GetPrimAtPath(protoPath.AppendChild(TfToken("Mesh"))));


    if (params.enableSurfaceSubsets) {
        for (const auto& surfaceIDBounds : r.surfaceIDBounds) {
            int count = surfaceIDBounds.endIdx - surfaceIDBounds.startIdx;

            VtIntArray indices(count);
            std::iota(indices.begin(), indices.end(), surfaceIDBounds.startIdx);
            
            UsdGeomSubset subset(stage->GetPrimAtPath(proto.GetPath().AppendChild(TfToken("SurfaceSubset_" + std::to_string(surfaceIDBounds.surfaceID)))));
            subset.CreateElementTypeAttr().Set(UsdGeomTokens->face);
            subset.CreateIndicesAttr().Set(indices);
            subset.CreateFamilyNameAttr().Set(TfToken("materialBind"));
        }
        
        UsdGeomSubset::SetFamilyType(proto, TfToken("materialBind"), UsdGeomTokens->partition);
    }

    proto.GetPointsAttr().Set(r.points);
    proto.GetFaceVertexCountsAttr().Set(r.faceVertexCounts);
    proto.GetFaceVertexIndicesAttr().Set(r.faceVertexIndices);
    proto.GetSubdivisionSchemeAttr().Set(UsdGeomTokens->none);
    proto.SetNormalsInterpolation(UsdGeomTokens->faceVarying);
    proto.GetNormalsAttr().Set(r.normals);

    UsdGeomPrimvarsAPI api(proto);
    if (params.enableUVs) if (UsdGeomPrimvar p = api.GetPrimvar(TfToken("st"))) p.Set(r.perSurfaceUVs);
    if (params.enableSurfaceID) if (UsdGeomPrimvar p = api.GetPrimvar(TfToken("surfaceID"))) p.Set(r.surfaceIDs);
    if (params.enableIsBoundaryVertex) if (UsdGeomPrimvar p = api.GetPrimvar(TfToken("isBoundaryVertex"))) p.Set(r.isBoundaryVertex);
}

static void defineWireframeGeometry(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessResult& r,
    const TessParams& params
) {
    SdfPath wireframePath = protoPath.AppendChild(TfToken("Wireframe"));
    UsdGeomXform::Define(stage, wireframePath);

    if (params.wireframeCombineCurves) {
        UsdGeomBasisCurves curve = UsdGeomBasisCurves::Define(
            stage, wireframePath.AppendChild(TfToken("Curves"))
        );
        if (!r.curveContinuity.empty()) {
            UsdGeomPrimvarsAPI(curve).CreatePrimvar(TfToken("continuityType"), SdfValueTypeNames->IntArray, UsdGeomTokens->uniform);
        }
    } else {
        int pointOffset = 0;
        for (int ci = 0; ci < (int)r.wireframeCounts.size(); ++ci) {
            int count = r.wireframeCounts[ci];

            UsdGeomBasisCurves curve = UsdGeomBasisCurves::Define(
                stage, wireframePath.AppendChild(TfToken("Wireframe_" + std::to_string(ci)))
            );
            
            if (ci < (int)r.curveContinuity.size()) {
                UsdGeomPrimvarsAPI(curve).CreatePrimvar(TfToken("continuityType"), SdfValueTypeNames->IntArray, UsdGeomTokens->uniform);
            }

            pointOffset += count;
        }
    }
}

static void writeWireframeGeometry(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessResult& r,
    const TessParams& params
) {
    SdfPath wireframePath = protoPath.AppendChild(TfToken("Wireframe"));

    if (params.wireframeCombineCurves) {
        UsdGeomBasisCurves curve(stage->GetPrimAtPath(wireframePath.AppendChild(TfToken("Curves"))));

        if (params.wireframeMode.type == TessParams::CurveType::Cubic) {
            curve.CreateTypeAttr().Set(UsdGeomTokens->cubic);
            curve.CreateBasisAttr().Set(UsdGeomTokens->catmullRom);
        } else {
            curve.CreateTypeAttr().Set(UsdGeomTokens->linear);
        }
        curve.CreateWrapAttr().Set(UsdGeomTokens->nonperiodic);
        curve.GetPointsAttr().Set(r.curvePoints);
        curve.GetCurveVertexCountsAttr().Set(r.wireframeCounts);
        
        curve.CreateWidthsAttr().Set(VtArray<float>{0.1f});
        UsdGeomPrimvarsAPI(curve).CreatePrimvar(TfToken("widths"), SdfValueTypeNames->FloatArray, UsdGeomTokens->constant).Set(VtArray<float>{0.1f});
        
        curve.GetDisplayColorAttr().Set(VtArray<GfVec3f>{{0.8f, 0.8f, 0.8f}});

        if (!r.curveContinuity.empty()) {
            UsdGeomPrimvarsAPI api(curve);
            if (UsdGeomPrimvar p = api.GetPrimvar(TfToken("continuityType"))) p.Set(r.curveContinuity);
        }
    } else {
        int pointOffset = 0;
        for (int ci = 0; ci < (int)r.wireframeCounts.size(); ++ci) {
            int count = r.wireframeCounts[ci];

            VtArray<GfVec3f> pts(
                r.curvePoints.begin() + pointOffset,
                r.curvePoints.begin() + pointOffset + count
            );

            UsdGeomBasisCurves curve(stage->GetPrimAtPath(wireframePath.AppendChild(TfToken("Wireframe_" + std::to_string(ci)))));

            if (params.wireframeMode.type == TessParams::CurveType::Cubic) {
                curve.CreateTypeAttr().Set(UsdGeomTokens->cubic);
                curve.CreateBasisAttr().Set(UsdGeomTokens->catmullRom);
            } else {
                curve.CreateTypeAttr().Set(UsdGeomTokens->linear);
            }
            curve.CreateWrapAttr().Set(UsdGeomTokens->nonperiodic);
            curve.GetPointsAttr().Set(pts);
            curve.GetCurveVertexCountsAttr().Set(VtIntArray{count});
            curve.CreateWidthsAttr().Set(VtArray<float>(count, 0.1f));
            curve.GetDisplayColorAttr().Set(VtArray<GfVec3f>{{0.8f, 0.8f, 0.8f}});

            if (ci < (int)r.curveContinuity.size()) {
                UsdGeomPrimvarsAPI api(curve);
                if (UsdGeomPrimvar p = api.GetPrimvar(TfToken("continuityType"))) p.Set(VtIntArray{r.curveContinuity[ci]});
            }

            pointOffset += count;
        }
    }
}

static void defineSketchGeometry(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessResult& r,
    const TessParams& params
) {
    SdfPath sketchPath = protoPath.AppendChild(TfToken("Sketch"));
    UsdGeomScope::Define(stage, sketchPath);

    if (params.sketchCombineCurves) {
        UsdGeomBasisCurves::Define(
            stage, sketchPath.AppendChild(TfToken("Curves"))
        );
    } else {
        int pointOffset = 0;
        for (int ci = 0; ci < (int)r.sketchCounts.size(); ++ci) {
            int count = r.sketchCounts[ci];

            UsdGeomBasisCurves sketchCurve = UsdGeomBasisCurves::Define(
                stage, sketchPath.AppendChild(TfToken("Curve_" + std::to_string(ci)))
            );

            pointOffset += count;
        }
    }
}

static void writeSketchGeometry(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessResult& r,
    const TessParams& params
) {
    SdfPath sketchPath = protoPath.AppendChild(TfToken("Sketch"));

    if (params.sketchCombineCurves) {
        UsdGeomBasisCurves sketchCurve(stage->GetPrimAtPath(sketchPath.AppendChild(TfToken("Curves"))));

        if (params.sketchMode.type == TessParams::CurveType::Cubic) {
            sketchCurve.CreateTypeAttr().Set(UsdGeomTokens->cubic);
            sketchCurve.CreateBasisAttr().Set(UsdGeomTokens->catmullRom);
        } else {
            sketchCurve.CreateTypeAttr().Set(UsdGeomTokens->linear);
        }
        sketchCurve.CreateWrapAttr().Set(UsdGeomTokens->nonperiodic);
        sketchCurve.GetPointsAttr().Set(r.sketchPoints);
        sketchCurve.GetCurveVertexCountsAttr().Set(r.sketchCounts);
        
        sketchCurve.CreateWidthsAttr().Set(VtArray<float>{0.1f});
        UsdGeomPrimvarsAPI(sketchCurve).CreatePrimvar(TfToken("widths"), SdfValueTypeNames->FloatArray, UsdGeomTokens->constant).Set(VtArray<float>{0.1f});
        
        sketchCurve.GetDisplayColorAttr().Set(VtArray<GfVec3f>{{0.4f, 0.7f, 1.0f}});
    } else {
        int pointOffset = 0;
        for (int ci = 0; ci < (int)r.sketchCounts.size(); ++ci) {
            int count = r.sketchCounts[ci];

            VtArray<GfVec3f> pts(
                r.sketchPoints.begin() + pointOffset,
                r.sketchPoints.begin() + pointOffset + count
            );

            UsdGeomBasisCurves sketchCurve(stage->GetPrimAtPath(sketchPath.AppendChild(TfToken("Curve_" + std::to_string(ci)))));

            if (params.sketchMode.type == TessParams::CurveType::Cubic) {
                sketchCurve.CreateTypeAttr().Set(UsdGeomTokens->cubic);
                sketchCurve.CreateBasisAttr().Set(UsdGeomTokens->catmullRom);
            } else {
                sketchCurve.CreateTypeAttr().Set(UsdGeomTokens->linear);
            }
            sketchCurve.CreateWrapAttr().Set(UsdGeomTokens->nonperiodic);
            sketchCurve.GetPointsAttr().Set(pts);
            sketchCurve.GetCurveVertexCountsAttr().Set(VtIntArray{count});
            sketchCurve.CreateWidthsAttr().Set(VtArray<float>(count, 0.1f));
            sketchCurve.GetDisplayColorAttr().Set(VtArray<GfVec3f>{{0.4f, 0.7f, 1.0f}});

            pointOffset += count;
        }
    }
}

static void defineSketchPlaneGeometry(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessResult& r,
    const TessParams& params
) {
    SdfPath sketchPlanesPath = protoPath.AppendChild(TfToken("SketchPlanes"));
    UsdGeomScope::Define(stage, sketchPlanesPath);
    for (size_t pi = 0; pi < r.sketchPlaneBounds.size(); ++pi) {
        UsdGeomMesh::Define(stage, sketchPlanesPath.AppendChild(TfToken("Plane_" + std::to_string(pi))));
    }
}

static void writeSketchPlaneGeometry(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessResult& r,
    const TessParams& params
) {
    SdfPath sketchPlanesPath = protoPath.AppendChild(TfToken("SketchPlanes"));
    for (size_t pi = 0; pi < r.sketchPlaneBounds.size(); ++pi) {
        const TessResult::SketchPlaneBounds& b = r.sketchPlaneBounds[pi];

        if (b.pointStart < 0 || b.pointCount <= 0 ||
            b.faceCountStart < 0 || b.faceCountCount <= 0 ||
            b.faceIndexStart < 0 || b.faceIndexCount <= 0 ||
            b.normalStart < 0 || b.normalCount <= 0) {
            continue;
        }

        if (static_cast<size_t>(b.pointStart + b.pointCount) > r.sketchPlanePoints.size() ||
            static_cast<size_t>(b.faceCountStart + b.faceCountCount) > r.sketchPlaneFaceVertexCounts.size() ||
            static_cast<size_t>(b.faceIndexStart + b.faceIndexCount) > r.sketchPlaneFaceVertexIndices.size() ||
            static_cast<size_t>(b.normalStart + b.normalCount) > r.sketchPlaneNormals.size()) {
            continue;
        }

        VtArray<GfVec3f> points(
            r.sketchPlanePoints.begin() + b.pointStart,
            r.sketchPlanePoints.begin() + b.pointStart + b.pointCount
        );
        VtArray<int> counts(
            r.sketchPlaneFaceVertexCounts.begin() + b.faceCountStart,
            r.sketchPlaneFaceVertexCounts.begin() + b.faceCountStart + b.faceCountCount
        );
        VtArray<int> indices(
            r.sketchPlaneFaceVertexIndices.begin() + b.faceIndexStart,
            r.sketchPlaneFaceVertexIndices.begin() + b.faceIndexStart + b.faceIndexCount
        );
        for (int& idx : indices) {
            idx -= b.pointStart;
        }
        VtArray<GfVec3f> normals(
            r.sketchPlaneNormals.begin() + b.normalStart,
            r.sketchPlaneNormals.begin() + b.normalStart + b.normalCount
        );

        UsdGeomMesh mesh(stage->GetPrimAtPath(sketchPlanesPath.AppendChild(TfToken("Plane_" + std::to_string(pi)))));
        mesh.GetPointsAttr().Set(points);
        mesh.GetFaceVertexCountsAttr().Set(counts);
        mesh.GetFaceVertexIndicesAttr().Set(indices);
        mesh.GetSubdivisionSchemeAttr().Set(UsdGeomTokens->none);
        mesh.SetNormalsInterpolation(UsdGeomTokens->faceVarying);
        mesh.GetNormalsAttr().Set(normals);
        mesh.GetDisplayColorAttr().Set(VtArray<GfVec3f>{{0.55f, 0.8f, 1.0f}});
    }
}

void UsdStepExporter::writePrototypeGeometries(
    UsdStageRefPtr stage,
    const std::vector<ProtoGeomJob>& inJobs,
    const std::unordered_set<SdfPath, SdfPath::Hash>& selectedPaths,
    const SdfPath& containerPrimPath,
    const std::string& variantSetName,
    const std::string& variantName
) {
    const int total = (int)inJobs.size();
    if (total == 0) return;

    // Create a mutable copy of the jobs vector 
    // so we can sort it for load balancing
    std::vector<ProtoGeomJob> jobs = inJobs;
    std::sort(jobs.begin(), jobs.end(), [](const ProtoGeomJob& a, const ProtoGeomJob& b) {
        // Sort descending by number of points to process larger geometries first
        return a.result.points.size() > b.result.points.size();
    });

    std::atomic<int> completed(0);
    std::string logLabel = "";
    if (!variantSetName.empty()) {
        logLabel = " {" + variantSetName + "=" + variantName + "}";
    }

    struct ThreadResult {
        int startIdx;
        int endIdx;
        SdfLayerRefPtr layer;
    };

    std::vector<ThreadResult> threadResults;
    std::mutex resultsMutex;

    const int grainSize = 1;

    WorkParallelForN(total, [&](int startIdx, int endIdx) {
        SdfLayerRefPtr threadLayer = SdfLayer::CreateAnonymous();
        UsdStageRefPtr threadStage = UsdStage::Open(threadLayer);

        // Define Prims first 
        // their properties are later populated in the SdfChangeBlock 
        for (int i = startIdx; i < endIdx; i++) {
            const SdfPath& protoPath = jobs[i].protoPath;
            const TessResult& r = jobs[i].result;
            const TessParams& params = jobs[i].params;

            if (!selectedPaths.empty() && !isPrototypeActiveInFilter(selectedPaths, containerPrimPath, variantSetName, variantName, protoPath)) {
                continue;
            }

            UsdGeomScope protoXform;
                
            auto variantSelection = protoPath.GetVariantSelection();
            if (!variantSelection.first.empty()) {
                SdfPath baseProtoPath = protoPath.StripAllVariantSelections();
                
                UsdPrim basePrim = threadStage->GetPrimAtPath(baseProtoPath);
                if (!basePrim) {
                    protoXform = UsdGeomScope::Define(threadStage, baseProtoPath);
                    basePrim = protoXform.GetPrim();
                } else {
                    protoXform = UsdGeomScope(basePrim);
                }
                
                UsdVariantSet vset = basePrim.GetVariantSets().AddVariantSet(variantSelection.first);
                vset.AddVariant(variantSelection.second);
                vset.SetVariantSelection(variantSelection.second);
                
                UsdEditContext ctx(vset.GetVariantEditContext());
                
                threadStage->RemovePrim(baseProtoPath.AppendChild(TfToken("Mesh")));
                threadStage->RemovePrim(baseProtoPath.AppendChild(TfToken("Wireframe")));
                threadStage->RemovePrim(baseProtoPath.AppendChild(TfToken("Sketch")));
                threadStage->RemovePrim(baseProtoPath.AppendChild(TfToken("SketchPlanes")));

                if (r.renderOnly) {
                    UsdGeomImageable(protoXform.GetPrim()).CreatePurposeAttr();
                }

                if (!r.points.empty()) {
                    defineMeshGeometry(threadStage, baseProtoPath, r, params);
                }

                if (!r.wireframeCounts.empty()) {
                    defineWireframeGeometry(threadStage, baseProtoPath, r, params);
                }

                if (!r.sketchCounts.empty()) {
                    defineSketchGeometry(threadStage, baseProtoPath, r, params);
                }

                if (!r.sketchPlaneFaceVertexIndices.empty()) {
                    defineSketchPlaneGeometry(threadStage, baseProtoPath, r, params);
                }

            } else {
                protoXform = UsdGeomScope::Get(threadStage, protoPath);
                if (!protoXform) {
                    protoXform = UsdGeomScope::Define(threadStage, protoPath);
                }
                
                threadStage->RemovePrim(protoPath.AppendChild(TfToken("Mesh")));
                threadStage->RemovePrim(protoPath.AppendChild(TfToken("Wireframe")));
                threadStage->RemovePrim(protoPath.AppendChild(TfToken("Sketch")));
                threadStage->RemovePrim(protoPath.AppendChild(TfToken("SketchPlanes")));

                if (r.renderOnly) {
                    UsdGeomImageable(protoXform.GetPrim()).CreatePurposeAttr();
                }

                if (!r.points.empty()) {
                    defineMeshGeometry(threadStage, protoPath, r, params);
                }

                if (!r.wireframeCounts.empty()) {
                    defineWireframeGeometry(threadStage, protoPath, r, params);
                }

                if (!r.sketchCounts.empty()) {
                    defineSketchGeometry(threadStage, protoPath, r, params);
                }

                if (!r.sketchPlaneFaceVertexIndices.empty()) {
                    defineSketchPlaneGeometry(threadStage, protoPath, r, params);
                }
            }
        }

        { // SdfChangeBlock
            SdfChangeBlock block;
            for (int i = startIdx; i < endIdx; i++) {
                const SdfPath& protoPath = jobs[i].protoPath;
                const TessResult& r = jobs[i].result;
                const TessParams& params = jobs[i].params;

                if (!selectedPaths.empty() && !isPrototypeActiveInFilter(selectedPaths, containerPrimPath, variantSetName, variantName, protoPath)) {
                    int c = ++completed;
                    if (Logger::activeLevel == Logger::DEBUG) {
                        LOG_DEBUG("[" + std::to_string(c) + "/" + std::to_string(total) + "] Skip geometry (filtered): " + protoPath.GetString());
                    } else {
                        LOG_PROGRESS(c, total, "Writing geometry" + logLabel);
                    }
                    continue;
                }

                auto variantSelection = protoPath.GetVariantSelection();
                SdfPath writeProtoPath = variantSelection.first.empty() ? protoPath : protoPath.StripAllVariantSelections();
                
                // If variant selected, we need the edit context 
                // active to author values inside the variant
                std::optional<UsdEditContext> ctx;
                if (!variantSelection.first.empty()) {
                    UsdPrim basePrim = threadStage->GetPrimAtPath(writeProtoPath);
                    UsdVariantSet vset = basePrim.GetVariantSets().AddVariantSet(variantSelection.first);
                    vset.SetVariantSelection(variantSelection.second);
                    ctx.emplace(vset.GetVariantEditContext());
                }

                UsdGeomScope protoXform = UsdGeomScope::Get(threadStage, writeProtoPath);
                
                if (r.renderOnly) {
                    UsdGeomImageable(protoXform.GetPrim()).CreatePurposeAttr().Set(UsdGeomTokens->render);
                }

                if (!r.points.empty()) {
                    writeMeshGeometry(threadStage, writeProtoPath, r, params);
                }

                if (!r.wireframeCounts.empty()) {
                    writeWireframeGeometry(threadStage, writeProtoPath, r, params);
                }

                if (!r.sketchCounts.empty()) {
                    writeSketchGeometry(threadStage, writeProtoPath, r, params);
                }

                if (!r.sketchPlaneFaceVertexIndices.empty()) {
                    writeSketchPlaneGeometry(threadStage, writeProtoPath, r, params);
                }

                int c = ++completed;
                if (Logger::activeLevel == Logger::DEBUG) {
                    LOG_DEBUG("[" + std::to_string(c) + "/" + std::to_string(total) + "] Writing geometry: " + protoPath.GetString());
                } else {
                    LOG_PROGRESS(c, total, "Writing geometry" + logLabel);
                }
            }
        } // SdfChangeBlock
        
        std::lock_guard<std::mutex> resLock(resultsMutex);
        threadResults.push_back({startIdx, endIdx, threadLayer});
    }, grainSize);

    // Merge back on the main stage

    // There are two passes here to handle variant and non-variant geometry separately 
    // SdfCopy doesn't handle copying into variant bodies, so geometry under variants 
    // is written directly to the main stage in a separate pass

    { // SdfChangeBlock
        SdfChangeBlock changeBlock;
        SdfLayerHandle mainLayer = stage->GetRootLayer();

        // pre-create variant specs in mainLayer so the variant bodies exist
        for (const auto& res : threadResults) {
            if (!res.layer) continue;
            for (size_t i = res.startIdx; i < res.endIdx; i++) {
                const SdfPath& protoPath = jobs[i].protoPath;
                auto variantSelection = protoPath.GetVariantSelection();
                if (variantSelection.first.empty()) continue; // skip variants 

                SdfPath baseProtoPath = protoPath.StripAllVariantSelections();
                SdfPrimSpecHandle primSpec = mainLayer->GetPrimAtPath(baseProtoPath);
                if (!primSpec) continue;

                SdfVariantSetSpecHandle vsetSpec;
                auto vsetProxy = primSpec->GetVariantSets();
                auto vsetIt = vsetProxy.find(variantSelection.first);
                if (vsetIt == vsetProxy.end()) {
                    vsetSpec = SdfVariantSetSpec::New(primSpec, variantSelection.first);
                    primSpec->GetVariantSetNameList().Prepend(variantSelection.first);
                } else {
                    vsetSpec = vsetIt->second;
                }

                auto variantProxy = vsetSpec->GetVariants();
                if (variantProxy.find(variantSelection.second) == variantProxy.end()) {
                    SdfVariantSpec::New(vsetSpec, variantSelection.second);
                }
            }
        }

        // write the non variants
        for (const auto& res : threadResults) {
            if (!res.layer) continue;
            for (size_t i = res.startIdx; i < res.endIdx; i++) {
                const SdfPath& protoPath = jobs[i].protoPath;
                if (!selectedPaths.empty() && !isPrototypeActiveInFilter(selectedPaths, containerPrimPath, variantSetName, variantName, protoPath)) {
                    continue; // Skip filtered
                }
                if (!protoPath.GetVariantSelection().first.empty()) continue; // only non variants

                SdfPath copyPath = protoPath.GetVariantSelection().first.empty() ? protoPath : protoPath.StripAllVariantSelections();
                
                SdfPrimSpecHandle srcSpec = res.layer->GetPrimAtPath(copyPath);
                SdfPrimSpecHandle dstSpec = mainLayer->GetPrimAtPath(copyPath);
                if (srcSpec && dstSpec) {
                    dstSpec->SetTypeName(srcSpec->GetTypeName());
                    for (const auto& prop : srcSpec->GetProperties()) {
                        SdfCopySpec(res.layer, prop->GetPath(), mainLayer, prop->GetPath());
                    }
                    for (const auto& child : srcSpec->GetNameChildren()) {
                        SdfCopySpec(res.layer, child->GetPath(), mainLayer, child->GetPath());
                    }
                } else {
                    SdfCopySpec(res.layer, copyPath, mainLayer, copyPath);
                }
            }
        }
    } // SdfChangeBlock

    // Variant geometry written to main stage
    for (const auto& res : threadResults) {
        if (!res.layer) continue;
        for (size_t i = res.startIdx; i < res.endIdx; i++) {
            const SdfPath& protoPath = jobs[i].protoPath;
            if (!selectedPaths.empty() &&
                !isPrototypeActiveInFilter(selectedPaths, containerPrimPath, variantSetName, variantName, protoPath)) {
                continue;
            }

            auto variantSelection = protoPath.GetVariantSelection();
            if (variantSelection.first.empty()) continue;

            const TessResult& r = jobs[i].result;
            const TessParams& params = jobs[i].params;
            SdfPath baseProtoPath = protoPath.StripAllVariantSelections();

            UsdPrim basePrim = stage->GetPrimAtPath(baseProtoPath);
            if (!basePrim) {
                LOG_ERR("Missing base prim at " + baseProtoPath.GetString() + "\n");
                continue;
            }

            UsdVariantSet vset = basePrim.GetVariantSets().GetVariantSet(variantSelection.first);
            vset.SetVariantSelection(variantSelection.second);
            UsdEditContext ctx(vset.GetVariantEditContext());

            // Clear any previously written geometry under this variant
            stage->RemovePrim(baseProtoPath.AppendChild(TfToken("Mesh")));
            stage->RemovePrim(baseProtoPath.AppendChild(TfToken("Wireframe")));
            stage->RemovePrim(baseProtoPath.AppendChild(TfToken("Sketch")));
            stage->RemovePrim(baseProtoPath.AppendChild(TfToken("SketchPlanes")));

            if (r.renderOnly) {
                UsdPrim p = stage->GetPrimAtPath(baseProtoPath);
                if (p) {
                    UsdGeomImageable(p).CreatePurposeAttr().Set(UsdGeomTokens->render);
                }
            }

            bool hasPoints = !r.points.empty();
            bool hasWireframe = !r.wireframeCounts.empty();
            bool hasSketch = !r.sketchCounts.empty();
            bool hasSketchPlanes = !r.sketchPlaneFaceVertexIndices.empty();

            if (hasPoints) defineMeshGeometry(stage, baseProtoPath, r, params);
            if (hasWireframe) defineWireframeGeometry(stage, baseProtoPath, r, params);
            if (hasSketch) defineSketchGeometry(stage, baseProtoPath, r, params);
            if (hasSketchPlanes) defineSketchPlaneGeometry(stage, baseProtoPath, r, params);

            {
                SdfChangeBlock changeBlock;
                if (hasPoints) writeMeshGeometry(stage, baseProtoPath, r, params);
                if (hasWireframe) writeWireframeGeometry(stage, baseProtoPath, r, params);
                if (hasSketch) writeSketchGeometry(stage, baseProtoPath, r, params);
                if (hasSketchPlanes) writeSketchPlaneGeometry(stage, baseProtoPath, r, params);
            }

        }
    }

    LOG_PROGRESS_DONE();
}

// Assembly Xforms
void UsdStepExporter::writeAssemblyXforms(
    UsdStageRefPtr stage, 
    const SdfPath& containerPrimPath,
    const std::vector<StepModel::PartNode>& partNodes,
    const std::vector<SdfPath>& paths, 
    const LabelMap<SdfPath>& prototypePaths
) {
    LOG_SCOPED_TIMER("writeAssemblyXforms (" + std::to_string(partNodes.size()) + " nodes)");
    
    // pre compute which instances have children
    std::vector<bool> hasChildren(partNodes.size(), false);
    for (size_t i = 0; i < partNodes.size(); i++) {
        if (partNodes[i].parentIdx != -1)
            hasChildren[partNodes[i].parentIdx] = true;
    }
    // Define all xform nodes, wire references, and author transforms
    const int total = (int)partNodes.size();
    int completed = 0;
    for (size_t i = 0; i < partNodes.size(); i++) {
        const StepModel::PartNode& node = partNodes[i];
        UsdGeomXform xform = UsdGeomXform::Define(stage, paths[i]);
        if (!xform) {
            std::cerr << "[" << i << "] Failed to define Xform at " << paths[i] << "\n";
            completed++;
            if (Logger::activeLevel == Logger::DEBUG) {
                LOG_DEBUG("[" + std::to_string(completed) + "/" + std::to_string(total) + "] Failed to write Assembly Xform for " + paths[i].GetString());
            } else {
                LOG_PROGRESS(completed, total, "Writing Assembly");
            }
            continue;
        }
        {
            SdfChangeBlock changeBlock;
            // Usd composes the full world transform later
            xform.AddTransformOp().Set(trsfToGfMatrix(node.localTransform));
            if (node.type == StepModel::PartNodeType::Leaf) {
                auto protoIter = prototypePaths.find(node.definitionLabel);
                if (protoIter == prototypePaths.end()) {
                    completed++;
                    if (Logger::activeLevel == Logger::DEBUG) {
                        LOG_DEBUG("[" + std::to_string(completed) + "/" + std::to_string(total) + "] Skip missing prototype Assembly Xform: " + paths[i].GetString());
                    } else {
                        LOG_PROGRESS(completed, total, "Writing Assembly");
                    }
                    continue;
                }

                SdfPath assemblyProtoPath = protoIter->second.ReplacePrefix(SdfPath::AbsoluteRootPath(), containerPrimPath);

                xform.GetPrim().GetReferences().AddInternalReference(assemblyProtoPath);
                UsdModelAPI(xform.GetPrim()).SetKind(TfToken("component"));
                if (!hasChildren[i]) {
                    xform.GetPrim().SetInstanceable(true);
                }
                if (node.color.has_value()) {
                    VtArray<GfVec3f> displayColor = {{
                        static_cast<float>(node.color->Red()),
                        static_cast<float>(node.color->Green()),
                        static_cast<float>(node.color->Blue())
                    }};
                    UsdAttribute colorAttr = xform.GetPrim().CreateAttribute(
                        TfToken("primvars:displayColor"),
                        SdfValueTypeNames->Color3fArray,
                        false
                    );
                    colorAttr.Set(displayColor);
                }
                if (!node.visible) {
                    UsdGeomImageable(xform.GetPrim())
                        .CreateVisibilityAttr()
                        .Set(UsdGeomTokens->invisible);
                }
            } else {
                UsdModelAPI(xform.GetPrim()).SetKind(TfToken("assembly"));
            }
        } // SdfChangeBlock
        completed++;
        if (Logger::activeLevel == Logger::DEBUG) {
            LOG_DEBUG("[" + std::to_string(completed) + "/" + std::to_string(total) + "] Writing Assembly: " + paths[i].GetString());
        } else {
            LOG_PROGRESS(completed, total, "Writing Assembly");
        }
    }
    LOG_PROGRESS_DONE();
}