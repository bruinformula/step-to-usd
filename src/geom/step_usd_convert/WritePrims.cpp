#include <stddef.h>
#include <iostream>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <vector>

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

#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>
#include <pxr/usd/usdGeom/basisCurves.h>
#include <pxr/usd/usdGeom/imageable.h>
#include <pxr/usd/usdGeom/subset.h>
#include <pxr/usd/usdGeom/primvar.h>
#include <pxr/usd/usdGeom/xformOp.h>

#include <pxr/base/vt/array.h>
#include <pxr/base/vt/types.h>
#include <pxr/base/tf/staticData.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/gf/vec3f.h>

#pragma pop_macro("Handle")

#include "stepTessellationAPI.h"

#include "UsdStepExporter.h"
#include "StepModel.h"

PXR_NAMESPACE_USING_DIRECTIVE

// Write CAD pat
void UsdStepExporter::writeCadPart(
    UsdStageRefPtr prototypesStage,
    const UsdPrim& rootPrim,
    const SdfPath cadPartPath
) {
    std::optional<SdfReference> defaultParamsRef = UsdStepExporter::getPrototypesDefaultParams(rootPrim);

    fs::path rootStagePath = fs::canonical(
        rootPrim.GetStage()->GetRootLayer()->GetResolvedPath().GetPathString()
    );
    fs::path prototypesStagePath = fs::canonical(
        prototypesStage->GetRootLayer()->GetResolvedPath().GetPathString()
    );

    fs::path relativePath = fs::relative(rootStagePath, prototypesStagePath.parent_path());

    UsdPrim cadPart = prototypesStage->CreateClassPrim(cadPartPath);
    UsdGeomImageable(cadPart).CreateVisibilityAttr().Set(UsdGeomTokens->inherited);

    auto makeClassChild = [&](const char* name) {
        SdfPath childPath = cadPartPath.AppendChild(TfToken(name));
        UsdPrim child = prototypesStage->DefinePrim(childPath);
        UsdGeomImageable(child).CreateVisibilityAttr().Set(UsdGeomTokens->inherited);
    };

    makeClassChild("Mesh");
    makeClassChild("Wireframe");
    makeClassChild("Sketch");
    
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
    const UsdPrim& rootPrim,
    const std::vector<std::pair<TDF_Label, TopoDS_Shape>>& defs,
    const SdfPath& prototypesPath,
    const std::unordered_set<SdfPath, SdfPath::Hash>& prototypesFilter,
    LabelMap<SdfPath>& prototypePaths,
    const std::string& logLabel,
    bool makeFreshStage
) {
    std::unordered_map<std::string, int> protoNameCounts;
    const int total = (int)defs.size();
    int completed = 0;

    std::optional<SdfReference> defaultParamsRef;
    fs::path relativePath;
    
    SdfPath cadPartPath("/CADPart");

    for (int defIdx = 0; defIdx < total; defIdx++) {
        std::string rawName = getLabelName(defs[defIdx].first);
        if (rawName.empty()) {
            rawName = "Def_" + std::to_string(defIdx);
        }

        int protoCount = protoNameCounts[rawName]++;
        std::string name = sanitizeUsdName(rawName, protoCount);

        SdfPath protoPath = prototypesPath.AppendChild(TfToken(name));

        prototypePaths[defs[defIdx].first] = protoPath;

        if (!prototypesFilter.empty() && prototypesFilter.count(protoPath)) {
            prototypesStage->RemovePrim(protoPath);
        }

        if (!makeFreshStage && prototypesStage->GetPrimAtPath(protoPath).IsValid()) {
            std::cerr << "\r[" << ++completed << "/" << total << "] Writing prototypes " << logLabel << "..." << std::flush;
            continue;
        }

        UsdGeomXform protoXformPrim = UsdGeomXform::Define(prototypesStage, protoPath);

        UsdPrim protoPrim = protoXformPrim.GetPrim();

        if (!protoPrim.IsValid()) {
            std::cerr << "writePrototypeXform: prim invalid after Define at " << protoPath << "\n";
            continue;
        }

        { // SdfChangeBlock
            SdfChangeBlock changeBlock;

            // Clear existing inherits before adding to avoid duplicates on re-run
            protoPrim.GetInherits().ClearInherits();
            protoPrim.GetInherits().AddInherit(cadPartPath);

            AutolibStepTessellationAPI api(protoPrim);

            api.CreateStepDefIndexAttr().Set(defIdx);
        }
        
        std::cerr << "\r[" << ++completed << "/" << total << "] Writing prototypes " << logLabel << "..." << std::flush;
    }
    std::cerr << "\n";
}

void UsdStepExporter::writePrototypeOverridesInAssemblyStage(
    UsdStageRefPtr assemblyStage,
    const UsdPrim& rootPrim,
    LabelMap<SdfPath>& prototypePaths
) {
    const int total = (int)prototypePaths.size();
    int completed = 0;

    for (auto protoIter = prototypePaths.begin(); protoIter != prototypePaths.end(); ++protoIter) {
        const SdfPath& protoPath = protoIter->second;
        SdfPath assemblyProtoPath = protoPath.ReplacePrefix(SdfPath::AbsoluteRootPath(), rootPrim.GetPath());
        assemblyStage->OverridePrim(assemblyProtoPath);
        std::cerr << "\r[" << ++completed << "/" << total << "] Writing Prototype Overrides..." << std::flush;
    }

    std::cerr << "\n";
}

// Prototype Geometry
static void writeMeshGeometry(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessResult& r,
    const TessParams& params
) {
    UsdGeomMesh proto = UsdGeomMesh::Define(stage, protoPath.AppendChild(TfToken("Mesh")));

    for (const auto& surfaceIDBounds : r.surfaceIDBounds) {
        int count = surfaceIDBounds.endIdx - surfaceIDBounds.startIdx;

        VtIntArray indices(count);
        std::iota(indices.begin(), indices.end(), surfaceIDBounds.startIdx);

        UsdGeomSubset::CreateGeomSubset(
            proto,
            TfToken("SurfaceSubset_" + std::to_string(surfaceIDBounds.surfaceID)),
            UsdGeomTokens->face,
            indices,
            TfToken("materialBind"),
            UsdGeomTokens->nonOverlapping
        );
    }

    {
        SdfChangeBlock changeBlock;
        proto.GetPointsAttr().Set(r.points);
        proto.GetFaceVertexCountsAttr().Set(r.faceVertexCounts);
        proto.GetFaceVertexIndicesAttr().Set(r.faceVertexIndices);
        proto.GetSubdivisionSchemeAttr().Set(UsdGeomTokens->none);
        proto.SetNormalsInterpolation(UsdGeomTokens->faceVarying);
        proto.GetNormalsAttr().Set(r.normals);

        UsdGeomPrimvarsAPI api(proto);

        api.CreatePrimvar(TfToken("st"), SdfValueTypeNames->TexCoord2fArray, UsdGeomTokens->faceVarying)
            .Set(r.perSurfaceUVs);
        api.CreatePrimvar(TfToken("surfaceID"), SdfValueTypeNames->IntArray, UsdGeomTokens->uniform)
            .Set(r.surfaceIDs);
        api.CreatePrimvar(TfToken("isBoundaryVertex"), SdfValueTypeNames->BoolArray, UsdGeomTokens->vertex)
            .Set(r.isBoundaryVertex);
    }
}

static void writeWireframeGeometry(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessResult& r,
    const TessParams& params
) {
    SdfPath wireframePath = protoPath.AppendChild(TfToken("Wireframe"));
    UsdGeomXform::Define(stage, wireframePath);

    int pointOffset = 0;
    for (int ci = 0; ci < (int)r.wireframeCounts.size(); ++ci) {
        int count = r.wireframeCounts[ci];

        VtArray<GfVec3f> pts(
            r.curvePoints.begin() + pointOffset,
            r.curvePoints.begin() + pointOffset + count
        );

        UsdGeomBasisCurves curve = UsdGeomBasisCurves::Define(
            stage, wireframePath.AppendChild(TfToken("Wireframe_" + std::to_string(ci)))
        );

        {
            SdfChangeBlock changeBlock;
            if (params.wireframeMode.type == CurveType::CatmullRom) {
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
                UsdGeomPrimvarsAPI(curve)
                    .CreatePrimvar(TfToken("continuityType"), SdfValueTypeNames->IntArray, UsdGeomTokens->uniform)
                    .Set(VtIntArray{r.curveContinuity[ci]});
            }
        }

        pointOffset += count;
    }
}

static void writeSketchGeometry(
    UsdStageRefPtr stage,
    const SdfPath& protoPath,
    const TessResult& r,
    const TessParams& params
) {
    SdfPath sketchPath = protoPath.AppendChild(TfToken("Sketch"));
    UsdGeomXform::Define(stage, sketchPath);

    int pointOffset = 0;
    for (int ci = 0; ci < (int)r.sketchCounts.size(); ++ci) {
        int count = r.sketchCounts[ci];

        VtArray<GfVec3f> pts(
            r.sketchPoints.begin() + pointOffset,
            r.sketchPoints.begin() + pointOffset + count
        );

        UsdGeomBasisCurves sketchCurve = UsdGeomBasisCurves::Define(
            stage, sketchPath.AppendChild(TfToken("Sketch_" + std::to_string(ci)))
        );

        {
            SdfChangeBlock changeBlock;
            if (params.sketchMode.type == CurveType::CatmullRom) {
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
        }

        pointOffset += count;
    }
}

void UsdStepExporter::writePrototypeGeometries(
    UsdStageRefPtr stage,
    const std::vector<ProtoGeomJob>& jobs,
    const std::string& logLabel,
    const std::unordered_set<SdfPath, SdfPath::Hash>& prototypesFilter
) {
    const int total = (int)jobs.size();
    int completed = 0;

    for (int i = 0; i < total; i++) {
        const SdfPath& protoPath = jobs[i].protoPath;
        const TessResult& r = jobs[i].result;
        const TessParams& params = jobs[i].params;

        if (!prototypesFilter.empty() && !prototypesFilter.count(protoPath)) {
            std::cerr << "\r[" << ++completed << "/" << total << "] Writing geometry " << logLabel << "..." << std::flush;
            continue;
        }

        UsdGeomXform protoXform;
        
        auto variantSelection = protoPath.GetVariantSelection();
        if (!variantSelection.first.empty()) {
            // Strip variant selection to get the base prim path
            SdfPath baseProtoPath = protoPath.StripAllVariantSelections();
            
            UsdPrim basePrim = stage->GetPrimAtPath(baseProtoPath);
            if (!basePrim) {
                protoXform = UsdGeomXform::Define(stage, baseProtoPath);
                basePrim = protoXform.GetPrim();
            } else {
                protoXform = UsdGeomXform(basePrim);
            }
            
            // Ensure the variant set exists
            UsdVariantSet vset = basePrim.GetVariantSets().AddVariantSet(variantSelection.first);
            vset.AddVariant(variantSelection.second);
            vset.SetVariantSelection(variantSelection.second);
            
            // Switch to the edit context of this variant to write geometry
            UsdEditContext ctx(vset.GetVariantEditContext());
            
            if (r.renderOnly) {
                UsdGeomImageable(protoXform.GetPrim()).CreatePurposeAttr().Set(UsdGeomTokens->render);
            }

            if (!r.points.empty()) {
                writeMeshGeometry(stage, baseProtoPath, r, params);
            }

            if (!r.wireframeCounts.empty()) {
                writeWireframeGeometry(stage, baseProtoPath, r, params);
            }

            if (!r.sketchCounts.empty()) {
                writeSketchGeometry(stage, baseProtoPath, r, params);
            }

        } else {
            protoXform = UsdGeomXform::Get(stage, protoPath);
            if (!protoXform) { // fallback
                protoXform = UsdGeomXform::Define(stage, protoPath);
            }

            if (r.renderOnly) {
                UsdGeomImageable(protoXform.GetPrim()).CreatePurposeAttr().Set(UsdGeomTokens->render);
            }

            if (!r.points.empty()) {
                writeMeshGeometry(stage, protoPath, r, params);
            }

            if (!r.wireframeCounts.empty()) {
                writeWireframeGeometry(stage, protoPath, r, params);
            }

            if (!r.sketchCounts.empty()) {
                writeSketchGeometry(stage, protoPath, r, params);
            }
        }

        std::cerr << "\r[" << ++completed << "/" << total << "] Writing geometry " << logLabel << "..." << std::flush;
    }
    std::cerr << "\n";
}

// Assembly Xforms
void UsdStepExporter::writeAssemblyXforms(
    UsdStageRefPtr stage, 
    const SdfPath& rootPrimPath,
    const std::vector<StepModel::PartNode>& partNodes,
    const std::vector<SdfPath>& paths, 
    const LabelMap<SdfPath>& prototypePaths
) {
    
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
            std::cerr << "\r[" << ++completed << "/" << total << "] Writing Assembly..." << std::flush;
            continue;
        }
        {
            SdfChangeBlock changeBlock;
            // Usd composes the full world transform later
            xform.AddTransformOp().Set(trsfToGfMatrix(node.localTransform));
            if (node.type == StepModel::PartNodeType::Leaf) {
                auto protoIter = prototypePaths.find(node.definitionLabel);
                if (protoIter == prototypePaths.end()) {
                    std::cerr << "\r[" << ++completed << "/" << total << "] Writing Assembly..." << std::flush;
                    continue;
                }

                SdfPath assemblyProtoPath = protoIter->second.ReplacePrefix(SdfPath::AbsoluteRootPath(), rootPrimPath);

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
        std::cerr << "\r[" << ++completed << "/" << total << "] Writing Assembly..." << std::flush;
    }
    std::cerr << "\n";
}