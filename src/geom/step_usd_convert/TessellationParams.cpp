#include <optional>
#include <vector>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/variantSets.h>
#include <pxr/usd/sdf/reference.h>
#include <pxr/usd/sdf/path.h>

#pragma pop_macro("Handle")

#include "stepTessellationAPI.h"
#include "stepFilePrototypesAPI.h"
#include "UsdStepExporter.h"
#include "Logger.h"
#include "UsdUtils.h"

PXR_NAMESPACE_USING_DIRECTIVE

TessParams UsdStepExporter::getTessParams(
    UsdPrim prim,
    const TessParams& defaultParams
) {
    TessParams params = defaultParams;

    if (prim.HasAPI<AutolibStepFilePrototypesAPI>()) {
        AutolibStepFilePrototypesAPI protoApi(prim);
        SdfPathVector targets;
        protoApi.GetStepDefaultParamsRel().GetForwardedTargets(&targets);
        if (!targets.empty()) {
            UsdPrim targetPrim = prim.GetStage()->GetPrimAtPath(targets[0]);
            if (targetPrim.IsValid() && targetPrim.HasAPI<AutolibStepTessellationAPI>()) {
                params = getTessParams(targetPrim, params);
            }
        }
    }

    AutolibStepTessellationAPI api(prim);

    // Meshing
    updateIfAuthored(api.GetStepMeshLinearDeflectionAttr(), &params.meshLinearDeflection);
    updateIfAuthored(api.GetStepMeshAngularDeflectionAttr(), &params.meshAngularDeflection);
    updateIfAuthored(api.GetStepMeshMinSizeAttr(), &params.meshMinSize);
    updateIfAuthored(api.GetStepMeshSelfIntersectionThresholdAttr(), &params.meshSelfIntersectionThreshold);
    updateIfAuthored(api.GetStepMeshMaxNumberRemeshPassesAttr(), &params.meshMaxNumberRemeshPasses);
    updateIfAuthored(api.GetStepMeshFixPrecisionAttr(), &params.meshFixPrecision);
    updateIfAuthored(api.GetStepMeshFixToleranceAttr(), &params.meshFixTolerance);

    updateIfAuthored(api.GetStepMeshFixTimeoutAttr(), &params.meshFixTimeout);
    updateIfAuthored(api.GetStepMeshMeshTimeoutAttr(), &params.meshMeshTimeout);
    updateIfAuthored(api.GetStepMeshRemeshTimeoutAttr(), &params.meshRemeshTimeout);

    // Wireframe
    updateIfAuthored(api.GetStepWireframeCombineCurvesAttr(), &params.wireframeCombineCurves);
    updateIfAuthored(api.GetStepWireframeDeflectionAttr(), &params.wireframeDeflection);
    updateIfAuthored(api.GetStepWireframeTypeAttr(), &params.wireframeMode.type);
    updateIfAuthored(api.GetStepWireframeEmbedSurfaceNormalsAttr(), &params.wireframeEmbedSurfaceNormals);
    updateIfAuthored(api.GetStepWireframeSamplingAttr(), &params.wireframeMode.sampling);

    // Sketch
    updateIfAuthored(api.GetStepSketchCombineCurvesAttr(), &params.sketchCombineCurves);
    updateIfAuthored(api.GetStepSketchDeflectionAttr(), &params.sketchDeflection);
    updateIfAuthored(api.GetStepSketchTypeAttr(), &params.sketchMode.type);
    updateIfAuthored(api.GetStepSketchEmbedSurfaceNormalsAttr(), &params.sketchEmbedSurfaceNormals);
    updateIfAuthored(api.GetStepSketchSamplingAttr(), &params.sketchMode.sampling);

    // Sketch Plane
    updateIfAuthored(api.GetStepSketchPlaneLinearDeflectionAttr(), &params.sketchPlaneLinearDeflection);
    updateIfAuthored(api.GetStepSketchPlaneAngularDeflectionAttr(), &params.sketchPlaneAngularDeflection);
    updateIfAuthored(api.GetStepSketchPlaneMinSizeAttr(), &params.sketchPlaneMinSize);
    updateIfAuthored(api.GetStepSketchPlaneCombineToleranceAttr(), &params.sketchPlaneCombineTolerance);
    updateIfAuthored(api.GetStepSketchPlaneFixPrecisionAttr(), &params.sketchPlaneFixPrecision);
    updateIfAuthored(api.GetStepSketchPlaneFixToleranceAttr(), &params.sketchPlaneFixTolerance);
    updateIfAuthored(api.GetStepSketchPlaneFixTimeoutAttr(), &params.sketchPlaneFixTimeout);
    updateIfAuthored(api.GetStepSketchPlaneMeshTimeoutAttr(), &params.sketchPlaneMeshTimeout);

    // Other
    updateIfAuthored(api.GetStepRenderPurposeThresholdAttr(), &params.renderPurposeThreshold);
    updateIfAuthored(api.GetStepMeshEnableSurfaceSubsetsAttr(), &params.meshEnableSurfaceSubsets);
    updateIfAuthored(api.GetStepMeshEnableUVsAttr(), &params.meshEnableUVs);
    updateIfAuthored(api.GetStepMeshEnableSurfaceIDAttr(), &params.meshEnableSurfaceID);
    updateIfAuthored(api.GetStepMeshEnableIsBoundaryVertexAttr(), &params.meshEnableIsBoundaryVertex);

    return params;
}

std::map<SdfPath, TessParams> UsdStepExporter::resolveParams(
    const UsdPrim& prim, 
    const TessParams& defaultParams
) {
    bool initialActive = prim.IsActive();
    if (prim.HasAuthoredActive() && !initialActive) {
        prim.SetActive(true);
    }

    std::map<SdfPath, TessParams> results;
    
    TessParams primParams = getTessParams(prim, defaultParams);
    
    std::vector<std::string> vsetNames;
    prim.GetVariantSets().GetNames(&vsetNames);

    if (vsetNames.empty()) {
        results[prim.GetPath()] = primParams;
    } else {
        // Since a prim can have multiple variant sets, need a resolution 
        // of all variant selections to fully evaluate every permutation 
        // authored on this prim.
        for (const std::string& name : vsetNames) {
            UsdVariantSet vset = prim.GetVariantSet(name);
            std::string originalSelection = vset.GetVariantSelection();
            std::vector<std::string> variantNames = vset.GetVariantNames();

            for (const std::string& variantName : variantNames) {
                vset.SetVariantSelection(variantName);
                
                // Re-evaluate params in case this variant authos new opinions
                TessParams variantParams = getTessParams(prim, primParams);
                SdfPath variantPath = prim.GetPath().AppendVariantSelection(name, variantName);
                
                results[variantPath] = variantParams;
            }
            
            // Restore original selection
            if (!originalSelection.empty()) {
                vset.SetVariantSelection(originalSelection);
            } else {
                vset.ClearVariantSelection();
            }
        }
    }
    
    //LOG_DEBUG("Params for prim" + prim.GetPath().GetString() + ":");
    //for (const auto& [path, params] : results) {
    //    LOG_DEBUG("  " + path.GetString() + ": "
    //            + "meshLinearDeflection=" + std::to_string(params.meshLinearDeflection) + ", "
    //            + "meshAngularDeflection=" + std::to_string(params.meshAngularDeflection) + ", "
    //            + "sketchDefl=" + std::to_string(params.sketchDeflection));
    //}
    

    if (prim.HasAuthoredActive()) {
        prim.SetActive(initialActive);
    }
    return results;
}

std::optional<SdfReference> UsdStepExporter::getPrototypesDefaultParams(const UsdPrim& prototypesPrim) {
    AutolibStepFilePrototypesAPI api(prototypesPrim);

    SdfPathVector targets;
    api.GetStepDefaultParamsRel().GetForwardedTargets(&targets);

    if (targets.empty()) {
        LOG_ERR("No default params target specified on prototypes prim. Using hardcoded defaults.");
        return std::nullopt;
    }

    UsdPrim paramsPrim = prototypesPrim.GetStage()->GetPrimAtPath(targets[0]);

    if (!paramsPrim.IsValid()) {
        LOG_ERR("Default params target " + targets[0].GetString() + " is invalid. Using hardcoded defaults.");
        return std::nullopt;
    }

    if (!paramsPrim.HasAPI<AutolibStepTessellationAPI>()) {
        LOG_ERR("Default params target " + targets[0].GetString() + " does not have AutolibStepTessellationAPI. Using hardcoded defaults.");
        return std::nullopt;
    }

    // We want the path *in the container stage* because WritePrims will pair this 
    // with a relative path pointing back to the container stage.
    SdfReference reference("", targets[0]);

    return reference;
}