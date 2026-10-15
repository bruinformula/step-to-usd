#include <optional>
#include <vector>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/reference.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/variantSets.h>

#pragma pop_macro("Handle")

#include "CadUSD/CadUsdPipeline.h"
#include "CadUSD/Logger.h"
#include "CadUSD/UsdUtils.h"
#include "cadPrototypesAPI.h"
#include "cadTessellationAPI.h"

PXR_NAMESPACE_USING_DIRECTIVE

namespace CadUSD {

TessParams CadUsdPipeline::getTessParams(UsdPrim prim,
                                         const TessParams& defaultParams) {
    TessParams params = defaultParams;

    if (prim.HasAPI<AutolibCadPrototypesAPI>()) {
        AutolibCadPrototypesAPI protoApi(prim);
        SdfPathVector targets;
        protoApi.GetCadDefaultParamsRel().GetForwardedTargets(&targets);
        if (!targets.empty()) {
            UsdPrim targetPrim = prim.GetStage()->GetPrimAtPath(targets[0]);
            if (targetPrim.IsValid() && targetPrim.HasAPI<AutolibCadTessellationAPI>()) {
                params = getTessParams(targetPrim, params);
            }
        }
    }

    AutolibCadTessellationAPI api(prim);

    // Meshing
    

    return params;
}

std::map<SdfPath,
         TessParams>
CadUsdPipeline::resolveParams(const UsdPrim& prim,
                              const TessParams& defaultParams) {
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

    // LOG_DEBUG("Params for prim" + prim.GetPath().GetString() + ":");
    // for (const auto& [path, params] : results) {
    //     LOG_DEBUG("  " + path.GetString() + ": "
    //             + "meshLinearDeflection=" + std::to_string(params.meshLinearDeflection) + ", "
    //             + "meshAngularDeflection=" + std::to_string(params.meshAngularDeflection) + ", "
    //             + "sketchDefl=" + std::to_string(params.sketchDeflection));
    // }

    if (prim.HasAuthoredActive()) {
        prim.SetActive(initialActive);
    }
    return results;
}

std::optional<SdfReference> CadUsdPipeline::getPrototypesDefaultParams(const UsdPrim& prototypesPrim) {
    AutolibCadPrototypesAPI api(prototypesPrim);

    SdfPathVector targets;
    api.GetCadDefaultParamsRel().GetForwardedTargets(&targets);

    if (targets.empty()) {
        LOG_ERR("No default params target specified on prototypes prim. Using hardcoded defaults.");
        return std::nullopt;
    }

    UsdPrim paramsPrim = prototypesPrim.GetStage()->GetPrimAtPath(targets[0]);

    if (!paramsPrim.IsValid()) {
        LOG_ERR("Default params target " + targets[0].GetString() + " is invalid. Using hardcoded defaults.");
        return std::nullopt;
    }

    if (!paramsPrim.HasAPI<AutolibCadTessellationAPI>()) {
        LOG_ERR("Default params target " + targets[0].GetString() +
                " does not have AutolibCadTessellationAPI. Using hardcoded defaults.");
        return std::nullopt;
    }

    // We want the path *in the container stage* because WritePrims will pair this
    // with a relative path pointing back to the container stage.
    SdfReference reference("", targets[0]);

    return reference;
}
} // namespace CadUSD
