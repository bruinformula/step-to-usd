#include "CadUSD/Tessellation/MeshRoutine.h"
#include "CadUSD/UsdUtils.h"

namespace CadUSD {

    void MeshParams::populateParams(const AutolibCadTessellationAPI& api) {
        updateIfAuthored(api.GetCadMeshLinearDeflectionAttr(), &linearDeflection);
        updateIfAuthored(api.GetCadMeshAngularDeflectionAttr(), &angularDeflection);
        updateIfAuthored(api.GetCadMeshMinSizeAttr(), &minSize);
        updateIfAuthored(api.GetCadMeshSelfIntersectionThresholdAttr(), &selfIntersectionThreshold);
        updateIfAuthored(api.GetCadMeshMaxNumberRemeshPassesAttr(), &maxRemeshPasses);
        updateIfAuthored(api.GetCadMeshFixPrecisionAttr(), &fixPrecision);
        updateIfAuthored(api.GetCadMeshFixToleranceAttr(), &fixTolerance);
        updateIfAuthored(api.GetCadMeshEnableRepairShapeAttr(), &enableRepairPass);

        updateIfAuthored(api.GetCadMeshFixTimeoutAttr(), &fixTimeout);
        updateIfAuthored(api.GetCadMeshMeshTimeoutAttr(), &meshTimeout);
        updateIfAuthored(api.GetCadMeshRemeshTimeoutAttr(), &remeshTimeout);

        updateIfAuthored(api.GetCadMeshEnableSurfaceSubsetsAttr(), &enableSurfaceSubsets);
        updateIfAuthored(api.GetCadMeshEnableUVsAttr(), &enableUVs);
        updateIfAuthored(api.GetCadMeshEnableSurfaceIDAttr(), &enableSurfaceID);
        updateIfAuthored(api.GetCadMeshEnableIsBoundaryVertexAttr(), &enableIsBoundaryVertex);
    };

    void WireframeParams::populateParams(const AutolibCadTessellationAPI &api) {
        updateIfAuthored(api.GetCadWireframeCombineCurvesAttr(), &combineCurves);
        updateIfAuthored(api.GetCadWireframeDeflectionAttr(), &deflection);
        updateIfAuthored(api.GetCadWireframeTypeAttr(), &type);
        updateIfAuthored(api.GetCadWireframeEmbedSurfaceNormalsAttr(), &embedSurfaceNormals);
        updateIfAuthored(api.GetCadWireframePointLimitAttr(), &pointLimit);
        updateIfAuthored(api.GetCadWireframeSamplingAttr(), &sampling);
    }
}