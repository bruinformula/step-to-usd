#include "CadUSD/Tessellation/MeshRoutine.h"
#include "CadUSD/UsdUtils.h"

namespace CadUSD {

    void SketchParams::populateParams(const AutolibCadTessellationAPI& api) {
        updateIfAuthored(api.GetCadSketchCombineCurvesAttr(), &combineCurves);
        updateIfAuthored(api.GetCadSketchDeflectionAttr(), &deflection);
        updateIfAuthored(api.GetCadSketchTypeAttr(), &type);
        updateIfAuthored(api.GetCadSketchEmbedSurfaceNormalsAttr(), &embedSurfaceNormals);
        updateIfAuthored(api.GetCadSketchPointLimitAttr(), &pointLimit);
    };

    void SketchPlaneParams::populateParams(const AutolibCadTessellationAPI &api) {
        updateIfAuthored(api.GetCadSketchPlaneLinearDeflectionAttr(), &linearDeflection);
        updateIfAuthored(api.GetCadSketchPlaneAngularDeflectionAttr(), &angularDeflection);
        updateIfAuthored(api.GetCadSketchPlaneMinSizeAttr(), &minSize);
        updateIfAuthored(api.GetCadSketchPlaneCombineToleranceAttr(), &combineTolerance);
        updateIfAuthored(api.GetCadSketchPlaneFixPrecisionAttr(), &fixPrecision);
        updateIfAuthored(api.GetCadSketchPlaneFixToleranceAttr(), &fixTolerance);
        updateIfAuthored(api.GetCadSketchPlaneFixTimeoutAttr(), &fixTimeout);
        updateIfAuthored(api.GetCadSketchPlaneMeshTimeoutAttr(), &meshTimeout);
    }
}