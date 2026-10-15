#pragma once

#include <vector>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/reference.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/common.h>
#include <pxr/usd/usd/timeCode.h>

#include <pxr/base/gf/vec2f.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/vt/array.h>

#pragma pop_macro("Handle")

#include "OpenCascadeAssembly.h"
#include "Tessellation/TessellationRoutine.h"

PXR_NAMESPACE_USING_DIRECTIVE

namespace CadUSD {

struct StageFilterInfo {
    bool makeFresh = false;             // whole stage targeted
    bool hasSpecificPrototypes = false; // individual prototypes targeted
};

// Represent generated work units generated
// after traversing the stage, getting containers
// and flattening their variants
struct PrototypeContainer {
    std::shared_ptr<OpenCascadeAssembly> model;
    std::string variantSetName;
    std::string variantName;
    bool makeFreshStage = false;
    UsdStageRefPtr stage;          // contains the /Assembly and /Prototypes prims, subLayers contains container stage
    UsdStageRefPtr containerStage; // defins the container prim and mesh params
    double sourceToOutputScale;
};

// Expanded from the Prototype containers
struct TessellationJob {
    std::shared_ptr<PrototypeContainer> proto;
    int defIndex;
    SdfPath protoPath;
    TessParams params;
    std::shared_ptr<TessellationRoutine> routine;
};

struct CadUsdPipeline {

    // A set of globals for the generated stages
    struct PathConfig {
        SdfPath containerPrimPath;
        SdfPath assemblyPath;
        SdfPath prototypesPath;
        SdfPath partPath;
    };

    static std::optional<CadUsdPipeline> create(const fs::path& inputUsdFile);

    void populateUsd(const SdfPath& containerPrimPath,
                     const std::unordered_set<SdfPath,
                                              SdfPath::Hash> selectedInContainerPaths);

    CadUsdPipeline(UsdStageRefPtr cs,
                   std::unordered_map<CADBundleKey,
                                      OpenCascadeAssembly,
                                      CADBundleKey::Hash> mc)
        : containerStage(std::move(cs)),
          modelCache(std::move(mc)) {
    }

    PathConfig pathConfig;
    UsdStageRefPtr containerStage; // this is the original stage given in the initialization
    UsdStageRefPtr scratchStage;   // This contains all of the temp writes for variants and such

    std::unordered_map<CADBundleKey, OpenCascadeAssembly, CADBundleKey::Hash> modelCache;
    TessellationRoutine partRoutine;

  private:
    struct ProtoGeomJob {
        SdfPath protoPath;
        TessParams params;
        TessellationRoutine routine;
    };

    const OpenCascadeAssembly& getModelForProto(const PrototypeContainer& proto) const;

    bool isAssemblyActiveInFilter(const std::unordered_set<SdfPath,
                                                           SdfPath::Hash>& selectedPaths,
                                  const SdfPath& protoPath);

    bool isPrototypeActiveInFilter(const std::unordered_set<SdfPath,
                                                            SdfPath::Hash>& selectedPaths,
                                   const SdfPath& protoPath,
                                   const std::string& variantSetName,
                                   const std::string& variantName);

    void tessellateGeometry(std::vector<TessellationJob>& tessJobs,
                            const std::unordered_set<SdfPath,
                                                     SdfPath::Hash>& selectedPaths);

    bool populatePrototypeContainers(const std::unordered_set<SdfPath,
                                                              SdfPath::Hash>& selectedPaths,
                                     const std::unordered_set<SdfPath,
                                                              SdfPath::Hash>& containerVariantPaths,
                                     double outputMetersPerUnit,
                                     std::vector<PrototypeContainer>& prototypes);

    bool buildPrototypeStages(std::vector<PrototypeContainer>& prototypes,
                              const std::unordered_set<SdfPath,
                                                       SdfPath::Hash>& selectedPaths);

    bool populateTessellationJobs(const std::vector<PrototypeContainer>& prototypes,
                                  std::vector<TessellationJob>& tessJobs);

    void writePartClass(UsdStageRefPtr prototypesStage);

    void writePrototypeXforms(PrototypeContainer& proto,
                              LabelMap<SdfPath>& variantPrototypePaths);

    void writeAssemblyXforms(const PrototypeContainer& proto,
                             const std::vector<SdfPath>& paths);

    void writePrototypeGeometries(UsdStageRefPtr stage,
                                  std::vector<const TessellationJob*>& jobs,
                                  const std::unordered_set<SdfPath,
                                                           SdfPath::Hash>& selectedPaths,
                                  const std::string& variantSetName,
                                  const std::string& variantName);

    bool writePrototypeXform(UsdStageRefPtr stage,
                             const SdfPath& protoPath,
                             int defIdx);

    bool writePrototypeGeometry(UsdStageRefPtr stage,
                                const SdfPath& protoPath,
                                const TessParams& params,
                                int defIdx);

    void writeGeometry(const std::vector<std::pair<TDF_Label,
                                                   TopoDS_Shape>>& defs,
                       const LabelMap<SdfPath>& protoPaths,
                       UsdStageRefPtr targetStage,
                       const std::string& logLabel,
                       const TessParams& containerParams,
                       const std::map<SdfPath,
                                      TessParams>& paramsBank,
                       const std::unordered_set<SdfPath,
                                                SdfPath::Hash>& prototypeFilter = {});

    static std::optional<SdfReference> getPrototypesDefaultParams(const UsdPrim& containerPrim);

    static std::map<SdfPath,
                    TessParams>
    resolveParams(const UsdPrim& containerPrim,
                  const TessParams& defaultParams);

    static TessParams getTessParams(UsdPrim prim,
                                    const TessParams& defaultParams = {});
};

} // namespace CadUSD