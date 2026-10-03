
#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <filesystem>
#include <string>
#include <optional>
#include <utility>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/pxr.h>
#include <pxr/usd/usd/common.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/prim.h>

#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/path.h>

#include <pxr/base/work/loops.h>
#include <pxr/base/work/workTBB/loops_impl.h>

#pragma pop_macro("Handle")

#include "cadContainerAPI.h"
#include "cadContainer.h"

#include "CadUSD/CadUsdPipeline.h"
#include "CadUSD/OpenCascadeAssembly.h"
#include "CadUSD/Logger.h"

PXR_NAMESPACE_USING_DIRECTIVE

struct CadUsdTesselateArgs {

    std::filesystem::path inputUsdFile;
    std::unordered_set<SdfPath, SdfPath::Hash> selectedPaths;

    bool parse(const std::string& token, const std::string& nextToken, bool& consumeNext);

    bool verify();
};