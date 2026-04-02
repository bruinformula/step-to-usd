#include <stddef.h>
#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <filesystem>
#include <string>
#include <optional>
#include <string_view>
#include <utility>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/pxr.h>
#include <pxr/usd/usd/common.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/variantSets.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/prim.h>

#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/path.h>

#include <pxr/base/work/loops.h>
#include <pxr/base/work/workTBB/loops_impl.h>

#pragma pop_macro("Handle")

#include "stepFileContainerAPI.h"
#include "stepFileContainer.h"

#include "ArgumentHandler.h"
#include "UsdStepExporter.h"
#include "StepModel.h"

PXR_NAMESPACE_USING_DIRECTIVE
namespace fs = std::filesystem;

const std::string argOptions =
    " StepConvertUsd -- Converts Step files to Usd\n"
    " Options: \n"
    "    -i, --inputUsdFile <path>                        Path to the input Usd file. \n"
    "    -p, --prim <sdfPath>                             Only tesselate the prim at this path including variants. Can be multiple paths. if left empty all prim will be tessellated\n"
    "    -v, --verbose                                    Print additional details during execution.\n"
    "    -h, --help                                       Prints this message.\n"
    "    usage: StepConvertUsd -i <path> -p <sdfPath> \n";

struct StepConvertUsdArgumentHandler : public ArgumentHandler {

    std::filesystem::path inputUsdFile;
    std::unordered_set<SdfPath, SdfPath::Hash> selectedPaths; 
    UsdStepExporter::LoggingMode verbose = UsdStepExporter::LoggingMode::NONE;

    ParseResult parse(const std::string& token, const std::string& nextToken) override {
        switch (hashString(token)) {
            case hashString("-i"):
            case hashString("--inputUsdFile"): {
                if (nextToken.empty()) goto expectOption;
                if (!inputUsdFile.empty()) goto alreadySet;
                inputUsdFile = nextToken;
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("-p"):
            case hashString("--prim"): {
                if (nextToken.empty()) goto expectOption;
                SdfPath path(nextToken);
                selectedPaths.insert(path);
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("-v"):
            case hashString("--verbose"): {
                verbose = UsdStepExporter::LoggingMode::VERBOSE;
                return SUCCESS;
            }
            case hashString("-h"):
            case hashString("--help"): {
                std::cout << argOptions << std::endl;
                return EXIT;
            }
            default: {
                std::cout << "Unrecognized command-line option: " << token << std::endl;
                return FAILURE;
            }
        }

        alreadySet: {
            std::cerr << token << " is already set!" << std::endl;
            return FAILURE;
        }
        expectOption: {
            std::cerr << "Expected another token following command-line option: " << token << std::endl; 
            return FAILURE;
        }

        std::cout << "Parsing token: " << token << " with next token: " << nextToken << std::endl;
        return FAILURE;
    }

    bool verify() const override {
        if (inputUsdFile.empty()) {
            std::cerr << "inputUsdFile is not set!" << std::endl;
            return false;
        }
        if (!std::filesystem::exists(inputUsdFile)) {
            std::cerr << "The provided input Usd file does not exist: " << inputUsdFile << std::endl;
            return false;
        }

        return true;
    }
};

int main(int argc, char** argv) {
    std::vector<std::string> tokens;
    for (int i = 1; i < argc; i++) {
        tokens.emplace_back(argv[i]);
    }
    
    StepConvertUsdArgumentHandler inputArgs;
    for (size_t i = 0; i < tokens.size(); i++) {
        const std::string& token = tokens[i];
        const std::string& nextToken = i + 1 < tokens.size() ? tokens[i + 1] : "";
        
        ArgumentHandler::ParseResult parseResult = inputArgs.parse(token, nextToken);
        switch (parseResult) {
            case ArgumentHandler::SUCCESS:
                break;
            case ArgumentHandler::SUCCESS_CONSUME_NEXT:
                i++;
                break;
            case ArgumentHandler::FAILURE:
                return 1;
            case ArgumentHandler::EXIT:
                return 0;
        }
    }
    
    if (!inputArgs.verify()) {
        std::cerr << "Input argument verification failed." << std::endl;
        return 1;
    }

    auto start = std::chrono::high_resolution_clock::now();

    UsdStageRefPtr stage = UsdStage::Open(inputArgs.inputUsdFile, UsdStage::LoadNone);
    if (!stage) {
        std::cerr << "Failed to create stage at " << inputArgs.inputUsdFile << "\n";
        return 1;
    }

    std::unordered_set<SdfAssetPath, SdfAssetPath::Hash> referencedStepAssetPaths;

    // Do a scan for all refernced Step Assets, so 
    // we can load them in parallel and cache the 
    // results to avoid redundant parsing of the same STEP file.
    for (const auto& prim : stage->TraverseAll()) {
        if (!prim.HasAPI<AutolibStepFileContainerAPI>()) continue;

        AutolibStepFileContainer container(prim);
        UsdAttribute pathAttr = container.GetStepSourceAssetAttr();

        SdfAssetPath sdfAssetPath;
        if (!pathAttr.Get(&sdfAssetPath)) {
            std::cerr << "Failed to get asset path from UsdAttribute\n";
            continue;
        }

        referencedStepAssetPaths.insert(sdfAssetPath);
    }

    std::unordered_map<SdfAssetPath, StepModel, SdfAssetPath::Hash> modelCache;

    WorkParallelForEach( referencedStepAssetPaths.begin(), referencedStepAssetPaths.end(), [&](const SdfAssetPath& assetPath) {
        std::string resolvedPath = assetPath.GetResolvedPath();

        if (resolvedPath.empty()) {
            std::cerr << "Failed to resolve path to: " << assetPath.GetAssetPath() << "\n";
            return;
        }

        std::optional<StepModel> optModel = StepModel::loadFromFile(resolvedPath);

        if (!optModel.has_value()) {
            std::cerr << "Failed to load STEP model from " << resolvedPath << "\n";
            return;
        }

        modelCache.insert_or_assign(assetPath, std::move(*optModel));
    });


    for (UsdPrim prim : stage->TraverseAll()) {
        if (!prim.HasAPI<AutolibStepFileContainerAPI>()) continue;

        AutolibStepFileContainer container(prim);
        UsdAttribute pathAttr = container.GetStepSourceAssetAttr();

        SdfAssetPath sdfAssetPath;
        if (!pathAttr.Get(&sdfAssetPath)) {
            std::cerr << "Failed to get asset path from UsdAttribute\n";
            continue;
        }

        fs::path assetPath = sdfAssetPath.GetResolvedPath();

        std::cout << "Processing STEP file: " << assetPath << "\n";

        // Load the model, using the cache to avoid re-parsing the same STEP file.
        auto iter = modelCache.find(sdfAssetPath);
        if (iter == modelCache.end()) {
            std::cerr << "Model not found in cache for asset path: " << assetPath << "\n";
            continue;
        }

        const StepModel& model = iter->second;

        //inputArgs.selectedPaths.insert(SdfPath("/Wonderful/Prototypes/rod0"));
        //inputArgs.selectedPaths.insert(SdfPath("/Wonderful/Prototypes/rod0").AppendVariantSelection("quality", "draft"));
        //inputArgs.selectedPaths.insert(SdfPath("/Wonderful").AppendVariantSelection("LOD", "high"));
        //inputArgs.selectedPaths.insert(SdfPath("/Wonderful").AppendVariantSelection("LOD", "high").AppendPath(SdfPath("rod0")));

        //for (const auto& path : selectedPaths)
        //    std::cout << "Filter path: " << path.GetString() << "\n";

        UsdStepExporter::populateUsd(model, stage, prim, inputArgs.selectedPaths, inputArgs.verbose);

        stage->Save();
    }

    stage->GetRootLayer()->Save();
    auto end = std::chrono::high_resolution_clock::now();

    std::cout << "Total Time Taken: " << std::chrono::duration<double>(end - start).count() << " seconds" << std::endl;

    return 0;
}