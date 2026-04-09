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
#include "Logger.h"

PXR_NAMESPACE_USING_DIRECTIVE
namespace fs = std::filesystem;

const std::string argOptions =
    " StepConvertUsd -- Converts a Usd file containing StepFileContainers\n"
    " Options: \n"
    "    -i, --inputUsdFile <path>        Path to the input Usd file. \n"
    "    -p, --prim <sdfPath>             Only tessellate the prim at this path including variants. Can be multiple paths.\n"
    "    -q, --quiet                      Suppress all output.\n"
    "    -v, --verbose                    Prints like everything.\n"
    "    -h, --help                       Prints this message.\n\n"
    "    usage: StepConvertUsd -i <path> [options] \n";

struct StepConvertUsdArgumentHandler : public ArgumentHandler {

    std::filesystem::path inputUsdFile;
    std::unordered_set<SdfPath, SdfPath::Hash> selectedPaths; 

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
            case hashString("-q"):
            case hashString("--quiet"): {
                Logger::activeLevel = Logger::NONE;
                return SUCCESS;
            }
            case hashString("-v"):
            case hashString("--verbose"): {
                Logger::activeLevel = Logger::DEBUG;
                return SUCCESS;
            }
            case hashString("-h"):
            case hashString("--help"): {
                std::cout << argOptions << std::endl;
                return EXIT;
            }
            default: {
                std::cout << "Unrecognized command-line option: " << token << std::endl;
                std::cout << argOptions << std::endl;
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

    std::optional<UsdStepExporter> optionalStepExporter = UsdStepExporter::create(inputArgs.inputUsdFile);

    if (!optionalStepExporter.has_value()) {
        std::cerr << "Failed to initialize UsdStepExporter." << std::endl;
        return 1;
    }

    UsdStepExporter stepExporter = std::move(*optionalStepExporter);

    const UsdStageRefPtr& stage = stepExporter.containerStage;
    const std::unordered_map<SdfAssetPath, StepModel, SdfAssetPath::Hash>& modelCache = stepExporter.modelCache;

    // Search for step container prims and run populateUsd on each `containerPrim`
    for (UsdPrim prim : stage->TraverseAll()) {
        if (!prim.HasAPI<AutolibStepFileContainerAPI>()) continue;

        AutolibStepFileContainer container(prim);
        UsdAttribute pathAttr = container.GetStepSourceAssetAttr();
        
        SdfAssetPath sdfAssetPath;
        if (!pathAttr.Get(&sdfAssetPath)) {
            LOG_ERR("Failed to get asset path from UsdAttribute");
            continue;
        }

        fs::path assetPath = sdfAssetPath.GetResolvedPath();

        LOG_INFO("Processing STEP file: " + assetPath.string());

        // Load the model, using the cache to avoid re-parsing the same STEP file.
        auto iter = modelCache.find(sdfAssetPath);
        if (iter == modelCache.end()) {
            LOG_ERR("Model not found in cache for asset path: " + assetPath.string());
            continue;
        }

        UsdStepExporter::populateUsd(iter->second, stage, prim, inputArgs.selectedPaths);
    }

    stage->GetRootLayer()->Save();

    if (Logger::activeLevel == Logger::Level::INFO) {
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "Total Time Taken: " << std::chrono::duration<double>(end - start).count() << " seconds" << std::endl;
    }

    return 0;
}