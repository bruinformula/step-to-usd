
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

#include "stepContainerAPI.h"
#include "stepContainer.h"

#include "StepUsdPipeline.h"
#include "OpenCascadeAssembly.h"
#include "Logger.h"

PXR_NAMESPACE_USING_DIRECTIVE

const std::string argOptions =
    " StepUsdTesselate -- Meshes all StepContainer prims in a Usd scene\n"
    " Options: \n"
    "    -i, --input <path>               Path to the input Usd file. \n"
    "    -p, --prim  <sdfPath>            Only tessellate the prim at this path including variants. Can be multiple paths.\n"
    "    -q, --quiet                      Suppress all output.\n"
    "    -v, --verbose                    Prints like everything.\n"
    "    -h, --help                       Prints this message.\n\n"
    "    usage: StepUsdTesselate -i <path> [options] \n";

struct StepUsdTesselateArgs {

    enum ParseResult {
        SUCCESS,
        SUCCESS_CONSUME_NEXT,
        FAILURE,
        EXIT
    };

    std::filesystem::path inputUsdFile;
    std::filesystem::path outputUsdFile;
    std::unordered_set<SdfPath, SdfPath::Hash> selectedPaths;

    ParseResult parse(const std::string& token, const std::string& nextToken) {
        if (token == "-i" || token == "--input") {
            if (nextToken.empty()) {
                std::cerr << "Expected another token following command-line option: " << token << std::endl;
                return FAILURE;
            }
            if (!inputUsdFile.empty()) {
                std::cerr << token << " is already set!" << std::endl;
                return FAILURE;
            }
            inputUsdFile = nextToken;
            return SUCCESS_CONSUME_NEXT;
        }

       if (token == "-o" || token == "--output") {
            if (nextToken.empty()) {
                std::cerr << "Expected another token following command-line option: " << token << std::endl;
                return FAILURE;
            }
            if (!outputUsdFile.empty()) {
                std::cerr << token << " is already set!" << std::endl;
                return FAILURE;
            }
            outputUsdFile = nextToken;
            return SUCCESS_CONSUME_NEXT;
        }

        if (token == "-p" || token == "--prim") {
            if (nextToken.empty()) {
                std::cerr << "Expected another token following command-line option: " << token << std::endl;
                return FAILURE;
            }
            selectedPaths.insert(SdfPath(nextToken));
            return SUCCESS_CONSUME_NEXT;
        }

        if (token == "-q" || token == "--quiet") {
            Logger::activeLevel = Logger::NONE;
            return SUCCESS;
        }

        if (token == "-v" || token == "--verbose") {
            Logger::activeLevel = Logger::DEBUG;
            return SUCCESS;
        }

        if (token == "-h" || token == "--help") {
            std::cout << argOptions << std::endl;
            return EXIT;
        }

        // Not a recognized flag -- treat as an implicit positional input file,
        if (!token.empty() && token[0] != '-') {
            if (!inputUsdFile.empty()) {
                std::cerr << "inputUsdFile is already set! Unexpected extra argument: " << token << std::endl;
                return FAILURE;
            }
            inputUsdFile = token;
            return SUCCESS;
        }

        std::cout << "Unrecognized command-line option: " << token << std::endl;
        std::cout << argOptions << std::endl;
        return FAILURE;
    }

    bool verify() const {
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
    
    StepUsdTesselateArgs inputArgs;
    for (size_t i = 0; i < tokens.size(); i++) {
        const std::string& token = tokens[i];
        const std::string& nextToken = i + 1 < tokens.size() ? tokens[i + 1] : "";
        
        StepUsdTesselateArgs::ParseResult parseResult = inputArgs.parse(token, nextToken);
        switch (parseResult) {
            case StepUsdTesselateArgs::SUCCESS:
                break;
            case StepUsdTesselateArgs::SUCCESS_CONSUME_NEXT:
                i++;
                break;
            case StepUsdTesselateArgs::FAILURE:
                return 1;
            case StepUsdTesselateArgs::EXIT:
                return 0;
        }
    }
    
    if (!inputArgs.verify()) {
        std::cerr << "Input argument verification failed." << std::endl;
        return 1;
    }

    auto start = std::chrono::high_resolution_clock::now();

    std::optional<StepUsdPipeline> optionalStepExporter = StepUsdPipeline::create(inputArgs.inputUsdFile);

    if (!optionalStepExporter.has_value()) {
        std::cerr << "Failed to initialize StepUsdPipeline." << std::endl;
        return 1;
    }

    StepUsdPipeline stepExporter = std::move(*optionalStepExporter);

    const UsdStageRefPtr& stage = stepExporter.containerStage;

    // Search for step container prims and run populateUsd on each `containerPrim`
    for (UsdPrim prim : stage->TraverseAll()) {
        if (!prim.HasAPI<AutolibStepContainerAPI>()) continue;

        stepExporter.populateUsd(stage, prim, inputArgs.selectedPaths);
    }

    if (Logger::activeLevel == Logger::Level::INFO) {
        auto end = std::chrono::high_resolution_clock::now();
        LOG_INFO("Total Time Taken: " + std::to_string(std::chrono::duration<double>(end - start).count()) + " seconds");
    }

    return 0;
}