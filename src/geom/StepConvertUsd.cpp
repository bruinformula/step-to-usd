#include <iostream>
#include <ostream>
#include <vector>
#include <chrono>
#include <filesystem>
#include <string>
#include <stddef.h>

#include <pxr/pxr.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/sdf/assetPath.h>

#include <pxr/usd/usd/common.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/payloads.h>
#include <pxr/usd/sdf/schema.h>

#include "stepTessellationAPI.h"
#include "stepFileContainerAPI.h"
#include "stepFileContainer.h"
#include "tokens.h"

#include "ArgumentHandler.h"
#include "UsdStepExporter.h"
#include "StepModel.h"

PXR_NAMESPACE_USING_DIRECTIVE
namespace fs = std::filesystem;

const std::string argOptions =
    " StepConvertUsd -- Converts Step files to Usd\n"
    " Options: \n"
    "    --inputUsdFile <path>                                      Path to the input Usd file. \n"
    "    --help                                                     Prints this message.\n";

struct StepConvertUsdArgumentHandler : public ArgumentHandler {

    std::filesystem::path inputUsdFile;

    ParseResult parse(const std::string& token, const std::string& nextToken) override {

        switch (hashString(token)) {
            case hashString("--inputUsdFile"): {
                if (nextToken.empty()) goto expectOption;
                if (!inputUsdFile.empty()) goto alreadySet;
                inputUsdFile = nextToken;
                return SUCCESS_CONSUME_NEXT;
            }
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

    pxr::UsdStageRefPtr stage = pxr::UsdStage::Open(inputArgs.inputUsdFile, UsdStage::LoadNone);
    if (!stage) {
        std::cerr << "Failed to create stage at " << inputArgs.inputUsdFile << "\n";
        return 1;
    }

    for (const auto& prim : stage->Traverse()) {
        if (prim.HasAPI<AutolibStepFileContainerAPI>()) {
            AutolibStepFileContainer container(prim);

            UsdAttribute pathAttr = container.GetStepSourceAssetAttr();

            SdfAssetPath sdfAssetPath;
            if (!pathAttr.Get(&sdfAssetPath)) {
                std::cerr << "Failed to get asset path from UsdAttribute\n";
                continue;
            }

            // Convert USD file on a new stage
            fs::path assetPath = sdfAssetPath.GetResolvedPath();

            fs::path newStagePath = assetPath.replace_extension("usda");

            SdfLayerRefPtr layer = SdfLayer::FindOrOpen(newStagePath);
            if (layer) {
                std::cout << "Layer already exists at " << newStagePath << ", clearing contents.\n";
                layer->Clear();
                layer->Save();
            } else {
                std::cout << "Creating new layer at " << newStagePath << "\n";
                layer = SdfLayer::CreateNew(newStagePath);
            }

            UsdStageRefPtr newStage = UsdStage::Open(newStagePath);
            if (!newStage) {
                std::cerr << "Failed to create new stage at " << newStagePath << "\n";
                continue;
            }
            std::cout << sdfAssetPath.GetResolvedPath() << std::endl;
            std::optional<StepModel> optionalModel = StepModel::loadFromFile(sdfAssetPath.GetResolvedPath());
            if(!optionalModel.has_value()) {
                std::cerr << "Failed to load STEP model from " << sdfAssetPath.GetResolvedPath() << "\n";
                return 1;
            }

            StepModel model = optionalModel.value();

            TessParams params;
            UsdStepExporter::populateUsdPlain(model, newStage, params);

            newStage->Save();

            UsdPayloads primPayloads = prim.GetPayloads();

            primPayloads.AddPayload(newStagePath.filename());
        }
    }

    stage->GetRootLayer()->Save();
    auto end = std::chrono::high_resolution_clock::now();

    std::cout << "Total Time Taken: " << std::chrono::duration<double>(end - start).count() << " seconds" << std::endl;

    return 0;
}