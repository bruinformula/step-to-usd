
#include <stddef.h>
#include <ostream>
#include <optional>
#include <vector>
#include <chrono>
#include <filesystem>
#include <string>
#include <string_view>

#include <pxr/pxr.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/usd/common.h>
#include <pxr/usd/usd/stage.h>

#include "ArgumentHandler.h"
#include "UsdStepExporter.h"
#include "StepModel.h"

PXR_NAMESPACE_USING_DIRECTIVE

const std::string argOptions =
    " StepConvertUsdPlain -- Converts Step files to Usd\n"
    " Options: \n"
    "    --inputStepFile <path>                                     Path to the input Step file to convert. \n"
    "    --outputFile <path>                                        Path to the output Usd file. \n"
    "    --wireframeType <none|linear|catmullRom>                   Wireframe curve type (default: linear). \n"
    "    --wireframeSampling <underlying|resampled>                  Wireframe curve sampling (default: underlying). \n"
    "    --wireframeDeflection <float>                              Deflection value for wireframe curves (default: 1.0). \n"
    "    --sketchType <none|linear|resampledLinear|catmullRom>       Sketch curve type (default: linear). \n"
    "    --sketchSampling <underlying|resampled>                     Sketch curve sampling (default: underlying). \n"
    "    --sketchDeflection <float>                                 Deflection value for sketch curves (default: 0.5). \n"
    "    --meshLinearDeflection <float>                             Linear deflection as fraction of bbox diagonal (default: 0.05). \n"
    "    --meshAngularDeflection <float>                            Angular deflection in radians (default: 0.35). \n"
    "    --meshMinSize <float>                                      Minimum mesh size as fraction of linear deflection (default: 0.1). \n"
    "    --renderPurposeThreshold <float>                           Cull parts with bbox diagonal smaller than this value. \n"
    "    --invertMeshVisibility <true|false>                       Default visibility of mesh geometry (default: true). \n"
    "    --defaultWireframeVisibility <true|false>                  Default visibility of wireframe curves (default: false). \n"
    "    --defaultSketchVisibility <true|false>                     Default visibility of sketch curves (default: false). \n"

    "    --help                                                     Prints this message.\n";

static CurveSampling parseCurveSampling(const std::string& s) {
    switch (hashString(s)) {
        case hashString("underlying"): return CurveSampling::Underlying;
        case hashString("resampled"):  return CurveSampling::Resampled;
        default: return CurveSampling::Underlying;
    }
}

static CurveType parseCurveType(const std::string& s) {
    switch (hashString(s)) {
        case hashString("none"):            return CurveType::None;
        case hashString("linear"):          return CurveType::Linear;
        case hashString("catmullRom"):      return CurveType::CatmullRom;
        default: return CurveType::CatmullRom;
    }
}

struct StepConvertUsdPlainArgumentHandler : public ArgumentHandler {

    std::filesystem::path inputStepFile;
    std::filesystem::path outputFile;

    TessParams tessParams;

    ParseResult parse(const std::string& token, const std::string& nextToken) override {

        switch (hashString(token)) {
            case hashString("--inputStepFile"): {
                if (nextToken.empty()) goto expectOption;
                if (!inputStepFile.empty()) goto alreadySet;
                inputStepFile = nextToken;
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--outputFile"): {
                if (nextToken.empty()) goto expectOption;
                if (!outputFile.empty()) goto alreadySet;
                outputFile = nextToken;
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--wireframeType"): {
                if (nextToken.empty()) goto expectOption;
                tessParams.wireframeMode.type = parseCurveType(nextToken);
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--wireframeSampling"): {
                if (nextToken.empty()) goto expectOption;
                tessParams.wireframeMode.sampling = parseCurveSampling(nextToken);
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--wireframeDeflection"): {
                if (nextToken.empty()) goto expectOption;
                tessParams.wireframeDeflection = std::stof(nextToken);
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--sketchType"): {
                if (nextToken.empty()) goto expectOption;
                tessParams.sketchMode.type = parseCurveType(nextToken);
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--sketchSampling"): {
                if (nextToken.empty()) goto expectOption;
                tessParams.sketchMode.sampling = parseCurveSampling(nextToken);
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--sketchDeflection"): {
                if (nextToken.empty()) goto expectOption;
                tessParams.sketchDeflection = std::stof(nextToken);
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--meshLinearDeflection"): {
                if (nextToken.empty()) goto expectOption;
                tessParams.meshLinearDeflection = std::stof(nextToken);
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--meshAngularDeflection"): {
                if (nextToken.empty()) goto expectOption;
                tessParams.meshAngularDeflection = std::stof(nextToken);
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--meshMinSize"): {
                if (nextToken.empty()) goto expectOption;
                tessParams.meshMinSize = std::stod(nextToken);
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--renderPurposeThreshold"): {
                if (nextToken.empty()) goto expectOption;
                tessParams.renderPurposeThreshold = std::stof(nextToken);
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--selfIntersectionThreshold"): {
                if (nextToken.empty()) goto expectOption;
                tessParams.selfIntersectionThreshold = std::stod(nextToken);
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--maxNumberRemeshPasses"): {
                if (nextToken.empty()) goto expectOption;
                tessParams.maxNumberRemeshPasses = std::stoi(nextToken);
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
        if (inputStepFile.empty()) {
            std::cerr << "inputStepFile is not set!" << std::endl;
            return false;
        }
        if (!std::filesystem::exists(inputStepFile)) {
            std::cerr << "The provided input Step file does not exist: " << inputStepFile << std::endl;
            return false;
        }

        if (outputFile.empty()) {
            std::cerr << "outputFile is not set!" << std::endl;
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
    
    StepConvertUsdPlainArgumentHandler inputArgs;
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
        
    std::optional<StepModel> optionalModel = StepModel::loadFromFile(inputArgs.inputStepFile);
    
    if(!optionalModel.has_value()) {
        return 1;
    }

    StepModel model = optionalModel.value();
    
    //model.printInstanceTree();
    //model.printDefinitionShapes();
    //model.writeMeshTest(inputArgs.outputDir);

    UsdStageRefPtr stage = UsdStage::CreateNew(inputArgs.outputFile);
    if (!stage) {
        std::cerr << "Failed to create stage at " << inputArgs.outputFile << "\n";
        return 1;
    }

    UsdStepExporter::populateUsdPlain(model, stage, inputArgs.tessParams);

    std::cout << "Saving to " << inputArgs.outputFile << "...\n";
    stage->GetRootLayer()->Save();
    auto end = std::chrono::high_resolution_clock::now();

    std::cout << "Total Time Taken: " << std::chrono::duration<double>(end - start).count() << " seconds" << std::endl;

    return 0;
}