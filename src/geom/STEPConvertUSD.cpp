#include <iostream>
#include <optional>
#include <vector>

#include "ArgumentHandler.h"
#include "STEPModel.h"

const std::string argOptions =
    " STEPConvertUSD -- Converts STEP files to USD\n"
    " Options: \n"
    "    --inputSTEPFile <path>                                     Path to the input STEP file to convert. \n"
    "    --outputFile <path>                                        Path to the output USD file. \n"
    "    --wireframeMode <none|linear|resampledlinear|catmullrom>   Wireframe mode for visualization (default: linear). \n"
    "    --wireframeDeflection <float>                              Deflection value for wireframe curves (default: 1.0). \n"
    "    --sketchMode <none|linear|resampledlinear|catmullrom>      Sketch mode for visualization (default: linear). \n"
    "    --sketchDeflection <float>                                 Deflection value for sketch curves (default: 0.5). \n"
    "    --meshLinearDeflection <float>                             Linear deflection as fraction of bbox diagonal (default: 0.05). \n"
    "    --meshAngularDeflection <float>                            Angular deflection in radians (default: 0.35). \n"
    "    --meshMinSize <float>                                      Minimum mesh size as fraction of linear deflection (default: 0.1). \n"
    "    --lodCullingMinimumSize <float>                            Cull parts with bbox diagonal smaller than this value. \n"
    "    --defaultMeshVisibility <true|false>                       Default visibility of mesh geometry (default: true). \n"
    "    --defaultWireframeVisibility <true|false>                  Default visibility of wireframe curves (default: false). \n"
    "    --defaultSketchVisibility <true|false>                     Default visibility of sketch curves (default: false). \n"

    "    --help                                                     Prints this message.\n";

static CurveMode parseCurveMode(const std::string& s) {
    switch (hashString(s)) {
        case hashString("none"):            return CurveMode::None;
        case hashString("linear"):          return CurveMode::Linear;
        case hashString("resampledlinear"): return CurveMode::ResampledLinear;
        case hashString("catmullrom"):      return CurveMode::CatmullRom;
        default: return CurveMode::Linear;
    }
}

struct STEPConvertUSDArgumentHandler : public ArgumentHandler {

    std::filesystem::path inputSTEPFile;
    std::filesystem::path outputFile;

    STEPModel::TessParams tessParams;

    ParseResult parse(const std::string& token, const std::string& nextToken) override {

        switch (hashString(token)) {
            case hashString("--inputSTEPFile"): {
                if (nextToken.empty()) goto expectOption;
                if (!inputSTEPFile.empty()) goto alreadySet;
                inputSTEPFile = nextToken;
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--outputFile"): {
                if (nextToken.empty()) goto expectOption;
                if (!outputFile.empty()) goto alreadySet;
                outputFile = nextToken;
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--wireframeMode"): {
                if (nextToken.empty()) goto expectOption;
                tessParams.wireframeMode = parseCurveMode(nextToken);
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--wireframeDeflection"): {
                if (nextToken.empty()) goto expectOption;
                tessParams.wireframeDeflection = std::stof(nextToken);
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--sketchMode"): {
                if (nextToken.empty()) goto expectOption;
                tessParams.sketchMode = parseCurveMode(nextToken);
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
                tessParams.meshMinSize = std::stof(nextToken);
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--lodCullingMinimumSize"): {
                if (nextToken.empty()) goto expectOption;
                tessParams.lodCullingMinimumSize = std::stof(nextToken);
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--defaultMeshVisibility"): {
                if (nextToken.empty()) goto expectOption;
                tessParams.defaultMeshVisibility = (nextToken == "true");
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--defaultWireframeVisibility"): {
                if (nextToken.empty()) goto expectOption;
                tessParams.defaultWireframeVisibility = (nextToken == "true");
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--defaultSketchVisibility"): {
                if (nextToken.empty()) goto expectOption;
                tessParams.defaultSketchVisibility = (nextToken == "true");
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
        if (inputSTEPFile.empty()) {
            std::cerr << "inputSTEPFile is not set!" << std::endl;
            return false;
        }
        if (!std::filesystem::exists(inputSTEPFile)) {
            std::cerr << "The provided input STEP file does not exist: " << inputSTEPFile << std::endl;
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
    
    STEPConvertUSDArgumentHandler inputArgs;
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
        
    std::optional<STEPModel> optionalModel = STEPModel::loadFromFile(inputArgs.inputSTEPFile);
    
    if(!optionalModel.has_value()) {
        return 1;
    }

    STEPModel model = optionalModel.value();

    model.buildInstanceTree();
    
    //model.printInstanceTree();
    //model.printDefinitionShapes();
    //model.writeMeshTest(inputArgs.outputDir);

    pxr::UsdStageRefPtr stage = pxr::UsdStage::CreateNew(inputArgs.outputFile);
    if (!stage) {
        std::cerr << "Failed to create stage at " << inputArgs.outputFile << "\n";
        return 1;
    }

    model.populateUSD(stage, inputArgs.tessParams);

    stage->GetRootLayer()->Save();
    auto end = std::chrono::high_resolution_clock::now();

    std::cout << "Total Time Taken: " << std::chrono::duration<double>(end - start).count() << " seconds" << std::endl;

    return 0;
}