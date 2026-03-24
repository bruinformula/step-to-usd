#include <iostream>
#include <optional>
#include <vector>

#include "ArgumentHandler.h"
#include "StepModel.h"

const std::string argOptions =
    " StepConvertUsd -- Converts Step files to Usd\n"
    " Options: \n"
    "    --inputStepFile <path>                                     Path to the input Step file to convert. \n"
    "    --outputFile <path>                                        Path to the output Usd file. \n"
    "    --config <path>                                            Path to a configuration file. \n"
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

struct StepConvertUsdVariantArgumentHandler : public ArgumentHandler {

    std::filesystem::path inputStepFile;
    std::filesystem::path outputFile;

    std::vector<std::filesystem::path> configFiles;

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
            case hashString("--config"): {
                if (nextToken.empty()) goto expectOption;
                configFiles.emplace_back(nextToken);
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

std::string trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    size_t end   = s.find_last_not_of(" \t\r\n");
    return (start == std::string::npos) ? "" : s.substr(start, end - start + 1);
}

std::optional<StepModel::VariantParams> parseConfigFile(const std::filesystem::path& configFilePath) {
    if (!std::filesystem::exists(configFilePath)) {
        std::cerr << "Config file does not exist: " << configFilePath << std::endl;
        return std::nullopt;
    }

    std::ifstream configFile(configFilePath);
    if (!configFile.is_open()) {
        std::cerr << "Failed to open config file: " << configFilePath << std::endl;
        return std::nullopt;
    }

    std::string line;

    StepModel::VariantParams variantParams;
    while (std::getline(configFile, line)) {
        // Strip comment
        auto commentPos = line.find('#');
        if (commentPos != std::string::npos)
            line = line.substr(0, commentPos);

        // Split on ':'
        auto colonPos = line.find(':');
        if (colonPos == std::string::npos)
            continue;

        std::string key = trim(line.substr(0, colonPos));
        std::string value = trim(line.substr(colonPos + 1));

        if (key.empty() || value.empty()) continue;

        switch (hashString(key)) {
            case hashString("variantSetName"):
                variantParams.variantSetName = value;
                break;
            case hashString("variantName"):
                variantParams.variantName = value;
                break;
            case hashString("outpath"):
                variantParams.outpath = value;
                break;
            case hashString("overwrite"):
                variantParams.overwrite = (value == "true");
                break;
            case hashString("meshLinearDeflection"):
                variantParams.tessParams.meshLinearDeflection = std::stof(value);
                break;
            case hashString("meshAngularDeflection"):
                variantParams.tessParams.meshAngularDeflection = std::stof(value);
                break;
            case hashString("meshMinSize"):
                variantParams.tessParams.meshMinSize = std::stof(value);
                break;
            case hashString("wireframeDeflection"):
                variantParams.tessParams.wireframeDeflection = std::stof(value);
                break;
            case hashString("sketchDeflection"):
                variantParams.tessParams.sketchDeflection = std::stof(value);
                break;
            case hashString("wireframeCurveType"):
                variantParams.tessParams.wireframeMode.type = parseCurveType(value);
                break;
            case hashString("wireframeCurveSampling"):
                variantParams.tessParams.wireframeMode.sampling = parseCurveSampling(value);
                break;
            case hashString("sketchCurveType"):
                variantParams.tessParams.sketchMode.type = parseCurveType(value);
                break;
            case hashString("sketchCurveSampling"):
                variantParams.tessParams.sketchMode.sampling = parseCurveSampling(value);
                break;
            case hashString("renderPurposeThreshold"):
                variantParams.tessParams.renderPurposeThreshold = std::stof(value);
                break;
            case hashString("defaultMeshVisibility"):
                variantParams.tessParams.defaultMeshVisibility = (value == "true");
                break;
            case hashString("defaultWireframeVisibility"):
                variantParams.tessParams.defaultWireframeVisibility = (value == "true");
                break;
            case hashString("defaultSketchVisibility"):
                variantParams.tessParams.defaultSketchVisibility = (value == "true");
                break;
            default:
                std::cerr << "Unrecognized config key: " << key << " in config file: " << configFilePath << std::endl;
        }

    }

    return variantParams;
}

int main(int argc, char** argv) {
    std::vector<std::string> tokens;
    for (int i = 1; i < argc; i++) {
        tokens.emplace_back(argv[i]);
    }
    
    StepConvertUsdVariantArgumentHandler inputArgs;
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

    std::vector<StepModel::VariantParams> variantParams;

    const std::filesystem::path& basePath = inputArgs.outputFile.parent_path();

    for (const auto& config : inputArgs.configFiles) {
        std::optional<StepModel::VariantParams> params = parseConfigFile(config);
        if (!params.has_value()) {
            std::cerr << "Failed to parse config file: " << config << std::endl;
            return 1;
        }

        params->refpath = params->outpath;  // keep the config-relative path for Usd reference
        params->outpath = basePath / params->outpath;

        variantParams.push_back(params.value());
    }

    auto start = std::chrono::high_resolution_clock::now();
        
    std::optional<StepModel> optionalModel = StepModel::loadFromFile(inputArgs.inputStepFile);
    
    if(!optionalModel.has_value()) {
        return 1;
    }

    StepModel model = optionalModel.value();

    model.buildInstanceTree();

    pxr::UsdStageRefPtr stage = pxr::UsdStage::CreateNew(inputArgs.outputFile);
    if (!stage) {
        std::cerr << "Failed to create stage at " << inputArgs.outputFile << "\n";
        return 1;
    }

    StepModel::TessParams params = {};

    model.populateVariantUsd(stage, variantParams);

    stage->GetRootLayer()->Save();
    auto end = std::chrono::high_resolution_clock::now();

    std::cout << "Total Time Taken: " << std::chrono::duration<double>(end - start).count() << " seconds" << std::endl;

    return 0;
}