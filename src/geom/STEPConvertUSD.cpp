#include <iostream>
#include <optional>
#include <vector>

#include "ArgumentHandler.h"
#include "STEPModel.h"

struct STEPConvertUSDArgumentHandler : public ArgumentHandler {

    std::filesystem::path inputSTEPFile;
    std::filesystem::path outputDir;

    ParseResult parse(const std::string& token, const std::string& nextToken) override {

        switch (hashString(token)) {
            case hashString("--inputSTEPFile"): {
                if (nextToken.empty()) goto expectOption;
                if (!inputSTEPFile.empty()) goto alreadySet;
                inputSTEPFile = nextToken;
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--outputDir"): {
                if (nextToken.empty()) goto expectOption;
                if (!outputDir.empty()) goto alreadySet;
                outputDir = nextToken;
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--help"): {
                std::cout << "Usage: STEPConvertUSD --inputSTEPFile <path_to_step_file>" << std::endl;
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
        if (outputDir.empty()) {
            std::cerr << "outputDir is not set!" << std::endl;
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

    STEPModel::TessParams params = {};
    model.writeUSD(inputArgs.outputDir / "model.usdc", params);
    
    auto end = std::chrono::high_resolution_clock::now();

    std::cout << "Total Time Taken: " << std::chrono::duration<double>(end - start).count() << " seconds" << std::endl;

    return 0;
}