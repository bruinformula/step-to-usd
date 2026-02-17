#pragma once 

#include <string>
#include <iostream>
#include <filesystem>

constexpr uint32_t hashString(std::string_view s) {
    uint32_t h = 2166136261u;
    for (char c : s)
        h = (h ^ c) * 16777619u;
    return h;
}


struct ArgumentHandler {// TODO: duplicate from OSL Compiler.h figure out different build later

    enum ParseResult {
        SUCCESS,
        SUCCESS_CONSUME_NEXT,
        FAILURE,
        EXIT
    };

    virtual ParseResult parse(const std::string& token, const std::string& nextToken) = 0;

    virtual bool verify() const = 0;
};

struct USDStepReaderArgumentHandler : public ArgumentHandler {

    std::filesystem::path inputStepFile;
    std::filesystem::path outputUsdFile;

    ParseResult parse(const std::string& token, const std::string& nextToken) override {

        switch (hashString(token)) {
            case hashString("--inputStepFile"): {
                if (nextToken.empty()) goto expectOption;
                if (!inputStepFile.empty()) goto alreadySet;
                inputStepFile = nextToken;
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--outputUsdFile"): {
                if (nextToken.empty()) goto expectOption;
                if (!outputUsdFile.empty()) goto alreadySet;
                outputUsdFile = nextToken;
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--help"): {
                std::cout << "Usage: USDStepReader --inputStepFile <path_to_step_file> --outputUsdFile <path_to_output_usd_file>" << std::endl;
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
        if (outputUsdFile.empty()) {
            std::cerr << "outputUsdFile is not set!" << std::endl;
            return false;
        }
        if (!std::filesystem::exists(inputStepFile)) {
            std::cerr << "The provided input STEP file does not exist: " << inputStepFile << std::endl;
            return false;
        }
        if (std::filesystem::exists(outputUsdFile)) {
            std::cerr << "The provided output USD file already exists: " << outputUsdFile << std::endl;
            return false;
        }
        return true;
    }
};