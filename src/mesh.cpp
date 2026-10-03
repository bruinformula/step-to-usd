
#include <iostream>
#include <unordered_set>
#include <filesystem>
#include <string>

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

#include "CadUSD/Logger.h"

#include "mesh.h"

PXR_NAMESPACE_USING_DIRECTIVE

const std::string usageText =
    " frack -- Meshes all CadContainer prims in a Usd scene\n"
    " Options: \n"
    "    -i, --input <path>               Path to the input Usd file. \n"
    "    -p, --prim  <sdfPath>            Only tessellate the prim at this path including variants. Can be multiple paths.\n"
    "    -q, --quiet                      Suppress all output.\n"
    "    -v, --verbose                    Prints like everything.\n"
    "    -h, --help                       Prints this message.\n\n"
    "    usage: frack -i <path> [options] \n";

CadUsdTesselateArgs::ParseResult CadUsdTesselateArgs::parse(const std::string& token, const std::string& nextToken) {
    if (token == "-i" || token == "--input") {
        if (nextToken.empty()) {
            std::cerr << "Expected another token following command-line option: " << token << std::endl;
        return ParseResult::FAILURE;
        }
        if (!inputUsdFile.empty()) {
            std::cerr << token << " is already set!" << std::endl;
            return ParseResult::FAILURE;
        }
        inputUsdFile = nextToken;
        return ParseResult::SUCCESS_CONSUME_NEXT;
    }

    if (token == "-p" || token == "--prim") {
        if (nextToken.empty()) {
            std::cerr << "Expected another token following command-line option: " << token << std::endl;
            return ParseResult::FAILURE;
        }
        selectedPaths.insert(SdfPath(nextToken));
        return ParseResult::SUCCESS_CONSUME_NEXT;
    }

    if (token == "-q" || token == "--quiet") {
        Logger::activeLevel = Logger::NONE;
        return ParseResult::SUCCESS;
    }

    if (token == "-v" || token == "--verbose") {
        Logger::activeLevel = Logger::DEBUG;
        return ParseResult::SUCCESS;
    }

    if (token == "-h" || token == "--help") {
        std::cout << usageText << std::endl;
        return ParseResult::EXIT;
    }

    // Not a recognized flag -- treat as an implicit positional input file,
    if (!token.empty() && token[0] != '-') {
        if (!inputUsdFile.empty()) {
            std::cerr << "inputUsdFile is already set! Unexpected extra argument: " << token << std::endl;
            return ParseResult::FAILURE;
        }
        inputUsdFile = token;
        return ParseResult::SUCCESS;
    }

    std::cout << "Unrecognized command-line option: " << token << std::endl;
    return ParseResult::FAILURE;
}

bool CadUsdTesselateArgs::verify() {
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
