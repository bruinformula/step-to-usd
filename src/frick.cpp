#include <cassert>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <exception>
#include <filesystem>
#include <iostream>
#include <optional>
#include <pxr/usd/sdf/path.h>
#include <vector>

#include <BinXCAFDrivers.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <TDocStd_Application.hxx>
#include <TDocStd_Document.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <TDF_Label.hxx>
#include <gp_Trsf.hxx>
#include <gp_TrsfForm.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <PCDM_ReaderStatus.hxx>
#include <PCDM_StoreStatus.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Shape.hxx>
#include <Standard_Handle.hxx>
#include <BRepTools.hxx>
#include <BRepBuilderAPI_Transform.hxx>

#include "CadUSD/OpenCascadeAssembly.h"
#include "CadUSD/Logger.h"
#include "CadUSD/UsdUtils.h"

namespace occt = opencascade;
namespace fs = std::filesystem;

const std::string usageText =
    " frick -- CAD data io utility\n"
    " usage: frick <mode> [options]\n"
    "\n"
    " modes:\n"
    "    step   Read a STEP and write an XBF file.\n"
    "    brep   Read an XBF and write BREP file(s).\n"
    "\n"
    " frick step -i <path.step> [-o <path.xbf>]\n"
    "    -i, --input   <path>    Path to the input STEP file.\n"
    "    -o, --output  <path>    Path to the output XBF file. Defaults to\n"
    "                            the input path with a .xbf extension.\n"
    "\n"
    " frick brep -i <path.xbf> [-p <prim>] [-o <path>] [-r <root>] [--local]\n"
    "    -i, --input   <path>    Path to the input XBF file.\n"
    "    -p, --prim    <path>    USD-style path of a single prim to export,\n"
    "                            e.g. /Bracket__a1b2c3d4. If\n"
    "                            omitted, every leaf prim is exported.\n"
    "    -o, --output  <path>    With -p: output .brep file (default: the\n"
    "                            prim's own name, in the current\n"
    "                            directory).\n"
    "                            Without -p: output directory\n"
    "                            to fill with one .brep per prim,\n"
    "                            mirroring prim paths.\n"
    "\n"
    "    usage: frick brep assem.xbf -p /Bracket__a1b2c3d4\n"
    "           frick brep -i assem.xbf\n"
    "           frick brep -i assem.xbf -p Bracket__a1b2c3d4 -o test.brep\n";

struct CommonIOArgs {
    fs::path inputPath;
    fs::path outputPath;

    // Returns true if this token was consumed as a common flag/positional.
    // Sets consumeNext if nextToken was used up as this flag's value.
    bool parseCommon(const std::string& token, const std::string& nextToken, bool& consumeNext) {
        consumeNext = false;
        if (token == "-i" || token == "--input") {
            if (nextToken.empty()) { 
                std::cerr << "Expected a path after " << token << std::endl; 
                return false;
            }
            if (!inputPath.empty()) { 
                std::cerr << token << " is already set!" << std::endl; 
                return false;
            }
            inputPath = nextToken;
            consumeNext = true;
            return true;
        }
        if (token == "-o" || token == "--output") {
            if (nextToken.empty()) { 
                std::cerr << "Expected a path after " << token << std::endl; 
                return false;
            }
            if (!outputPath.empty()) { 
                std::cerr << token << " is already set!" << std::endl; 
                return false;
            }
            outputPath = nextToken;
            consumeNext = true;
            return true;
        }
        if (!token.empty() && token[0] != '-') {
            if (!inputPath.empty()) {
                LOG_ERR("Input is already set! Unexpected extra argument: " + token);
                return false;
            }
            inputPath = token;
            return true;
        }
        return false; // caller tries mode-specific flags
    }
};

// step mode: STEP -> XBF
struct StepArgs : public CommonIOArgs {
    bool verify() {
        if (inputPath.empty()) {
            std::cerr << "Input STEP path is not set!" << std::endl;
            return false;
        }
        if (!fs::exists(inputPath)) {
            std::cerr << "The provided input STEP file does not exist: " << inputPath << std::endl;
            return false;
        }
        if (outputPath.empty()) {
            outputPath = inputPath;
            outputPath.replace_extension("xbf");
        } else if (!outputPath.has_extension() || !outputPath.has_filename()) {
            std::cerr << "xbf output path invalid: " << outputPath << std::endl;
            return false;
        }
        return true;
    }
};

int runStepMode(const std::vector<std::string>& tokens) {
    StepArgs args;
    for (size_t i = 0; i < tokens.size(); i++) {
        const std::string& token = tokens[i];
        const std::string& nextToken = i + 1 < tokens.size() ? tokens[i + 1] : "";
        bool consumeNext = false;
        if (!args.parseCommon(token, nextToken, consumeNext)) {
            std::cerr << "Unrecognized option for 'step' mode: " << token << std::endl;
            std::cerr << usageText << std::endl;
            return 1;
        }
        if (consumeNext) i++;
    }
    if (!args.verify()) return 1;

    try {
        // Multit threaded doesn't produce stable tree ordering,
        // which causes trouble with hash logic.
        // OSD_Parallel::SetUseOcctThreads(true);
        occt::handle<TDocStd_Application> app = new TDocStd_Application();
        BinXCAFDrivers::DefineFormat(app);
        occt::handle<TDocStd_Document> doc;
        app->NewDocument("BinXCAF", doc);

        STEPCAFControl_Reader reader;
        if (reader.ReadFile(args.inputPath.c_str()) != IFSelect_RetDone) {
            LOG_ERR("Error reading Cad file");
            return 1;
        }
        if (!reader.Transfer(doc)) {
            LOG_ERR("Error transferring Cad data");
            return 1;
        }
        doc->ChangeStorageFormat("BinXCAF");
        if (app->SaveAs(doc, args.outputPath.c_str()) != PCDM_SS_OK) {
            LOG_ERR("Warning: failed to save XBF");
        }
    } catch (const Standard_Failure& e) {
        LOG_ERR("OCC exception: " + std::string(e.GetMessageString()));
        return 1;
    } catch (const std::exception& e) {
        LOG_ERR("std exception: " + std::string(e.what()));
        return 1;
    }
    return 0;
}

// brep mode: XBF -> BREP
struct BrepArgs : public CommonIOArgs {
    std::vector<std::string> primPaths;
    bool verbose = false; // TODO: make this is real mode
    bool bakeWorldTransform = true;

    bool parse(const std::string& token, const std::string& nextToken, bool& consumeNext) {
        consumeNext = false;
        if (parseCommon(token, nextToken, consumeNext)) return true;

        if (token == "-p" || token == "--prim") {
            if (nextToken.empty())
                LOG_ERR("Expected a prim path after " + token); return false;
            primPaths.push_back(nextToken);
            consumeNext = true;
            return true;
        }

        if (token == "-v" || token == "--verbose") {
            verbose = true;
            return true;
        }
        return false;
    }

    bool verify() {
        if (inputPath.empty()) {
            LOG_ERR("Input XBF path is not set!");
            return false;
        }
        if (!fs::exists(inputPath)) {
            LOG_ERR("The provided input XBF file does not exist: " + inputPath.string());
            return false;
        }

        if (primPaths.empty() && outputPath.empty()) {
            LOG_ERR("Must specific output directory for breps");
            return false;
        }
        return true;
    }
};

bool writeShapeToBrep(
    const TopoDS_Shape& localShape, 
    const gp_Trsf& worldTransform,
    bool bake, 
    const fs::path& outPath
) {
    if (!outPath.parent_path().empty()) {
        fs::create_directories(outPath.parent_path());
    }

    TopoDS_Shape shapeToWrite = localShape;
    if (bake && worldTransform.Form() != gp_Identity) {
        BRepBuilderAPI_Transform transformer(localShape, worldTransform, true);
        shapeToWrite = transformer.Shape();
    }

    if (!BRepTools::Write(shapeToWrite, outPath.c_str())) {
        LOG_ERR("Failed to write BREP: " + outPath.string());
        return false;
    }
    return true;
}

int runBrepMode(const std::vector<std::string>& tokens) {
    BrepArgs args;
    for (size_t i = 0; i < tokens.size(); i++) {
        const std::string& token = tokens[i];
        const std::string& nextToken = i + 1 < tokens.size() ? tokens[i + 1] : "";
        bool consumeNext = false;
        if (!args.parse(token, nextToken, consumeNext)) {
            LOG_ERR("Unrecognized option for 'brep' mode: " + token);
            std::cerr << usageText << std::endl;
            return 1;
        }
        if (consumeNext) i++;
    }
    if (!args.verify()) return 1;

    if (args.verbose) {
        Logger::activeLevel = Logger::Level::DEBUG;
    }

    auto assembly = OpenCascadeAssembly::loadFromFile(args.inputPath);
    if (!assembly) {
        LOG_ERR("Failed to load XBF: " + args.inputPath.string());
        return 1;
    }

    SdfPath assemblyRoot = SdfPath::AbsoluteRootPath();

    std::vector<OpenCascadeAssembly::ExportablePart> parts = assembly->getExportableLeaves(assemblyRoot);
    if (parts.empty()) {
        LOG_ERR("No leaf geometry found under root: " + assemblyRoot.GetString());
        return 1;
    }

    if (args.verbose) {
        for (const auto& p : parts) {
            LOG_DEBUG(p.path.GetString());
        }
    }

    // Single prim export 
    if (!args.primPaths.empty()) {
        for (const auto& wanted : args.primPaths) {
            auto it = std::find_if(parts.begin(), parts.end(), [&](const auto& p) {
                return p.path.GetString() == wanted;
            });
    
            if (it == parts.end()) {
                LOG_ERR("No prim found at path: " + wanted);
                std::cerr << "Prims whose path contains '" << wanted << "':" << std::endl;
                for (const auto& p : parts) {
                    if (p.path.GetString().find(wanted) != std::string::npos) {
                        std::cerr << "    " << p.path.GetString() << std::endl;
                    }
                }
                continue;
            }
    
            fs::path outPath = args.outputPath;
            if (outPath.empty()) {
                outPath = sanitizeUsdName(it->path.GetName()) + ".brep";
            }
            if (!writeShapeToBrep(it->localShape, it->worldTransform, args.bakeWorldTransform, outPath)) {
                continue;
            }
            LOG_INFO("Wrote " + outPath.string());
        }
        return 0;
    }

    // All-prims export 
    fs::path outDir = args.outputPath;
    if (outDir.empty()) {
        outDir = fs::current_path();
        LOG_WARN("Output path not give. Dumping in " + outDir.string());
    }

    int written = 0;
    for (const auto& part : parts) {
        std::string relative = part.path.GetString();
        if (!relative.empty() && relative[0] == '/') relative.erase(0, 1);
        fs::path outPath = outDir / (relative + ".brep");
        if (writeShapeToBrep(part.localShape, part.worldTransform, args.bakeWorldTransform, outPath)) {
            written++;
        }
    }

    LOG_INFO("Wrote " + std::to_string(written) + "/" + std::to_string(parts.size())
              + " BREP files to " + outDir.string());
    int success = written == static_cast<int>(parts.size()) ? 0 : 1;
    return success;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << usageText << std::endl;
        return 1;
    }

    std::string modeToken = argv[1];
    std::vector<std::string> tokens;
    for (int i = 2; i < argc; i++) 
        tokens.emplace_back(argv[i]);

    auto start = std::chrono::high_resolution_clock::now();

    int result;
    if (modeToken == "step") {
        result = runStepMode(tokens);
    } else if (modeToken == "brep") {
        result = runBrepMode(tokens);
    } else {
        LOG_ERR("Unrecognized mode: " + modeToken);
        std::cerr << usageText << std::endl;
        return 1;
    }

    auto end = std::chrono::high_resolution_clock::now();
    LOG_INFO("Total Time Taken: " + std::to_string(std::chrono::duration<double>(end - start).count()) + " seconds");

    return result;
}