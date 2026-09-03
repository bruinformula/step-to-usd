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
#include <TDataStd_Name.hxx>
#include <TCollection_ExtendedString.hxx>
#include <gp_Trsf.hxx>
#include <gp_TrsfForm.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <PCDM_ReaderStatus.hxx>
#include <PCDM_StoreStatus.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Shape.hxx>
#include <Standard_Handle.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include "CadUSD/OpenCascadeAssembly.h"
#include "CadUSD/Logger.h"
#include "CadUSD/UsdUtils.h"

namespace occt = opencascade;
namespace fs = std::filesystem;

const std::string usageText =
    " frick -- CAD data io utility\n"
    " usage: frick convert -i <input> [-o <output>] [-p <prim>] [-v]\n"
    "\n"
    " The following conversions are supported:\n"
    "    .step, .stp   STEP -> XBF\n"
    "    .brep         BREP -> XBF\n"
    "    .xbf          XBF  -> BREP\n"
    "\n"
    "    -i, --input   <path>    Path to the input file (.step, .brep, or .xbf).\n"
    "    -o, --output  <path>    Path to the output file/directory. If omitted:\n"
    "                              STEP/BREP -> XBF : input path with a .xbf extension\n"
    "                              XBF -> BREP, -p given   : the prim's own name, in\n"
    "                                the current directory\n"
    "                              XBF -> BREP, -p omitted : current directory, filled\n"
    "                                with one .brep per prim, mirroring prim paths\n"
    "    -p, --prim    <path>    (XBF -> BREP only) USD-style path of a prim to\n"
    "                            export, e.g. /Bracket__a1b2c3d4. Can be passed more\n"
    "                            than once. If omitted, every leaf prim is exported.\n"
    "    -v, --verbose            Enable debug logging.\n"
    "\n"
    " examples:\n"
    "    frick convert -i part.step\n"
    "    frick convert -i part.brep -o part.xbf\n"
    "    frick convert -i assem.xbf\n"
    "    frick convert -i assem.xbf -p /Bracket__a1b2c3d4 -o bracket.brep\n";

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

struct ConvertArgs : public CommonIOArgs {
    std::vector<std::string> primPaths;
    bool verbose = false;
    bool bakeWorldTransform = true;

    bool parse(const std::string& token, const std::string& nextToken, bool& consumeNext) {
        consumeNext = false;
        if (parseCommon(token, nextToken, consumeNext)) return true;
        if (token == "-p" || token == "--prim") {
            if (nextToken.empty()) {
                LOG_ERR("Expected a prim path after " + token);
                return false;
            }
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
};

enum class ConversionKind {
    StepToXbf,
    BrepToXbf,
    XbfToBrep,
    Unknown
};

std::string lowerExtension(const fs::path& p) {
    std::string ext = p.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(),
                    [](unsigned char c) { return std::tolower(c); });
    return ext;
}

ConversionKind detectConversionKind(const fs::path& inputPath) {
    std::string ext = lowerExtension(inputPath);
    if (ext == ".step" || ext == ".stp") return ConversionKind::StepToXbf;
    if (ext == ".brep") return ConversionKind::BrepToXbf;
    if (ext == ".xbf") return ConversionKind::XbfToBrep;
    return ConversionKind::Unknown;
}

bool verifyArgs(ConvertArgs& args, ConversionKind kind) {
    if (args.inputPath.empty()) {
        LOG_ERR("Input path is not set!");
        return false;
    }
    if (!fs::exists(args.inputPath)) {
        LOG_ERR("The provided input file does not exist: " + args.inputPath.string());
        return false;
    }
    switch (kind) {
        case ConversionKind::StepToXbf:
        case ConversionKind::BrepToXbf: {
            if (!args.primPaths.empty()) {
                LOG_WARN("-p/--prim is ignored when converting to XBF");
            }
            if (args.outputPath.empty()) {
                args.outputPath = args.inputPath;
                args.outputPath.replace_extension("xbf");
            } else if (!args.outputPath.has_extension() || !args.outputPath.has_filename()) {
                LOG_ERR("xbf output path invalid: " + args.outputPath.string());
                return false;
            }
            return true;
        }
        case ConversionKind::XbfToBrep: {
            if (args.primPaths.empty() && args.outputPath.empty()) {
                LOG_ERR("Must specify an output directory for breps when exporting all prims");
                return false;
            }
            return true;
        }
        case ConversionKind::Unknown:
        default:
            LOG_ERR("Unrecognized input extension '" + args.inputPath.extension().string() +
                     "'. Supported input extensions are .step, .stp, .brep, .xbf");
            return false;
    }
}

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

// STEP -> XBF
int convertStepToXbf(const fs::path& inputPath, const fs::path& outputPath) {
    // Multi-threaded read doesn't produce stable tree ordering,
    // which causes trouble with hash logic.
    // OSD_Parallel::SetUseOcctThreads(true);
    occt::handle<TDocStd_Application> app = new TDocStd_Application();
    BinXCAFDrivers::DefineFormat(app);
    occt::handle<TDocStd_Document> doc;
    app->NewDocument("BinXCAF", doc);

    STEPCAFControl_Reader reader;
    if (reader.ReadFile(inputPath.c_str()) != IFSelect_RetDone) {
        LOG_ERR("Error reading STEP file: " + inputPath.string());
        return 1;
    }
    if (!reader.Transfer(doc)) {
        LOG_ERR("Error transferring STEP data: " + inputPath.string());
        return 1;
    }

    doc->ChangeStorageFormat("BinXCAF");
    if (!outputPath.parent_path().empty()) {
        fs::create_directories(outputPath.parent_path());
    }
    if (app->SaveAs(doc, outputPath.c_str()) != PCDM_SS_OK) {
        LOG_ERR("Failed to save XBF: " + outputPath.string());
        return 1;
    }
    LOG_INFO("Wrote " + outputPath.string());
    return 0;
}

// BREP -> XBF
int convertBrepToXbf(const fs::path& inputPath, const fs::path& outputPath) {
    TopoDS_Shape shape;
    BRep_Builder builder;
    if (!BRepTools::Read(shape, inputPath.c_str(), builder)) {
        LOG_ERR("Error reading BREP file: " + inputPath.string());
        return 1;
    }
    if (shape.IsNull()) {
        LOG_ERR("BREP file contained no shape: " + inputPath.string());
        return 1;
    }

    occt::handle<TDocStd_Application> app = new TDocStd_Application();
    BinXCAFDrivers::DefineFormat(app);
    occt::handle<TDocStd_Document> doc;
    app->NewDocument("BinXCAF", doc);

    occt::handle<XCAFDoc_ShapeTool> shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    TDF_Label shapeLabel = shapeTool->AddShape(shape, /*makeAssembly*/ false);
    TDataStd_Name::Set(shapeLabel, TCollection_ExtendedString(inputPath.stem().string().c_str()));

    doc->ChangeStorageFormat("BinXCAF");
    if (!outputPath.parent_path().empty()) {
        fs::create_directories(outputPath.parent_path());
    }
    if (app->SaveAs(doc, outputPath.c_str()) != PCDM_SS_OK) {
        LOG_ERR("Failed to save XBF: " + outputPath.string());
        return 1;
    }
    LOG_INFO("Wrote " + outputPath.string());
    return 0;
}

// XBF -> BREP
int convertXbfToBrep(const ConvertArgs& args) {
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

    // Named prim(s) export.
    if (!args.primPaths.empty()) {
        bool allOk = true;
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
                allOk = false;
                continue;
            }

            fs::path outPath = args.outputPath;
            if (outPath.empty()) {
                outPath = sanitizeUsdName(it->path.GetName()) + ".brep";
            }
            if (!writeShapeToBrep(it->localShape, it->worldTransform, args.bakeWorldTransform, outPath)) {
                allOk = false;
                continue;
            }
            LOG_INFO("Wrote " + outPath.string());
        }
        return allOk ? 0 : 1;
    }

    // All-prims export.
    fs::path outDir = args.outputPath;
    if (outDir.empty()) {
        outDir = fs::current_path();
        LOG_WARN("Output path not given. Dumping in " + outDir.string());
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
    return written == static_cast<int>(parts.size()) ? 0 : 1;
}

int runConvertMode(const std::vector<std::string>& tokens) {
    ConvertArgs args;
    for (size_t i = 0; i < tokens.size(); i++) {
        const std::string& token = tokens[i];
        const std::string& nextToken = i + 1 < tokens.size() ? tokens[i + 1] : "";
        bool consumeNext = false;
        if (!args.parse(token, nextToken, consumeNext)) {
            LOG_ERR("Unrecognized option for 'convert' mode: " + token);
            std::cerr << usageText << std::endl;
            return 1;
        }
        if (consumeNext) i++;
    }

    ConversionKind kind = detectConversionKind(args.inputPath);
    if (!verifyArgs(args, kind)) return 1;

    if (args.verbose) {
        Logger::activeLevel = Logger::Level::DEBUG;
    }

    try {
        switch (kind) {
            case ConversionKind::StepToXbf:
                return convertStepToXbf(args.inputPath, args.outputPath);
            case ConversionKind::BrepToXbf:
                return convertBrepToXbf(args.inputPath, args.outputPath);
            case ConversionKind::XbfToBrep:
                return convertXbfToBrep(args);
            case ConversionKind::Unknown:
            default:
                return 1; // unreachable: verifyArgs already rejects Unknown
        }
    } catch (const Standard_Failure& e) {
        LOG_ERR("OCC exception: " + std::string(e.GetMessageString()));
        return 1;
    } catch (const std::exception& e) {
        LOG_ERR("std exception: " + std::string(e.what()));
        return 1;
    }
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << usageText << std::endl;
        return 1;
    }
    std::string modeToken = argv[1];
    if (modeToken != "convert") {
        LOG_ERR("Unrecognized mode: " + modeToken);
        std::cerr << usageText << std::endl;
        return 1;
    }

    std::vector<std::string> tokens;
    for (int i = 2; i < argc; i++)
        tokens.emplace_back(argv[i]);

    auto start = std::chrono::high_resolution_clock::now();
    int result = runConvertMode(tokens);
    auto end = std::chrono::high_resolution_clock::now();
    LOG_INFO("Total Time Taken: " + std::to_string(std::chrono::duration<double>(end - start).count()) + " seconds");
    return result;
}