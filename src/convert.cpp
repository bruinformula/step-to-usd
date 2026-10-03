#include <cassert>
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <vector>
#include <optional>

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

#pragma push_macro("Handle")
#undef Handle

#include <pxr/usd/sdf/path.h>

#pragma pop_macro("Handle")

#include "CadUSD/OpenCascadeAssembly.h"
#include "CadUSD/Logger.h"
#include "CadUSD/UsdUtils.h"

#include "convert.h"

namespace occt = opencascade;
namespace fs = std::filesystem;

const std::string usageText =
    "vroom <convert|mesh> [options]\n"
    "\n"
    "convert: STEP/BREP <-> XBF\n"
    "  -i, --input <path>       Input\n"
    "  -o, --output <path>      Output\n"
    "  -p, --prim <path>        BREP export prim (repeatable)\n"
    "  -v, --verbose            Debug\n"
    "\n"
    "mesh: Tessellate USD\n"
    "  -i, --input <path>       Input\n"
    "  -p, --prim <path>        Prim (repeatable)\n"
    "  -q, --quiet              Suppress output\n"
    "  -v, --verbose            Verbose\n"
    "  -h, --help               Help\n"
    "\n"
    "Examples:\n"
    "  vroom convert -i part.step\n"
    "  vroom convert -i assembly.xbf -p /Bracket\n"
    "  vroom mesh -i container.usd\n";

std::string lowerExtension(const fs::path& p) {
    std::string ext = p.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(),
                    [](unsigned char c) { return std::tolower(c); });
    return ext;
}

ConvertArgs::ParseResult ConvertArgs::parse(const std::string& token, const std::string& nextToken) {
    if (token == "-i" || token == "--input") {
        if (nextToken.empty()) {
            std::cerr << "Expected a path after " << token << std::endl;
            return ParseResult::FAILURE;
        }
        if (!inputPath.empty()) {
            std::cerr << token << " is already set!" << std::endl;
            return ParseResult::FAILURE;
        }
        inputPath = nextToken;
        std::string ext = lowerExtension(inputPath);
    
        if (ext == ".step" || ext == ".stp") {
            kind = ConversionKind::StepToXbf;
        } else if (ext == ".brep") {
            kind = ConversionKind::BrepToXbf;
        } else if (ext == ".xbf") {
            kind = ConversionKind::XbfToBrep;
        } else {
            kind = ConversionKind::Unknown;
        }

        return ParseResult::SUCCESS_CONSUME_NEXT;
    }
    if (token == "-o" || token == "--output") {
        if (nextToken.empty()) {
            std::cerr << "Expected a path after " << token << std::endl;
            return ParseResult::FAILURE;
        }
        if (!outputPath.empty()) {
            std::cerr << token << " is already set!" << std::endl;
            return ParseResult::FAILURE;
        }
        outputPath = nextToken;
        return ParseResult::SUCCESS_CONSUME_NEXT;
    }
    if (!token.empty() && token[0] != '-') {
        if (!inputPath.empty()) {
            LOG_ERR("Input is already set! Unexpected extra argument: " + token);
            return ParseResult::FAILURE;
        }
        inputPath = token;
        return ParseResult::SUCCESS;
    }
    if (token == "-p" || token == "--prim") {
        if (nextToken.empty()) {
            LOG_ERR("Expected a prim path after " + token);
            return ParseResult::FAILURE;
        }
        primPaths.push_back(nextToken);
        return ParseResult::SUCCESS_CONSUME_NEXT;
    }

    if (token == "-h" || token == "--help") {
        std::cout << usageText << std::endl;
        return ParseResult::EXIT;
    }

    if (token == "-q" || token == "--quiet") {
        Logger::activeLevel = Logger::NONE;
        return ParseResult::SUCCESS;
    }

    if (token == "-v" || token == "--verbose") {
        Logger::activeLevel = Logger::DEBUG;
        return ParseResult::SUCCESS;
    } 

    std::cout << "Unrecognized command-line option: " << token << std::endl;
    return ParseResult::FAILURE;
}

bool ConvertArgs::verify() {
    if (inputPath.empty()) {
        LOG_ERR("Input path is not set!");
        return false;
    }
    if (!fs::exists(inputPath)) {
        LOG_ERR("The provided input file does not exist: " + inputPath.string());
        return false;
    }
    switch (kind) {
        case ConversionKind::StepToXbf:
        case ConversionKind::BrepToXbf: {
            if (!primPaths.empty()) {
                LOG_WARN("-p/--prim is ignored when converting to XBF");
            }
            if (outputPath.empty()) {
                outputPath = inputPath;
                outputPath.replace_extension("xbf");
            } else if (!outputPath.has_extension() || !outputPath.has_filename()) {
                LOG_ERR("xbf output path invalid: " + outputPath.string());
                return false;
            }
            return true;
        }
        case ConversionKind::XbfToBrep: {
            if (primPaths.empty() && outputPath.empty()) {
                LOG_ERR("Must specify an output directory for breps when exporting all prims");
                return false;
            }
            return true;
        }
        case ConversionKind::Unknown:
        default:
            LOG_ERR("Unrecognized input extension '" + inputPath.extension().string() +
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
    if (Logger::activeLevel == Logger::Level::DEBUG) {
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

