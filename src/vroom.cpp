#include <cassert>
#include <cctype>
#include <chrono>
#include <exception>
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

#include "convert.h"
#include "mesh.h"

#include "cadContainerAPI.h"

#include "CadUSD/CadUsdPipeline.h"
#include "CadUSD/Logger.h"

const std::string usageText =
    " vroom -- CAD data io and meshing utility\n"
    " usage: vroom convert -i <input> [-o <output>] [-p <prim>] [-v]\n"
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
    "    vroom convert -i part.step\n"
    "    vroom convert -i part.brep -o part.xbf\n"
    "    vroom convert -i assem.xbf\n"
    "    vroom convert -i assem.xbf -p /Bracket__a1b2c3d4 -o bracket.brep\n\n";

int runMeshMode(const std::vector<std::string>& tokens) {
    CadUsdTesselateArgs args;
    for (size_t i = 0; i < tokens.size(); i++) {
        const std::string& token = tokens[i];
        const std::string& nextToken = i + 1 < tokens.size() ? tokens[i + 1] : "";
        
        CadUsdTesselateArgs::ParseResult parseResult = args.parse(token, nextToken);
        switch (parseResult) {
            case CadUsdTesselateArgs::SUCCESS:
                break;
            case CadUsdTesselateArgs::SUCCESS_CONSUME_NEXT:
                i++;
                break;
            case CadUsdTesselateArgs::FAILURE:
                return 1;
            case CadUsdTesselateArgs::EXIT:
                return 0;
        }
    }
    
    if (!args.verify()) {
        std::cerr << "Input argument verification failed." << std::endl;
        return 1;
    }

    auto start = std::chrono::high_resolution_clock::now();

    std::optional<CadUsdPipeline> optionalCadPipeline = CadUsdPipeline::create(args.inputUsdFile);

    if (!optionalCadPipeline.has_value()) {
        std::cerr << "Failed to initialize CadUsdPipeline." << std::endl;
        return 1;
    }

    CadUsdPipeline cadPipeline = std::move(*optionalCadPipeline);

    // Search for step container prims and run populateUsd on each `containerPrim`
    for (UsdPrim prim : cadPipeline.containerStage->TraverseAll()) {
        if (!prim.HasAPI<AutolibCadContainerAPI>()) continue;

        cadPipeline.populateUsd(prim, args.selectedPaths);
    }

    if (Logger::activeLevel == Logger::Level::INFO) {
        auto end = std::chrono::high_resolution_clock::now();
        LOG_INFO("Total Time Taken: " + std::to_string(std::chrono::duration<double>(end - start).count()) + " seconds");
    }
    return 0;
}

int runConvertMode(const std::vector<std::string>& tokens) {
    ConvertArgs args;
    for (size_t i = 0; i < tokens.size(); i++) {
        const std::string& token = tokens[i];
        const std::string& nextToken = i + 1 < tokens.size() ? tokens[i + 1] : "";
        
        ConvertArgs::ParseResult parseResult = args.parse(token, nextToken);
        switch (parseResult) {
            case ConvertArgs::SUCCESS:
                break;
            case ConvertArgs::SUCCESS_CONSUME_NEXT:
                i++;
                break;
            case ConvertArgs::FAILURE:
                return 1;
            case ConvertArgs::EXIT:
                return 0;
        }
    }

    if (!args.verify()) return 1;

    auto start = std::chrono::high_resolution_clock::now();

    try {
        switch (args.kind) {
            case ConversionKind::StepToXbf:
                return convertStepToXbf(args.inputPath, args.outputPath);
            case ConversionKind::BrepToXbf:
                return convertBrepToXbf(args.inputPath, args.outputPath);
            case ConversionKind::XbfToBrep:
                return convertXbfToBrep(args);
            case ConversionKind::Unknown:
            default:
                return 1; 
        }
    } catch (const Standard_Failure& e) {
        LOG_ERR("OCC exception: " + std::string(e.GetMessageString()));
        return 1;
    } catch (const std::exception& e) {
        LOG_ERR("std exception: " + std::string(e.what()));
        return 1;
    }

    if (Logger::activeLevel == Logger::Level::INFO) {
        auto end = std::chrono::high_resolution_clock::now();
        LOG_INFO("Total Time Taken: " + std::to_string(std::chrono::duration<double>(end - start).count()) + " seconds");
    }

}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << usageText << std::endl;
        return 1;
    }
    std::string modeToken = argv[1];

    if (modeToken == "-h" || modeToken == "--help") {
        std::cout << usageText << std::endl;
        return 0;
    } else if (modeToken == "convert") {
        return runConvertMode(std::vector<std::string>(argv + 2, argv + argc));
    } else if (modeToken == "mesh") {
        return runMeshMode(std::vector<std::string>(argv + 2, argv + argc));
    } else {
        LOG_ERR("Unrecognized mode: " + modeToken);
        std::cerr << usageText << std::endl;
        return 1;
    }

}