#include <cassert>
#include <filesystem>
#include <exception>
#include <iostream>

#include <BinXCAFDrivers.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <TDocStd_Application.hxx>
#include <TDocStd_Document.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <TDF_Label.hxx>
#include <TopLoc_Location.hxx>
#include <gp_Trsf.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_MaterialTool.hxx>
#include <XCAFDoc_LayerTool.hxx>
#include <OSD_Parallel.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <NCollection_Sequence.hxx>
#include <PCDM_ReaderStatus.hxx>
#include <PCDM_StoreStatus.hxx>
#include <Quantity_TypeOfColor.hxx>
#include <Standard_Failure.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TCollection_HAsciiString.hxx>
#include <TDF_LabelSequence.hxx>
#include <TopoDS_Shape.hxx>
#include <XCAFDoc_ColorType.hxx>
#include <gp_XYZ.hxx>
#include <Quantity_Color.hxx>
#include <Standard_Handle.hxx>
#include <TDataStd_Name.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include "CadUSD/Logger.h"

namespace occt = opencascade;

const std::string argOptions =
    " stepxbf -- Meshes all CadContainer prims in a Usd scene\n"
    " Options: \n"
    "    -i, --input   <path>            Path to the input step file. \n"
    "    -o, --output  <path>            Path to the output xbf file.\n"
    "    usage: stepxbf -i <path> \n";

struct CadXbfArgs {

    enum ParseResult {
        SUCCESS,
        SUCCESS_CONSUME_NEXT,
        FAILURE,
        EXIT
    };

    std::filesystem::path stepPath;
    std::filesystem::path xbfPath;

    ParseResult parse(const std::string& token, const std::string& nextToken) {
        if (token == "-i" || token == "--input") {
            if (nextToken.empty()) {
                std::cerr << "Expected another token following command-line option: " << token << std::endl;
                return FAILURE;
            }
            if (!stepPath.empty()) {
                std::cerr << token << " is already set!" << std::endl;
                return FAILURE;
            }
            stepPath = nextToken;
            return SUCCESS_CONSUME_NEXT;
        }

       if (token == "-o" || token == "--output") {
            if (nextToken.empty()) {
                std::cerr << "Expected another token following command-line option: " << token << std::endl;
                return FAILURE;
            }
            if (!xbfPath.empty()) {
                std::cerr << token << " is already set!" << std::endl;
                return FAILURE;
            }
            xbfPath = nextToken;
            return SUCCESS_CONSUME_NEXT;
        }

        std::cout << "Unrecognized command-line option: " << token << std::endl;
        std::cout << argOptions << std::endl;
        return FAILURE;
    }

    bool verify() {
        if (stepPath.empty()) {
            std::cerr << "stepPath is not set!" << std::endl;
            return false;
        }
        if (!std::filesystem::exists(stepPath)) {
            std::cerr << "The provided input step file does not exist: " << stepPath << std::endl;
            return false;
        }

        

        // Use Cad name if output isn't given 
        if (xbfPath.empty()) {
            xbfPath = stepPath;
            xbfPath.replace_extension("xbf");
        } else if (!xbfPath.has_extension() || !xbfPath.has_filename()) {
            std::cerr << "xbf path invalid: " << xbfPath << std::endl;
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
    
    CadXbfArgs args;
    for (size_t i = 0; i < tokens.size(); i++) {
        const std::string& token = tokens[i];
        const std::string& nextToken = i + 1 < tokens.size() ? tokens[i + 1] : "";
        
        CadXbfArgs::ParseResult parseResult = args.parse(token, nextToken);
        switch (parseResult) {
            case CadXbfArgs::SUCCESS:
                break;
            case CadXbfArgs::SUCCESS_CONSUME_NEXT:
                i++;
                break;
            case CadXbfArgs::FAILURE:
                return 1;
            case CadXbfArgs::EXIT:
                return 0;
        }
    }
    
    if (!args.verify()) {
        std::cerr << "Input argument verification failed." << std::endl;
        return 1;
    }

    auto start = std::chrono::high_resolution_clock::now();

    try {        
        OSD_Parallel::SetUseOcctThreads(true);
        occt::handle<TDocStd_Application> app = new TDocStd_Application();
        BinXCAFDrivers::DefineFormat(app);
        occt::handle<TDocStd_Document> doc;
        app->NewDocument("BinXCAF", doc);

        STEPCAFControl_Reader reader;
        if (reader.ReadFile(args.stepPath.c_str()) != IFSelect_RetDone) {
            LOG_ERR("Error reading Cad file");
            return 1;
        }
        if (!reader.Transfer(doc)) {
            LOG_ERR("Error transferring Cad data");
            return 1;
        }
        doc->ChangeStorageFormat("BinXCAF");
        if (app->SaveAs(doc, args.xbfPath.c_str()) != PCDM_SS_OK)
            LOG_ERR("Warning: failed to save XBF cache");
    } catch (const Standard_Failure& e) {
        LOG_ERR("OCC exception: " + std::string(e.GetMessageString()));
        return 1;
    } catch (const std::exception& e) {
        LOG_ERR("std exception: " + std::string(e.what()));
        return 1;
    }

    auto end = std::chrono::high_resolution_clock::now();

    LOG_INFO("Total Time Taken: " + std::to_string(std::chrono::duration<double>(end - start).count()) + " seconds");

    return 0;
}