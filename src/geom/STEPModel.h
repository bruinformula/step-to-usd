#pragma once

#include <filesystem>
#include <iostream>
#include <optional>

#include <opencascade/BinXCAFDrivers.hxx>
#include <opencascade/STEPCAFControl_Reader.hxx>
#include <opencascade/TDocStd_Application.hxx>
#include <opencascade/TDocStd_Document.hxx>
#include <opencascade/XCAFDoc_DocumentTool.hxx>
#include <opencascade/XCAFDoc_ShapeTool.hxx>
#include <opencascade/TDF_Label.hxx>
#include <OSD_Parallel.hxx>

namespace occt = opencascade;
namespace fs = std::filesystem;

struct STEPModel {
    occt::handle<TDocStd_Application> app;
    occt::handle<TDocStd_Document> doc;
    occt::handle<XCAFDoc_ShapeTool> shapeTool;

    STEPModel(occt::handle<TDocStd_Application> a, occt::handle<TDocStd_Document> d, occt::handle<XCAFDoc_ShapeTool> st) : 
        app(a), 
        doc(d), 
        shapeTool(st) 
    {}

    static std::optional<STEPModel> loadFromFile(const fs::path& stepPath) {
        try {

            OSD_Parallel::SetUseOcctThreads(true);

            occt::handle<TDocStd_Application> app = new TDocStd_Application();
            BinXCAFDrivers::DefineFormat(app);

            fs::path xdfPath = stepPath;
            xdfPath.replace_extension("xbf");

            occt::handle<TDocStd_Document> doc;

            // xbf files are a binary representation of the document 
            // makes reloading much faster after the first time, but 
            // they are not human-readable and can get out of sync 
            // with the step file if the step file changes

            if (!fs::exists(xdfPath) || fs::last_write_time(xdfPath) < fs::last_write_time(stepPath)) {
                std::cout << "XBF doesn't exist or is out of date. Building from STEP..." << std::endl;
                app->NewDocument("BinXCAF", doc);

                STEPCAFControl_Reader reader;
                IFSelect_ReturnStatus stat = reader.ReadFile(stepPath.c_str());
                if (stat != IFSelect_RetDone) {
                    std::cerr << "Error reading STEP file\n";
                    return std::nullopt;
                }
                if (!reader.Transfer(doc)) {
                    std::cerr << "Error transferring STEP data\n";
                    return std::nullopt;
                }

                std::cout << "Saving XBF to " << xdfPath << std::endl;
                PCDM_StoreStatus status = app->SaveAs(doc, xdfPath.c_str());
                if (status != PCDM_SS_OK) {
                    std::cerr << "Failed to save XBF document" << std::endl;
                    // non-fatal, we still have the doc in memory
                }
            } else {
                std::cout << "Loading cached XBF from " << xdfPath << std::endl;
                PCDM_ReaderStatus status = app->Open(xdfPath.c_str(), doc);
                if (status != PCDM_RS_OK) {
                    std::cerr << "Error opening XBF file, status: " << status << std::endl;
                    return std::nullopt;
                }
            }

            occt::handle<XCAFDoc_ShapeTool> shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
            return STEPModel(app, doc, shapeTool);

        } catch (const Standard_Failure& e) {
            std::cerr << "OCC exception: " << e.GetMessageString() << std::endl;
            return std::nullopt;
        } catch (const std::exception& e) {
            std::cerr << "std exception: " << e.what() << std::endl;
            return std::nullopt;
        }
    }
};