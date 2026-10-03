#pragma once

#include <cassert>
#include <cctype>
#include <filesystem>
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

namespace fs = std::filesystem;

enum class ConversionKind {
    StepToXbf,
    BrepToXbf,
    XbfToBrep,
    Unknown
};

struct ConvertArgs {

    ConversionKind kind;
    fs::path inputPath;
    fs::path outputPath;

    std::vector<std::string> primPaths;
    bool verbose = false;
    bool bakeWorldTransform = true;

    bool parse(const std::string& token, const std::string& nextToken, bool& consumeNext); 
    bool verify();
};

// STEP -> XBF
int convertStepToXbf(const fs::path& inputPath, const fs::path& outputPath); 
// BREP -> XBF
int convertBrepToXbf(const fs::path& inputPath, const fs::path& outputPath);
// XBF -> BREP
int convertXbfToBrep(const ConvertArgs& args); 
