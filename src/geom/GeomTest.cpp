#include <iostream>
#include <optional>
#include <vector>
#include <iomanip>
#include <set>
#include <cmath>

#include <opencascade/TopoDS_Shape.hxx>
#include <opencascade/TopoDS.hxx>
#include <opencascade/TopLoc_Location.hxx>
#include <opencascade/gp_Trsf.hxx>
#include <opencascade/gp_XYZ.hxx>
#include <opencascade/BRepBuilderAPI_Transform.hxx>
#include <opencascade/STEPControl_Reader.hxx>
#include <opencascade/STEPControl_Writer.hxx>
#include <opencascade/XSControl_WorkSession.hxx>
#include <opencascade/XSControl_TransferReader.hxx>
#include <opencascade/STEPCAFControl_Reader.hxx>
#include <opencascade/TDocStd_Document.hxx>
#include <opencascade/XCAFDoc_DocumentTool.hxx>
#include <opencascade/XCAFDoc_ShapeTool.hxx>
#include <opencascade/TDF_Label.hxx>
#include <opencascade/TDF_ChildIterator.hxx>
#include <opencascade/XCAFDoc_Location.hxx>
#include <opencascade/TDataStd_Name.hxx>
#include <opencascade/TDF_LabelSequence.hxx>
#include <opencascade/STEPCAFControl_Writer.hxx>
#include <opencascade/BRep_Builder.hxx>
#include <opencascade/TopoDS_Compound.hxx>

#include "ArgumentHandler.h"
#include "STEPModel.h"

namespace occt = opencascade;

struct GeomTestArgumentHandler : public ArgumentHandler {

    std::filesystem::path inputSTEPFile;

    ParseResult parse(const std::string& token, const std::string& nextToken) override {

        switch (hashString(token)) {
            case hashString("--inputSTEPFile"): {
                if (nextToken.empty()) goto expectOption;
                if (!inputSTEPFile.empty()) goto alreadySet;
                inputSTEPFile = nextToken;
                return SUCCESS_CONSUME_NEXT;
            }
            case hashString("--help"): {
                std::cout << "Usage: GeomTest --inputSTEPFile <path_to_step_file>" << std::endl;
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
        return true;
    }
};

void printTransform(const gp_Trsf& trsf, const std::string& prefix = "") {
    std::cout << prefix << "Transform Matrix:\n";
    for (int i = 1; i <= 3; i++) {
        std::cout << prefix << "  [";
        for (int j = 1; j <= 4; j++) {
            std::cout << std::setw(10) << std::fixed << std::setprecision(4) 
                      << trsf.Value(i, j);
            if (j < 4) std::cout << ", ";
        }
        std::cout << "]\n";
    }
    std::cout << prefix << "  [" << std::setw(10) << 0.0 << ", " 
              << std::setw(10) << 0.0 << ", " 
              << std::setw(10) << 0.0 << ", " 
              << std::setw(10) << 1.0 << "]\n";
    
    // translation
    gp_XYZ translation = trsf.TranslationPart();
    std::cout << prefix << "Translation: (" 
              << translation.X() << ", " 
              << translation.Y() << ", " 
              << translation.Z() << ")\n";
    
    // scale
    std::cout << prefix << "Scale Factor: " << trsf.ScaleFactor() << "\n";
}

void printXCAFHierarchy(const TDF_Label& label, 
                        const gp_Trsf& parentTransform, 
                        occt::handle<XCAFDoc_ShapeTool> shapeTool, 
                        std::set<int>& printedDefinitions, 
                        int indent = 0
) {
    std::string indentStr(indent, ' ');
    
    occt::handle<TDataStd_Name> name;
    std::string labelName = "UNAMED!";
    if (label.FindAttribute(TDataStd_Name::GetID(), name)) {
        TCollection_ExtendedString extName = name->Get();
        int length = extName.Length();
        labelName = "";
        for (int i = 1; i <= length; i++) {
            char16_t c = extName.Value(i);
            if (c < 128) {
                labelName += static_cast<char>(c);
            }
        }
    }
    
    std::cout << indentStr << "Label: " << labelName << "\n";
    std::cout << indentStr << "Tag: " << label.Tag() << "\n";
    
    TopLoc_Location loc = shapeTool->GetLocation(label);
    gp_Trsf localTransform = loc.Transformation();
    
    // the accumulated transform from the root to this label
    gp_Trsf accumulatedTransform = parentTransform * localTransform;
    
    auto isIdentity = [](const gp_Trsf& t) -> bool {
        const double epsilon = 1e-10;
        // rotation/scale part
        for (int i = 1; i <= 3; i++) {
            for (int j = 1; j <= 3; j++) {
                double expected = (i == j) ? 1.0 : 0.0;
                if (std::abs(t.Value(i, j) - expected) > epsilon) {
                    return false;
                }
            }
        }
        // translation part
        if (std::abs(t.Value(1, 4)) > epsilon ||
            std::abs(t.Value(2, 4)) > epsilon ||
            std::abs(t.Value(3, 4)) > epsilon) {
            return false;
        }
        return true;
    };
    
    constexpr bool debug = false; 

    if (debug) {
        if (!isIdentity(localTransform)) {
            std::cout << indentStr << "Local Transform:\n";
            printTransform(localTransform, indentStr + "  ");
        } else {
            std::cout << indentStr << "Local Transform: Identity\n";
        }

        if (!isIdentity(accumulatedTransform)) {
            std::cout << indentStr << "Accumulated Transform (from root):\n";
            printTransform(accumulatedTransform, indentStr + "  ");
        } else {
            std::cout << indentStr << "Accumulated Transform: Identity\n";
        }
    }
    
    if (shapeTool->IsShape(label)) {
        TopoDS_Shape shape = shapeTool->GetShape(label);
        std::cout << indentStr << "Shape Type: ";

        switch (shape.ShapeType()) {
            case TopAbs_COMPOUND: std::cout << "COMPOUND"; break;
            case TopAbs_COMPSOLID: std::cout << "COMPSOLID"; break;
            case TopAbs_SOLID: std::cout << "SOLID"; break;
            case TopAbs_SHELL: std::cout << "SHELL"; break;
            case TopAbs_FACE: std::cout << "FACE"; break;
            case TopAbs_WIRE: std::cout << "WIRE"; break;
            case TopAbs_EDGE: std::cout << "EDGE"; break;
            case TopAbs_VERTEX: std::cout << "VERTEX"; break;
            default: std::cout << "UNKNOWN"; break;
        }

        std::cout << "\n";
        
        if (shapeTool->IsAssembly(label)) {
            std::cout << indentStr << "Type: ASSEMBLY\n";
        } else if (shapeTool->IsComponent(label)) {
            std::cout << indentStr << "Type: COMPONENT\n"; // componets are instances
            TDF_Label ref;
            if (shapeTool->GetReferredShape(label, ref)) {
                int refTag = ref.Tag();
                std::cout << indentStr << "References Label Tag: " << refTag << "\n";
                
                if (printedDefinitions.find(refTag) == printedDefinitions.end()) {
                    // First time seeing this definition - print it
                    std::cout << indentStr << "REFERENCED DEFINITION:\n";
                    printedDefinitions.insert(refTag);
                    printXCAFHierarchy(ref, accumulatedTransform, shapeTool, printedDefinitions, indent + 4);
                    std::cout << indentStr << "END REFERENCED DEFINITION\n";
                } else {
                    // Just note the location of of the OG definition
                    std::cout << indentStr << "RE-REFERENCED DEFINITION - Tag " << refTag << "\n";
                }
            }
        } else if (shapeTool->IsSimpleShape(label)) {
            std::cout << indentStr << "Type: SIMPLE SHAPE\n";
        } else if (shapeTool->IsFree(label)) {
            std::cout << indentStr << "Type: FREE SHAPE\n";
        }
    }
        
    // recurse into children
    if (shapeTool->IsAssembly(label)) {
        // For assemblies, use GetComponents to get child components
        NCollection_Sequence<TDF_Label> components;
        shapeTool->GetComponents(label, components);
        
        for (int i = 1; i <= components.Length(); i++) {
            printXCAFHierarchy(components.Value(i), accumulatedTransform, shapeTool, printedDefinitions, indent + 2);
        }
    }
}

struct EntityCounts {
    int numAssemblies = 0;
    int numComponents = 0;
    int numSimpleShapes = 0;
    int numWithTransforms = 0;
};

void countEntities(
    const TDF_Label& label, 
    bool isInstance, 
    EntityCounts& counts, 
    occt::handle<XCAFDoc_ShapeTool>& shapeTool, 
    std::set<int>& countedDefinitions
) {
    if (!shapeTool->IsShape(label)) {
        return;
    }
    
    int tag = label.Tag();
    
    if (shapeTool->IsAssembly(label)) {
        // Count assembly definition only once
        if (countedDefinitions.find(tag) == countedDefinitions.end()) {
            counts.numAssemblies++;
            countedDefinitions.insert(tag);
        }
        
        // Get and count all component instances of this assembly
        NCollection_Sequence<TDF_Label> components;
        shapeTool->GetComponents(label, components);
        
        for (int i = 1; i <= components.Length(); i++) {
            countEntities(components.Value(i), true, counts, shapeTool, countedDefinitions);  // These are instances
        }
        
    } else if (shapeTool->IsComponent(label)) {
        // Always count the component instance
        counts.numComponents++;
        
        // Check for transform on this instance
        TopLoc_Location loc = shapeTool->GetLocation(label);
        gp_Trsf t = loc.Transformation();
        bool hasTransform = false;
        const double epsilon = 1e-10;
        
        for (int i = 1; i <= 3; i++) {
            for (int j = 1; j <= 3; j++) {
                double expected = (i == j) ? 1.0 : 0.0;
                if (std::abs(t.Value(i, j) - expected) > epsilon) {
                    hasTransform = true;
                    break;
                }
            }
            if (hasTransform) break;
        }
        
        if (!hasTransform) {
            if (std::abs(t.Value(1, 4)) > epsilon ||
                std::abs(t.Value(2, 4)) > epsilon ||
                std::abs(t.Value(3, 4)) > epsilon) {
                hasTransform = true;
            }
        }
        
        if (hasTransform) counts.numWithTransforms++;
        
        TDF_Label ref;
        if (shapeTool->GetReferredShape(label, ref)) {
            countEntities(ref, false, counts, shapeTool, countedDefinitions);  // The definition itself is not an instance
        }
        
    } else if (shapeTool->IsSimpleShape(label)) {
        // Count simple shape definition only once
        if (countedDefinitions.find(tag) == countedDefinitions.end()) {
            counts.numSimpleShapes++;
            countedDefinitions.insert(tag);
        }
    }
};

int main(int argc, char** argv) {
    std::vector<std::string> tokens;
    for (int i = 1; i < argc; i++) {
        tokens.emplace_back(argv[i]);
    }
    
    GeomTestArgumentHandler inputArgs;
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
        
    std::optional<STEPModel> optionalModel = STEPModel::loadFromFile(inputArgs.inputSTEPFile);
    
    if(!optionalModel.has_value()) {
        return 1;
    }

    STEPModel model = optionalModel.value();

    model.buildInstanceTree();

    model.printInstanceTree();
    model.printDefinitionShapes();

    model.writeMeshTest();

    return 0;
}