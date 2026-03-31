#include <cassert>
#include <filesystem>
#include <unordered_map>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>

#include <opencascade/BinXCAFDrivers.hxx>
#include <opencascade/STEPCAFControl_Reader.hxx>
#include <opencascade/TDocStd_Application.hxx>
#include <opencascade/TDocStd_Document.hxx>
#include <opencascade/XCAFDoc_DocumentTool.hxx>
#include <opencascade/XCAFDoc_ShapeTool.hxx>
#include <opencascade/TDF_Label.hxx>
#include <opencascade/TopLoc_Location.hxx>
#include <opencascade/gp_Trsf.hxx>
#include <opencascade/XCAFDoc_ColorTool.hxx>
#include <opencascade/XCAFDoc_MaterialTool.hxx>
#include <opencascade/XCAFDoc_LayerTool.hxx>
#include <opencascade/OSD_Parallel.hxx>
#include <opencascade/IFSelect_ReturnStatus.hxx>
#include <opencascade/NCollection_Sequence.hxx>
#include <opencascade/PCDM_ReaderStatus.hxx>
#include <opencascade/PCDM_StoreStatus.hxx>
#include <opencascade/Quantity_TypeOfColor.hxx>
#include <opencascade/STEPControl_Reader.hxx>
#include <opencascade/Standard_Failure.hxx>
#include <opencascade/TColStd_SequenceOfAsciiString.hxx>
#include <opencascade/TCollection_AsciiString.hxx>
#include <opencascade/TCollection_ExtendedString.hxx>
#include <opencascade/TCollection_HAsciiString.hxx>
#include <opencascade/TDF_LabelSequence.hxx>
#include <opencascade/TopoDS_Shape.hxx>
#include <opencascade/XCAFDoc_ColorType.hxx>
#include <opencascade/gp_XYZ.hxx>
#include <opencascade/Quantity_Color.hxx>
#include <opencascade/Standard_Handle.hxx>
#include <opencascade/TDataStd_Name.hxx>

#include "StepModel.h"

// Init
static fs::path unitsCachePath(const fs::path& xbfPath) {
    fs::path p = xbfPath;
    return p.replace_extension("units");
}

static void saveUnitsCache(const fs::path& xbfPath, double metersPerUnit) {
    std::ofstream f(unitsCachePath(xbfPath));
    if (f) f << std::setprecision(17) << metersPerUnit;
}

static double loadUnitsCache(const fs::path& xbfPath) {
    std::ifstream f(unitsCachePath(xbfPath));
    double v = 0.001;
    if (f) f >> v;
    return v;
}

static double readStepLengthUnit(STEPControl_Reader& cafReader) {
    TColStd_SequenceOfAsciiString lengthUnits, angleUnits, solidAngleUnits;
    cafReader.FileUnits(lengthUnits, angleUnits, solidAngleUnits);

    if (lengthUnits.IsEmpty()) {
        std::cerr << "Warning: FileUnits() returned nothing, assuming mm\n";
        return 0.001;
    }

    TCollection_AsciiString unit = lengthUnits.First();
    unit.LowerCase();
    std::cout << "STEP file length unit: " << unit << "\n";

    if (unit == "metre"      || unit == "meter"      || unit == "m")   return 1.0;
    if (unit == "millimetre" || unit == "millimeter"  || unit == "mm")  return 0.001;
    if (unit == "centimetre" || unit == "centimeter"  || unit == "cm")  return 0.01;
    if (unit == "micrometre" || unit == "micrometer"  || unit == "um")  return 1e-6;
    if (unit == "kilometre"  || unit == "kilometer"   || unit == "km")  return 1000.0;
    if (unit == "inch"       || unit == "in")                           return 0.0254;
    if (unit == "foot"       || unit == "ft")                           return 0.3048;
    if (unit == "yard"       || unit == "yd")                           return 0.9144;
    if (unit == "mil"        || unit == "thou")                         return 2.54e-5;

    std::cerr << "Warning: unrecognised unit '" << unit << "', assuming mm\n";
    return 0.001;
}

std::string getLabelName(const TDF_Label& label) {
    occt::handle<TDataStd_Name> nameAttr;
    if (!label.FindAttribute(TDataStd_Name::GetID(), nameAttr)) 
        return "";
    
    const TCollection_ExtendedString& ext = nameAttr->Get();
    std::string result;
    for (int i = 1; i <= ext.Length(); i++) {
        char16_t c = ext.Value(i);
        if (c < 128) result += static_cast<char>(c);
    }
    return result;
}

std::optional<StepModel> StepModel::loadFromFile(const fs::path& stepPath) {
    try {
        OSD_Parallel::SetUseOcctThreads(true);
        occt::handle<TDocStd_Application> app = new TDocStd_Application();
        BinXCAFDrivers::DefineFormat(app);
        fs::path xbfPath = stepPath;
        xbfPath.replace_extension("xbf");
        occt::handle<TDocStd_Document> doc;
        app->NewDocument("BinXCAF", doc);

        double metersPerUnit = 0.0;
        if (!fs::exists(xbfPath) || fs::last_write_time(xbfPath) < fs::last_write_time(stepPath)) {
            std::cout << "XBF doesn't exist or is out of date. Building from Step...\n";
            STEPCAFControl_Reader reader;
            if (reader.ReadFile(stepPath.c_str()) != IFSelect_RetDone) {
                std::cerr << "Error reading Step file\n";
                return std::nullopt;
            }
            STEPControl_Reader innerReader = reader.Reader();
            metersPerUnit = readStepLengthUnit(innerReader);
            std::cout << "Step length unit: " << metersPerUnit << " m\n";
            if (!reader.Transfer(doc)) {
                std::cerr << "Error transferring Step data\n";
                return std::nullopt;
            }
            std::cout << "Saving XBF to " << xbfPath << "\n";
            doc->ChangeStorageFormat("BinXCAF");
            if (app->SaveAs(doc, xbfPath.c_str()) != PCDM_SS_OK)
                std::cerr << "Warning: failed to save XBF cache\n";
            saveUnitsCache(xbfPath, metersPerUnit);
        } else {
            std::cout << "Loading cached XBF from " << xbfPath << "\n";
            if (app->Open(xbfPath.c_str(), doc) != PCDM_RS_OK) {
                std::cerr << "Error opening XBF\n";
                return std::nullopt;
            }
            metersPerUnit = loadUnitsCache(xbfPath);
            std::cout << "Step length unit (cached): " << metersPerUnit << " m\n";
        }

        auto shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
        auto colorTool = XCAFDoc_DocumentTool::ColorTool(doc->Main());
        auto materialTool = XCAFDoc_DocumentTool::MaterialTool(doc->Main());
        auto layerTool = XCAFDoc_DocumentTool::LayerTool(doc->Main());

        StepModel model(stepPath, app, doc, shapeTool, colorTool, materialTool, layerTool, metersPerUnit);
        model.buildInstanceTree();
        return model;
    } catch (const Standard_Failure& e) {
        std::cerr << "OCC exception: " << e.GetMessageString() << "\n";
        return std::nullopt;
    } catch (const std::exception& e) {
        std::cerr << "std exception: " << e.what() << "\n";
        return std::nullopt;
    }
}

void StepModel::buildInstanceTree() {
    partNodes.clear();
    definitionShapes.clear();

    NCollection_Sequence<TDF_Label> freeShapes;
    shapeTool->GetFreeShapes(freeShapes);

    // Pass 1: count the total node count so we can resize partNodes[] once
    // and use direct index writes in pass 2
    int numNodes = 0;
    for (int i = 1; i <= freeShapes.Length(); i++) {
        numNodes += countNodes(freeShapes.Value(i));
    }

    partNodes.resize(numNodes);

    // Pass 2: pre-order fill — each assembly claims its slot then immediately
    // records firstChildIdx = cursor before recursing, so children are written
    // contiguously after it.
    int cursor = 0;
    gp_Trsf identity;
    for (int i = 1; i <= freeShapes.Length(); i++) {
        fillNode(freeShapes.Value(i), identity, -1, 0, cursor);
    }

    assert(cursor == numNodes); // should hit all the nodes we counted in pass 1

    int leaves = 0, assemblies = 0;
    for (const auto& n : partNodes) {
        if (n.type == PartNodeType::Leaf)     leaves++;
        if (n.type == PartNodeType::Assembly) assemblies++;
    }
    std::cout << "Instance tree built: " << partNodes.size() << " nodes\n";
    std::cout << "  Assemblies:          " << assemblies << "\n";
    std::cout << "  Leaves:              " << leaves     << "\n";
    std::cout << "  Unique definitions:  " << definitionShapes.size() << "\n";
}

bool StepModel::isLabelVisible(const TDF_Label& label) const {
    if (!colorTool->IsVisible(label))
        return false;

    // Some CAD tools might export visibility as 
    // a layer property rather than on the shape itself
    TDF_LabelSequence layers;
    layerTool->GetLayers(label, layers);
    for (int li = 1; li <= layers.Length(); ++li) {
        if (!layerTool->IsVisible(layers.Value(li)))
            return false;
    }

    return true;
}

// Counting
int StepModel::countNodes(const TDF_Label& label) {
    if (!shapeTool->IsShape(label)) return 0;

    if (shapeTool->IsComponent(label)) {
        TDF_Label defLabel;
        if (!shapeTool->GetReferredShape(label, defLabel)) return 0;
        if (shapeTool->IsSimpleShape(defLabel))  return 1;
        if (shapeTool->IsAssembly(defLabel))     return 1 + countAssemblyChildren(defLabel);
        return 1;
    }

    if (shapeTool->IsAssembly(label))    return 1 + countAssemblyChildren(label);
    if (shapeTool->IsSimpleShape(label)) return 1;
    return 0;
}

int StepModel::countAssemblyChildren(const TDF_Label& assemblyDef) {
    NCollection_Sequence<TDF_Label> components;
    shapeTool->GetComponents(assemblyDef, components);
    int total = 0;
    for (int i = 1; i <= components.Length(); i++)
        total += countNodes(components.Value(i));
    return total;
}

// Filling
void StepModel::fillNode(
    const TDF_Label& label,
    const gp_Trsf& parentWorld,
    int parentIdx,
    int depth,
    int& cursor
) {
    if (!shapeTool->IsShape(label)) return;

    // Extract just this label's own location
    // world is computed here purely so children can extract their own localTrsf
    // correctly relative to their parent; it is never stored.
    TopLoc_Location loc = shapeTool->GetLocation(label);
    gp_Trsf localTrsf = loc.Transformation();
    gp_Trsf world = parentWorld * localTrsf;

    if (shapeTool->IsComponent(label)) {
        TDF_Label defLabel;
        if (!shapeTool->GetReferredShape(label, defLabel)) return;

        if (shapeTool->IsSimpleShape(defLabel)) {
            fillLeaf(defLabel, localTrsf, parentIdx, depth, cursor);
        } else if (shapeTool->IsAssembly(defLabel)) {
            fillAssembly(defLabel, localTrsf, world, parentIdx, depth, cursor);
        } else {
            fillLeaf(defLabel, localTrsf, parentIdx, depth, cursor);
        }

    } else if (shapeTool->IsAssembly(label)) {
        fillAssembly(label, localTrsf, world, parentIdx, depth, cursor);

    } else if (shapeTool->IsSimpleShape(label)) {
        fillLeaf(label, localTrsf, parentIdx, depth, cursor);
    }
}

void StepModel::fillLeaf(
    const TDF_Label& defLabel,
    const gp_Trsf& localTrsf,
    int parentIdx,
    int depth,
    int& cursor
) {
    int myIdx = cursor++;

    partNodes[myIdx].name = getLabelName(defLabel);

    Quantity_Color color(0.8, 0.8, 0.8, Quantity_TOC_RGB);
    
    occt::handle<TCollection_HAsciiString> aName;
    occt::handle<TCollection_HAsciiString> aDescription;
    double                                 aDensity;
    occt::handle<TCollection_HAsciiString> aDensName;
    occt::handle<TCollection_HAsciiString> aDensValType;

    if (materialTool->GetMaterial(defLabel, aName, aDescription, aDensity, aDensName, aDensValType)) {
        aName->Print(std::cout);
        aDescription->Print(std::cout);
        aDensName->Print(std::cout);
        aDensValType->Print(std::cout);
    }

    //std::cout << partNodes[myIdx].materialName << std::endl;

    bool hasColor = colorTool->GetColor(defLabel, XCAFDoc_ColorSurf, color);
    if (hasColor) {
        partNodes[myIdx].color = color;
    } else {
        partNodes[myIdx].color = std::nullopt;
    }

    partNodes[myIdx].type             = PartNodeType::Leaf;
    partNodes[myIdx].definitionLabel  = defLabel;
    partNodes[myIdx].localTransform   = localTrsf;
    partNodes[myIdx].parentIdx        = parentIdx;
    partNodes[myIdx].firstChildIdx    = -1;
    partNodes[myIdx].childCount       = 0;
    partNodes[myIdx].depth            = depth;
    partNodes[myIdx].visible          = isLabelVisible(defLabel);

    // Only store the shape the first time we see this definition label.
    // Subsequent instances of the same part share this entry.
    if (definitionShapes.find(defLabel) == definitionShapes.end()) {
        definitionShapes[defLabel] = shapeTool->GetShape(defLabel);
    }
}

void StepModel::fillAssembly(
    const TDF_Label& defLabel,
    const gp_Trsf& localTrsf,
    const gp_Trsf& world,
    int parentIdx,
    int depth,
    int& cursor
) {
    int myIdx = cursor++;

    partNodes[myIdx].name = getLabelName(defLabel);
    partNodes[myIdx].color = std::nullopt;
    partNodes[myIdx].visible = isLabelVisible(defLabel);

    NCollection_Sequence<TDF_Label> components;
    shapeTool->GetComponents(defLabel, components);

    int validChildren = 0;
    for (int i = 1; i <= components.Length(); i++) {
        if (countNodes(components.Value(i)) > 0) 
            validChildren++;
    }
    // Record firstChildIdx before recursing — cursor is at the first child slot
    // right now, and recursion will fill slots contiguously from here
    int firstChild = (validChildren > 0) ? cursor : -1;

    partNodes[myIdx].type             = PartNodeType::Assembly;
    partNodes[myIdx].definitionLabel  = defLabel;
    partNodes[myIdx].localTransform   = localTrsf;
    partNodes[myIdx].parentIdx        = parentIdx;
    partNodes[myIdx].firstChildIdx    = firstChild;
    partNodes[myIdx].childCount       = validChildren;
    partNodes[myIdx].depth            = depth;

    for (int i = 1; i <= components.Length(); i++)
        fillNode(components.Value(i), world, myIdx, depth + 1, cursor);
}

// Debug 
void StepModel::debugPrintInstances() const {
    for (size_t i = 0; i < partNodes.size(); i++) {
        const PartNode& inst = partNodes[i];
        std::string type = (inst.type == PartNodeType::Assembly) ? "ASM" : "LEAF";
        gp_XYZ lt = inst.localTransform.TranslationPart();
        std::cout << "[" << i << "] " << type
                  << " parent="     << inst.parentIdx
                  << " firstChild=" << inst.firstChildIdx
                  << " childCount=" << inst.childCount
                  << " depth="      << inst.depth
                  << " localT=("    << lt.X() << "," << lt.Y() << "," << lt.Z() << ")"
                  << "\n";
    }
}