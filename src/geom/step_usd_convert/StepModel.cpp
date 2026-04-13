#include <cassert>
#include <filesystem>
#include <unordered_map>
#include <exception>
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
#include <opencascade/Standard_Failure.hxx>
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
#include "Logger.h"

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

    // OCCT sometimes stores the entry path as the name attribute — treat as unnamed
    if (!result.empty() && result[0] == '=')
        return "";

    return result;
}

std::optional<StepModel> StepModel::loadFromFile(const fs::path& stepPath) {
    try {
        constexpr double kOcctWorkingMetersPerUnit = 0.001;
        OSD_Parallel::SetUseOcctThreads(true);
        occt::handle<TDocStd_Application> app = new TDocStd_Application();
        BinXCAFDrivers::DefineFormat(app);
        fs::path xbfPath = stepPath;
        xbfPath.replace_extension("xbf");
        occt::handle<TDocStd_Document> doc;
        app->NewDocument("BinXCAF", doc);

        double metersPerUnit = kOcctWorkingMetersPerUnit;
        if (!fs::exists(xbfPath) || fs::last_write_time(xbfPath) < fs::last_write_time(stepPath)) {
            LOG_INFO("XBF doesn't exist or is out of date. Building from Step...");
            LOG_SCOPED_TIMER("Build XBF Document from STEP file");
            STEPCAFControl_Reader reader;
            if (reader.ReadFile(stepPath.c_str()) != IFSelect_RetDone) {
                LOG_ERR("Error reading Step file");
                return std::nullopt;
            }
            if (!reader.Transfer(doc)) {
                LOG_ERR("Error transferring Step data");
                return std::nullopt;
            }
            LOG_INFO("Saving XBF to " + xbfPath.string());
            doc->ChangeStorageFormat("BinXCAF");
            if (app->SaveAs(doc, xbfPath.c_str()) != PCDM_SS_OK)
                LOG_ERR("Warning: failed to save XBF cache");
        } else {
            LOG_INFO("Loading cached XBF from " + xbfPath.string());
            if (app->Open(xbfPath.c_str(), doc) != PCDM_RS_OK) {
                LOG_ERR("Error opening XBF");
                return std::nullopt;
            }
        }
        LOG_INFO("Working OCC length unit: " + std::to_string(metersPerUnit) + " m");

        auto shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
        auto colorTool = XCAFDoc_DocumentTool::ColorTool(doc->Main());
        auto materialTool = XCAFDoc_DocumentTool::MaterialTool(doc->Main());
        auto layerTool = XCAFDoc_DocumentTool::LayerTool(doc->Main());

        StepModel model(stepPath, app, doc, shapeTool, colorTool, materialTool, layerTool, metersPerUnit);
        model.buildInstanceTree();
        return model;
    } catch (const Standard_Failure& e) {
        LOG_ERR("OCC exception: " + std::string(e.GetMessageString()));
        return std::nullopt;
    } catch (const std::exception& e) {
        LOG_ERR("std exception: " + std::string(e.what()));
        return std::nullopt;
    }
}

void StepModel::buildInstanceTree() {
    LOG_SCOPED_TIMER("buildInstanceTree");
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
        const TDF_Label& freeLabel = freeShapes.Value(i);
        fillNode(freeLabel, freeLabel, "", identity, -1, 0, cursor);
    }

    assert(cursor == numNodes); // should hit all the nodes we counted in pass 1

    int leaves = 0, assemblies = 0;
    for (const auto& n : partNodes) {
        if (n.type == PartNodeType::Leaf)     leaves++;
        if (n.type == PartNodeType::Assembly) assemblies++;
    }

    LOG_INFO("Instance tree built: " + std::to_string(partNodes.size()) + " nodes");
    LOG_INFO("  Assemblies:          " + std::to_string(assemblies));
    LOG_INFO("  Leaves:              " + std::to_string(leaves));
    LOG_INFO("  Unique definitions:  " + std::to_string(definitionShapes.size()));
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
    const TDF_Label& instLabel,
    const TDF_Label& defLabel,
    const std::string& parentName,
    const gp_Trsf& parentWorld,
    int parentIdx,
    int depth,
    int& cursor
) {
    if (!shapeTool->IsShape(instLabel)) return;

    // Extract just this label's own location
    // world is computed here purely so children can extract their own localTrsf
    // correctly relative to their parent; it is never stored.
    TopLoc_Location loc = shapeTool->GetLocation(instLabel);
    gp_Trsf localTrsf = loc.Transformation();
    gp_Trsf world = parentWorld * localTrsf;

    if (shapeTool->IsComponent(instLabel)) {
        TDF_Label defLabel;
        if (!shapeTool->GetReferredShape(instLabel, defLabel)) return;

        std::string instanceName = getLabelName(instLabel);
        if (instanceName.empty()) instanceName = parentName;

        if (shapeTool->IsSimpleShape(defLabel)) {
            fillLeaf(instLabel, defLabel, instanceName, localTrsf, parentIdx, depth, cursor);
        } else if (shapeTool->IsAssembly(defLabel)) {
            fillAssembly(instLabel, defLabel, instanceName, localTrsf, world, parentIdx, depth, cursor);
        } else {
            fillLeaf(instLabel, defLabel, instanceName, localTrsf, parentIdx, depth, cursor);
        }

    } else if (shapeTool->IsAssembly(instLabel)) {
        fillAssembly(instLabel, defLabel, "", localTrsf, world, parentIdx, depth, cursor);

    } else if (shapeTool->IsSimpleShape(instLabel)) {
        fillLeaf(instLabel, defLabel, "", localTrsf, parentIdx, depth, cursor);
    }
}

void StepModel::fillLeaf(
    const TDF_Label& instLabel,
    const TDF_Label& defLabel,
    const std::string& parentName,
    const gp_Trsf& localTrsf,
    int parentIdx,
    int depth,
    int& cursor
) {
    int myIdx = cursor++;

    std::string defName = getLabelName(defLabel);
    std::string resolved;
    if (!isAutoGeneratedName(parentName)) {
        resolved = parentName;
    } else if (!isAutoGeneratedName(defName)) {
        resolved = defName;
    } else {
        resolved = parentName;
    }
    
    partNodes[myIdx].name = resolved;

    // std::ostringstream e, pe;
    // defLabel.EntryDump(e);
    // defLabel.Father().EntryDump(pe);
    // LOG_DEBUG("fillLeaf: name=" + resolved 
    //     + " entry=" + e.str() 
    //     + " parent=" + pe.str()
    //     + " shapeType=" + std::to_string(shapeTool->GetShape(defLabel).ShapeType()));

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
    partNodes[myIdx].instanceLabel    = instLabel;
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
    if (definitionNames.find(defLabel) == definitionNames.end()) {
        definitionNames[defLabel] = partNodes[myIdx].name;
    }
}

void StepModel::fillAssembly(
    const TDF_Label& instLabel,
    const TDF_Label& defLabel,
    const std::string& parentName,
    const gp_Trsf& localTrsf,
    const gp_Trsf& world,
    int parentIdx,
    int depth,
    int& cursor
) {
    int myIdx = cursor++;

    std::string defName = getLabelName(defLabel);

    std::string resolved;
    if (!isAutoGeneratedName(parentName)) {
        resolved = parentName;
    } else if (!isAutoGeneratedName(defName)) {
        resolved = defName;
    } else {
        resolved = parentName;
    }

    partNodes[myIdx].name = resolved;
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
    partNodes[myIdx].instanceLabel    = instLabel;
    partNodes[myIdx].localTransform   = localTrsf;
    partNodes[myIdx].parentIdx        = parentIdx;
    partNodes[myIdx].firstChildIdx    = firstChild;
    partNodes[myIdx].childCount       = validChildren;
    partNodes[myIdx].depth            = depth;

    for (int i = 1; i <= components.Length(); i++) {
        TDF_Label child = components.Value(i);
        // pass resolved assembly name as fallback for nameless children
        fillNode(child, child, partNodes[myIdx].name, world, myIdx, depth + 1, cursor);
    }
}

// Debug 
void StepModel::debugPrintInstances() const {
    for (size_t i = 0; i < partNodes.size(); i++) {
        const PartNode& inst = partNodes[i];
        std::string type = (inst.type == PartNodeType::Assembly) ? "ASM" : "LEAF";

        std::string shapeType;
        switch (shapeTool->GetShape(inst.definitionLabel).ShapeType()) {
            case TopAbs_COMPOUND:  shapeType = "COMPOUND"; break;
            case TopAbs_COMPSOLID: shapeType = "COMPSOLID"; break;
            case TopAbs_SOLID:     shapeType = "SOLID"; break;
            case TopAbs_SHELL:     shapeType = "SHELL"; break;
            case TopAbs_FACE:      shapeType = "FACE"; break;
            case TopAbs_WIRE:      shapeType = "WIRE"; break;
            case TopAbs_EDGE:      shapeType = "EDGE"; break;
            case TopAbs_VERTEX:    shapeType = "VERTEX"; break;
            default:               shapeType = "UNKNOWN"; break;
        }

        std::ostringstream oss;
        inst.definitionLabel.EntryDump(oss);
        std::string dump = oss.str();

        gp_XYZ lt = inst.localTransform.TranslationPart();
        std::cout << "[" << i << "] " << type
                  << " name=" << inst.name
                  << " shapeType=" << shapeType
                  << " dump=" << dump
                  << " parent="     << inst.parentIdx
                  << " firstChild=" << inst.firstChildIdx
                  << " childCount=" << inst.childCount
                  << " depth="      << inst.depth
                  << " localT=("    << lt.X() << "," << lt.Y() << "," << lt.Z() << ")"
                  << "\n";
    }
}