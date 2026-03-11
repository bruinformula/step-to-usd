#pragma once 

#include <string>

#include <opencascade/TDF_Label.hxx>
#include <opencascade/TDataStd_Name.hxx>
#include <opencascade/gp_Trsf.hxx>
#include <opencascade/TopoDS_Face.hxx>
#include <opencascade/Poly_Triangulation.hxx>

#pragma push_macro("Handle")
#undef Handle

#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/vec3f.h>

#pragma pop_macro("Handle")

namespace occt = opencascade;

// rotation block: transposed relative to OCC Value(row,col) convention
// translation: from TranslationPart() into the last row
static pxr::GfMatrix4d trsfToGfMatrix(const gp_Trsf& t) {
    gp_XYZ trans = t.TranslationPart();
    auto clean = [](double v) { return std::abs(v) < 1e-10 ? 0.0 : v; };
    return pxr::GfMatrix4d(
        clean(t.Value(1,1)), clean(t.Value(2,1)), clean(t.Value(3,1)), 0.0,
        clean(t.Value(1,2)), clean(t.Value(2,2)), clean(t.Value(3,2)), 0.0,
        clean(t.Value(1,3)), clean(t.Value(2,3)), clean(t.Value(3,3)), 0.0,
        clean(trans.X()),    clean(trans.Y()),    clean(trans.Z()),    1.0
    );
}

static std::string getLabelName(const TDF_Label& label) {
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

static std::string sanitizeUsdName(const std::string_view& name, int idx) {
    if (name.empty()) return "Node_" + std::to_string(idx);

    std::string result;
    result.reserve(name.size());

    for (char c : name) {
        if (std::isalnum(c) || c == '_') result += c;
        else result += '_'; // replace hyphens, spaces, dots, etc.
    }

    // Usd prim names must start with a letter or underscore
    if (!result.empty() && std::isdigit(result[0]))
        result = "_" + result;

    if (result.empty()) 
        return "Node_";

    return result + std::to_string(idx);
}