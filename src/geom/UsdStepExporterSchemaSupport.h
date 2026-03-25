#pragma once 

#include "UsdStepExporter.h"

PXR_NAMESPACE_USING_DIRECTIVE

// Usd Tess Params Resolution
template <typename T>
bool getInheritedAttribute(const UsdPrim& prim, const TfToken& attrName, T* value, const UsdTimeCode& time);

template <typename T>
void updateIfAuthored(const UsdAttribute& attr, T* value);

// Read token attrs as TfToken, then convert to your enum
template <>
void updateIfAuthored<CurveType>(const UsdAttribute& attr, CurveType* value);

template <>
void updateIfAuthored<CurveSampling>(const UsdAttribute& attr, CurveSampling* value);

TessParams getResolvedTessParams(const UsdPrim& prim);