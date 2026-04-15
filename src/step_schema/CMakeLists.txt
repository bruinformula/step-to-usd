find_package(pxr REQUIRED)

include(AddUsdSchema)
add_usd_schema("${CMAKE_CURRENT_SOURCE_DIR}" autolibStepTessellation)