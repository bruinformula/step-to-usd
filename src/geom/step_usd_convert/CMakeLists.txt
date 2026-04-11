find_package(pxr REQUIRED)
find_package(OpenCASCADE REQUIRED)

set(AUTOLIB_GEOM_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/Tessellation.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/StepModel.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/WritePrims.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/UsdUtils.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/PopulateUsd.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/TessellationParams.cpp"
)

# AutolibGeom
add_library(AutolibGeom SHARED ${AUTOLIB_GEOM_SOURCES})

target_link_libraries(AutolibGeom PUBLIC
    autolibStepTessellation
    usd
    usdGeom
    sdf
    tf
    ${OpenCASCADE_LIBRARIES}
)

install(TARGETS AutolibGeom
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

# StepConvertUsd
add_executable(StepConvertUsd "${CMAKE_CURRENT_SOURCE_DIR}/StepConvertUsd.cpp")
target_link_libraries(StepConvertUsd PRIVATE AutolibGeom)

install(TARGETS StepConvertUsd
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
