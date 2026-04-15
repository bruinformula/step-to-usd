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

# step-to-usd
add_library(step-to-usd SHARED ${AUTOLIB_GEOM_SOURCES})

target_link_libraries(step-to-usd PUBLIC
    autolibStepTessellation
    usd
    usdGeom
    sdf
    tf
    ${OpenCASCADE_LIBRARIES}
)

install(TARGETS step-to-usd
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

# StepConvertUsd
add_executable(StepConvertUsd "${CMAKE_CURRENT_SOURCE_DIR}/StepConvertUsd.cpp")
target_link_libraries(StepConvertUsd PRIVATE step-to-usd)

install(TARGETS StepConvertUsd
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
