find_package(pxr REQUIRED)
find_package(OpenCASCADE REQUIRED)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/tess_schema")


set(AUTOLIB_GEOM_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/UsdStepExporter.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/StepModel.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/UsdStepExporterSchemaSupport.cpp"
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

add_executable(test_schema test.cpp)

target_link_libraries(test_schema PRIVATE
    autolibStepTessellation
    tf 
    sdf 
    usd
    AutolibGeom
)

# TODO: Get to this later
#add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/instant_meshes")

#add_executable(InstantMeshes "${CMAKE_CURRENT_SOURCE_DIR}/InstantMeshes.cpp")
#target_link_libraries(InstantMeshes PRIVATE AutolibInstantMeshes)

