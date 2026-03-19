find_package(pxr REQUIRED)
find_package(OpenCASCADE REQUIRED)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/tess_schema")

# AutolibGeom
add_library(AutolibGeom SHARED "${CMAKE_CURRENT_SOURCE_DIR}/StepModel.cpp")
target_include_directories(AutolibGeom PUBLIC
    ${pxr_INCLUDES}
    ${OpenCASCADE_INCLUDE_DIR}
)
target_link_libraries(AutolibGeom PUBLIC
    usd
    usdGeom
    sdf
    tf
    ${OpenCASCADE_LIBRARIES}
)

# StepConvertUsd
add_executable(StepConvertUsd "${CMAKE_CURRENT_SOURCE_DIR}/StepConvertUsd.cpp")
target_include_directories(StepConvertUsd BEFORE PRIVATE ${pxr_INCLUDES})
target_link_libraries(StepConvertUsd PRIVATE AutolibGeom)

# StepConvertUsdVariant
add_executable(StepConvertUsdVariant "${CMAKE_CURRENT_SOURCE_DIR}/StepConvertUsdVariant.cpp")
target_include_directories(StepConvertUsdVariant BEFORE PRIVATE ${pxr_INCLUDES})
target_link_libraries(StepConvertUsdVariant PRIVATE AutolibGeom)

add_executable(test_schema test.cpp)

target_link_libraries(test_schema PRIVATE
    autolibStepTessellation
    tf 
    sdf 
    usd
)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/instant-meshes")