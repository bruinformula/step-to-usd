find_package(pxr REQUIRED)
find_package(OpenCASCADE REQUIRED)

#StepConvertUsd

add_executable(StepConvertUsd "${CMAKE_CURRENT_SOURCE_DIR}/StepConvertUsd.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/StepModel.cpp" )

target_include_directories(StepConvertUsd BEFORE PRIVATE ${pxr_INCLUDES})

target_link_libraries(StepConvertUsd PRIVATE
    usd
    usdGeom
    sdf
    tf
)

target_include_directories(StepConvertUsd PRIVATE ${OpenCASCADE_INCLUDES})
target_link_libraries(StepConvertUsd PRIVATE ${OpenCASCADE_LIBRARIES})

set_target_properties(StepConvertUsd PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)


#StepConvertUsdVariant

add_executable(StepConvertUsdVariant "${CMAKE_CURRENT_SOURCE_DIR}/StepConvertUsdVariant.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/StepModel.cpp" )

target_include_directories(StepConvertUsdVariant BEFORE PRIVATE ${pxr_INCLUDES})
target_include_directories(StepConvertUsdVariant BEFORE PRIVATE ${OpenCASCADE_INCLUDES})

target_link_libraries(StepConvertUsdVariant PRIVATE
    usd
    usdGeom
    sdf
    tf
)

target_include_directories(StepConvertUsdVariant PRIVATE ${OpenCASCADE_INCLUDE_DIR})
target_link_libraries(StepConvertUsdVariant PRIVATE ${OpenCASCADE_LIBRARIES})

set_target_properties(StepConvertUsdVariant PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)