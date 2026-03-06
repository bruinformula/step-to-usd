find_package(pxr REQUIRED)
find_package(OpenCASCADE REQUIRED)

#STEPConvertUSD

add_executable(STEPConvertUSD "${CMAKE_CURRENT_SOURCE_DIR}/STEPConvertUSD.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/STEPModel.cpp" )

target_include_directories(STEPConvertUSD BEFORE PRIVATE ${pxr_INCLUDES})
target_include_directories(STEPConvertUSD BEFORE PRIVATE ${OpenCASCADE_INCLUDES})


target_link_libraries(STEPConvertUSD PRIVATE
    usd
    usdGeom
    sdf
    tf
)

target_include_directories(STEPConvertUSD PRIVATE ${OpenCASCADE_INCLUDE_DIR})
target_link_libraries(STEPConvertUSD PRIVATE ${OpenCASCADE_LIBRARIES})

set_target_properties(STEPConvertUSD PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)


#STEPConvertUSDVariant

add_executable(STEPConvertUSDVariant "${CMAKE_CURRENT_SOURCE_DIR}/STEPConvertUSDVariant.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/STEPModel.cpp" )

target_include_directories(STEPConvertUSDVariant BEFORE PRIVATE ${pxr_INCLUDES})
target_include_directories(STEPConvertUSDVariant BEFORE PRIVATE ${OpenCASCADE_INCLUDES})

target_link_libraries(STEPConvertUSDVariant PRIVATE
    usd
    usdGeom
    sdf
    tf
)

target_include_directories(STEPConvertUSDVariant PRIVATE ${OpenCASCADE_INCLUDE_DIR})
target_link_libraries(STEPConvertUSDVariant PRIVATE ${OpenCASCADE_LIBRARIES})

set_target_properties(STEPConvertUSDVariant PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)