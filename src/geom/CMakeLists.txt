find_package(gmsh REQUIRED)
find_package(pxr REQUIRED)

add_executable(GeomTest "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp")

target_include_directories(GeomTest BEFORE PRIVATE ${pxr_INCLUDES})
target_include_directories(GeomTest BEFORE PRIVATE ${gmsh_INCLUDES})

target_link_libraries(GeomTest PRIVATE
    usd
    usdGeom
    sdf
    tf
)
target_link_libraries(GeomTest PRIVATE
    gmsh::shared
)

set_target_properties(GeomTest PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)