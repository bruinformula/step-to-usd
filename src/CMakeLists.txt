find_package(pxr REQUIRED)
find_package(OpenCASCADE REQUIRED)

# stepmap
add_executable(stepmap "${CMAKE_CURRENT_SOURCE_DIR}/stepmap.cpp")
target_link_libraries(stepmap PRIVATE StepUSD)

target_include_directories(stepmap
    PUBLIC "${CMAKE_SOURCE_DIR}/include"
)

install(TARGETS stepmap
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

# stepmesh
add_executable(stepmesh "${CMAKE_CURRENT_SOURCE_DIR}/stepmesh.cpp")
target_link_libraries(stepmesh PRIVATE StepUSD)

target_include_directories(stepmesh
    PUBLIC "${CMAKE_SOURCE_DIR}/include"
)

install(TARGETS stepmesh
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

# brepview 
if(APPLE)
add_executable(brepview "${CMAKE_CURRENT_SOURCE_DIR}/brepview.mm")
target_link_libraries(brepview PRIVATE OpenCASCADE::OpenCASCADE)

target_include_directories(brepview
    PUBLIC "${CMAKE_SOURCE_DIR}/include"
)

target_link_libraries(brepview PRIVATE 
    "-framework Foundation"
    "-framework Cocoa"
)

install(TARGETS brepview
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
endif()

add_executable(instmesh "${CMAKE_CURRENT_SOURCE_DIR}/InstantMeshes.cc")
target_link_libraries(instmesh PRIVATE InstantMeshes)

target_include_directories(instmesh
    PUBLIC "${CMAKE_SOURCE_DIR}/include"
)

install(TARGETS instmesh
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)