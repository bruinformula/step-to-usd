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

target_include_directories(stepmap
    PUBLIC "${CMAKE_SOURCE_DIR}/include"
)

install(TARGETS stepmesh
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
