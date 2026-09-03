find_package(pxr REQUIRED)
find_package(OpenCASCADE REQUIRED)

# frack
add_executable(frack "${CMAKE_CURRENT_SOURCE_DIR}/frick.cpp")
target_link_libraries(frack PRIVATE CadUSD)

target_include_directories(frack
    PUBLIC "${CMAKE_SOURCE_DIR}/include"
)

install(TARGETS frack
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

# frick 
add_executable(frick "${CMAKE_CURRENT_SOURCE_DIR}/frick.cpp")
target_link_libraries(frick PRIVATE CadUSD)

target_include_directories(frick
    PUBLIC "${CMAKE_SOURCE_DIR}/include"
)
install(TARGETS frick
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