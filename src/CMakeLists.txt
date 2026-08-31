find_package(pxr REQUIRED)
find_package(OpenCASCADE REQUIRED)

# stepmap
add_executable(stepmap "${CMAKE_CURRENT_SOURCE_DIR}/stepmap.cpp")
target_link_libraries(stepmap PRIVATE CadUSD)

target_include_directories(stepmap
    PUBLIC "${CMAKE_SOURCE_DIR}/include"
)

install(TARGETS stepmap
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

# vroom
add_executable(vroom "${CMAKE_CURRENT_SOURCE_DIR}/vroom.cpp")
target_link_libraries(vroom PRIVATE CadUSD)

target_include_directories(vroom
    PUBLIC "${CMAKE_SOURCE_DIR}/include"
)

install(TARGETS vroom
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
# stepxbf
add_executable(stepxbf "${CMAKE_CURRENT_SOURCE_DIR}/stepxbf.cpp")
target_link_libraries(stepxbf PRIVATE OpenCASCADE::OpenCASCADE)

target_include_directories(stepxbf
    PUBLIC "${CMAKE_SOURCE_DIR}/include"
)
install(TARGETS stepxbf
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
