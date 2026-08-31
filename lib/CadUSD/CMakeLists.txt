find_package(pxr REQUIRED)
find_package(OpenCASCADE REQUIRED)

file(GLOB_RECURSE STEPUSD_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/**.cpp"
)

# step-to-usd
add_library(CadUSD STATIC ${STEPUSD_SOURCES})

target_link_libraries(CadUSD PUBLIC
    autolibCadTessellation
    pxr::pxr
    OpenCASCADE::OpenCASCADE
)

target_include_directories(CadUSD
    PUBLIC "${CMAKE_SOURCE_DIR}/include"
)

install(TARGETS CadUSD
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
