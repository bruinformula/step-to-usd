find_package(pxr REQUIRED)
find_package(OpenCASCADE REQUIRED)

file(GLOB STEPUSD_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Routine/*.cpp"
)

# step-to-usd
add_library(StepUSD STATIC ${STEPUSD_SOURCES})

target_link_libraries(StepUSD PUBLIC
    autolibStepTessellation
    usd
    usdGeom
    sdf
    tf
    ${OpenCASCADE_LIBRARIES}
)

target_include_directories(StepUSD
    PUBLIC "${CMAKE_SOURCE_DIR}/include"
)

install(TARGETS StepUSD
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
