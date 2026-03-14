find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

find_program(USDGENSCHEMA usdGenSchema REQUIRED)
message(STATUS "usdGenSchema: ${USDGENSCHEMA}")
message(STATUS "Python:        ${Python3_EXECUTABLE}")

# Schema generation
set(SCHEMA_FILE   "${CMAKE_CURRENT_SOURCE_DIR}/schema.usda")
set(GENERATE_DIR  "${CMAKE_CURRENT_BINARY_DIR}/generated")

file(TIMESTAMP "${SCHEMA_FILE}" SCHEMA_TIMESTAMP)

if(NOT SCHEMA_TIMESTAMP STREQUAL "${TESSPARAMS_SCHEMA_LAST_TIMESTAMP}")
    message(STATUS "Running usdGenSchema...")
    file(MAKE_DIRECTORY "${GENERATE_DIR}")

    execute_process(
        COMMAND "${Python3_EXECUTABLE}" "${USDGENSCHEMA}" "${SCHEMA_FILE}" "${GENERATE_DIR}"
        RESULT_VARIABLE GEN_RESULT
        OUTPUT_VARIABLE GEN_OUTPUT
        ERROR_VARIABLE  GEN_ERROR
    )

    if(NOT GEN_RESULT EQUAL 0)
        message(FATAL_ERROR "usdGenSchema failed:\n${GEN_ERROR}")
    endif()

    set(TESSPARAMS_SCHEMA_LAST_TIMESTAMP "${SCHEMA_TIMESTAMP}"
        CACHE INTERNAL "schema.usda timestamp at last successful generation")
endif()

# Glob generated sources
# usdGenSchema emits: <name>.h, <name>.cpp, tokens.h, tokens.cpp,
# wrapTokens.cpp, wrap<Name>.cpp, moduleDeps.cpp, module.cpp
# plus plugInfo.json and generatedSchema.usda into resources/

file(GLOB GENERATED_CPP  "${GENERATE_DIR}/*.cpp")
file(GLOB GENERATED_H    "${GENERATE_DIR}/*.h")

# C++ plugin library
add_library(tessParams SHARED ${GENERATED_CPP})

target_include_directories(tessParams
    PUBLIC
        "$<BUILD_INTERFACE:${GENERATE_DIR}>"
        "$<INSTALL_INTERFACE:include/tessParams>"
)

target_compile_definitions(tessParams PRIVATE
    TESSPARAMS_EXPORTS
    MFB_PACKAGE_NAME=tessParams
    MFB_ALT_PACKAGE_NAME=tessParams
    MFB_PACKAGE_MODULE=TessParams
)

target_compile_options(tessParams PRIVATE -Wno-deprecated -Wno-deprecated-declarations)

# Link USD monolithic 
if(TARGET usd_ms)
    target_link_libraries(tessParams PUBLIC usd_ms Python3::Python)
else()
    target_link_libraries(tessParams PUBLIC tf sdf usd usdGeom Python3::Python)
endif()

# Python bindings module
# usdGenSchema emits wrap*.cpp + module.cpp + moduleDeps.cpp for the bindings.
# These are already included in GENERATED_CPP above via the glob,
# but we need a separate .so with no lib prefix that Python can import.

file(GLOB WRAP_CPP "${GENERATE_DIR}/wrap*.cpp" "${GENERATE_DIR}/module.cpp")
list(REMOVE_ITEM GENERATED_CPP ${WRAP_CPP})  # keep them out of the C++ lib

# Re-define the C++ lib without wrap files
set_property(TARGET tessParams PROPERTY SOURCES ${GENERATED_CPP})

add_library(_tessParams SHARED ${WRAP_CPP})
add_dependencies(_tessParams tessParams)

set_target_properties(_tessParams PROPERTIES
    PREFIX ""
    OUTPUT_NAME "_tessParams"
)

set_target_properties(_tessParams PROPERTIES SUFFIX ".so")

target_include_directories(_tessParams PRIVATE "${GENERATE_DIR}")
target_compile_definitions(_tessParams PRIVATE
    MFB_PACKAGE_NAME=tessParams
    MFB_ALT_PACKAGE_NAME=tessParams
    MFB_PACKAGE_MODULE=TessParams
)

if(TARGET usd_ms)
    target_link_libraries(_tessParams PUBLIC usd_ms Python3::Python tessParams)
else()
    target_link_libraries(_tessParams PUBLIC tf sdf usd Python3::Python tessParams)
endif()

# plugInfo.json substitute the @...@ placeholders
set(PLUG_INFO_IN  "${GENERATE_DIR}/plugInfo.json")
set(PLUG_INFO_OUT "${CMAKE_CURRENT_BINARY_DIR}/plugInfo.json")

# Codeful schemas emit @PLUG_INFO_LIBRARY_PATH@ etc. — substitute them
set(PLUG_INFO_ROOT          ".")
set(PLUG_INFO_RESOURCE_PATH ".")
set(PLUG_INFO_LIBRARY_PATH  "../../libtessParams${CMAKE_SHARED_LIBRARY_SUFFIX}")
configure_file("${PLUG_INFO_IN}" "${PLUG_INFO_OUT}" @ONLY)

# Install
set(PLUGIN_RESOURCE_DIR "plugin/usd/tessParams/resources")

install(TARGETS tessParams
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES ${GENERATED_H}
    DESTINATION "include/tessParams"
)

install(FILES "${PLUG_INFO_OUT}"
    DESTINATION "${PLUGIN_RESOURCE_DIR}"
    RENAME plugInfo.json
)

install(FILES "${GENERATE_DIR}/generatedSchema.usda"
    DESTINATION "${PLUGIN_RESOURCE_DIR}"
)

# Python module goes next to the __init__.py USD expects
install(TARGETS _tessParams
    LIBRARY DESTINATION "lib/python/TessParams"
    RUNTIME DESTINATION "lib/python/TessParams"
)

install(FILES "${GENERATE_DIR}/__init__.py"
    DESTINATION "lib/python/TessParams"
)