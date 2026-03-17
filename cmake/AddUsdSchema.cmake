find_program(USDGENSCHEMA usdGenSchema REQUIRED)

function(add_usd_schema SCHEMA_SOURCE_DIR SCHEMA_NAME)

    # Run usdGenSchema
    set(_schema_file "${SCHEMA_SOURCE_DIR}/schema.usda")
    set(_gen_dir     "${CMAKE_CURRENT_BINARY_DIR}/generated")

    file(MAKE_DIRECTORY "${_gen_dir}")
    file(TIMESTAMP "${_schema_file}" _ts "%s")

    if(NOT DEFINED _USD_SCHEMA_TS_${SCHEMA_NAME} OR
       NOT _USD_SCHEMA_TS_${SCHEMA_NAME} STREQUAL _ts)

        execute_process(
            COMMAND "${Python3_EXECUTABLE}" "${USDGENSCHEMA}"
                    "${_schema_file}" "${_gen_dir}"
            RESULT_VARIABLE _result
            OUTPUT_VARIABLE _stdout
            ERROR_VARIABLE  _stderr)

        if(NOT _result EQUAL 0)
            message(FATAL_ERROR "usdGenSchema failed:\n${_stdout}\n${_stderr}")
        endif()

        set(_USD_SCHEMA_TS_${SCHEMA_NAME} "${_ts}" CACHE INTERNAL "")
    endif()

    # Collect generated C++
    file(GLOB _all_cpp  "${_gen_dir}/*.cpp")
    file(GLOB _wrap_cpp "${_gen_dir}/wrap*.cpp")
    set(_lib_cpp ${_all_cpp})
    list(REMOVE_ITEM _lib_cpp ${_wrap_cpp})

    file(GLOB _headers "${_gen_dir}/*.h")

    # Shared library
    add_library(${SCHEMA_NAME} SHARED ${_lib_cpp} ${_headers})

    string(TOUPPER "${SCHEMA_NAME}" _upper)
    target_compile_definitions(${SCHEMA_NAME} PRIVATE ${_upper}_EXPORTS)

    target_include_directories(${SCHEMA_NAME} PUBLIC
        $<BUILD_INTERFACE:${_gen_dir}>
        $<BUILD_INTERFACE:${SCHEMA_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include/${SCHEMA_NAME}>)

    if(TARGET usd_ms)
        target_link_libraries(${SCHEMA_NAME} PUBLIC usd_ms)
    else()
        target_link_libraries(${SCHEMA_NAME} PUBLIC tf sdf usd)
    endif()

    set_target_properties(${SCHEMA_NAME} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
        PUBLIC_HEADER "${_headers}"
    )

    # Configure plugInfo.json for the build tree
    set(_lib_filename
        "${CMAKE_SHARED_LIBRARY_PREFIX}${SCHEMA_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX}")

    set(_res_dir "${CMAKE_BINARY_DIR}/plugin/usd/${SCHEMA_NAME}/resources")
    file(MAKE_DIRECTORY "${_res_dir}")

    file(RELATIVE_PATH _lib_rel "${_res_dir}" "${CMAKE_BINARY_DIR}/lib")

    set(PLUG_INFO_ROOT          ".")
    set(PLUG_INFO_RESOURCE_PATH ".")
    set(PLUG_INFO_LIBRARY_PATH  "${_lib_rel}/${_lib_filename}")

    configure_file("${_gen_dir}/plugInfo.json" "${_res_dir}/plugInfo.json" @ONLY)
    configure_file("${_gen_dir}/generatedSchema.usda"
                   "${_res_dir}/generatedSchema.usda" COPYONLY)

    # Install
    set(_install_res "plugin/usd/${SCHEMA_NAME}/resources")

    file(RELATIVE_PATH _install_lib_rel
        "${CMAKE_INSTALL_PREFIX}/${_install_res}"
        "${CMAKE_INSTALL_PREFIX}/lib")

    set(PLUG_INFO_LIBRARY_PATH "${_install_lib_rel}/${_lib_filename}")
    configure_file("${_gen_dir}/plugInfo.json"
                   "${_gen_dir}/plugInfo.json.install" @ONLY)

    install(TARGETS ${SCHEMA_NAME}
        EXPORT        "${SCHEMA_NAME}Targets"
        RUNTIME       DESTINATION lib
        LIBRARY       DESTINATION lib
        ARCHIVE       DESTINATION lib
        PUBLIC_HEADER DESTINATION "include/${SCHEMA_NAME}")

    install(EXPORT "${SCHEMA_NAME}Targets"
        NAMESPACE "${SCHEMA_NAME}::" DESTINATION cmake)

    install(FILES "${_gen_dir}/plugInfo.json.install"
        DESTINATION "${_install_res}" RENAME plugInfo.json)

    install(FILES "${_gen_dir}/generatedSchema.usda"
        DESTINATION "${_install_res}")

    message(STATUS "add_usd_schema: '${SCHEMA_NAME}' ready")
    message(STATUS "  PXR_PLUGINPATH_NAME=${CMAKE_BINARY_DIR}/plugin/usd")

endfunction()