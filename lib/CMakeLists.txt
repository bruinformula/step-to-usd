if (${WITH_QUADRIFLOW})
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/Quadriflow")
endif()

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/CadSchema")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/CadUSD")