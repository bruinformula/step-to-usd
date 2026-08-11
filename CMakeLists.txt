cmake_minimum_required(VERSION 3.26)

project(step_to_usd VERSION 0.1 LANGUAGES CXX C)
set (PROJ_NAME ${PROJECT_NAME})    # short name
string (TOLOWER ${PROJ_NAME} PROJ_NAME_LOWER)  # short name lower case
string (TOUPPER ${PROJ_NAME} PROJ_NAME_UPPER)  # short name upper case

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" "${CMAKE_SOURCE_DIR}/cmake/modules")

if (NOT CMAKE_BUILD_TYPE)
    set (CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(Colors)
include(SetUtilities)
include(Namespace)
include(DependencyUtilities)
include(DependencyList)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

#set(CMAKE_CXX_INCLUDE_WHAT_YOU_USE include-what-you-use)
#set(CMAKE_OSX_SYSROOT "$(xcrun --show-sdk-path)")

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/lib")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/src")

#enable_testing()

#add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/testsweet")
