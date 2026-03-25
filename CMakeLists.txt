cmake_minimum_required(VERSION 3.26)

project(Livery-Shaders VERSION 0.1 LANGUAGES CXX C)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

option(BUILD_AUTOLIB_MTL "Build lookdev tools. Requires MaterialX, OSL, and OpenImageIO" ON)
option(BUILD_AUTOLIB_MTL_ARNOLD "Build arnold BSDFs. Requires Arnold-SDK" ON)
option(BUILD_AUTOLIB_GEOM "Build geometry tools. Requires Usd, TBB (greater than ) and OpenCascade" ON)
option(AUTOLIB_BUILD_STEP_USD_SCHEMA "Build the Usd schema for STEP tessellation parameters. Requires Usd" ON)
option(BUILD_AUTOLIB_SIM "Build simulation tools. Requires Usd and Chrono" OFF)

if (NOT CMAKE_BUILD_TYPE)
    set (CMAKE_BUILD_TYPE "Release")
endif()

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/src")

enable_testing()

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/testsweet")
