cmake_minimum_required(VERSION 3.26)

set(CMAKE_CUDA_ARCHITECTURES 86)

project(Livery-Shaders VERSION 0.1 LANGUAGES CXX C)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_AUTOLIB_MTL "Build lookdev tools. Requires MaterialX, OSL, and OpenImageIO" ON)
option(BUILD_AUTOLIB_MTL_ARNOLD "Build arnold BSDFs. Requires Arnold-SDK" ON)
option(BUILD_AUTOLIB_GEOM "Build geometry tools. Requires Usd and OpenCascade" ON)
option(BUILD_AUTOLIB_SIM "Build simulation tools. Requires Usd and Chrono" OFF)


if (NOT CMAKE_BUILD_TYPE)
    set (CMAKE_BUILD_TYPE "Release")
endif()

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/src")

enable_testing()

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/testsweet")
