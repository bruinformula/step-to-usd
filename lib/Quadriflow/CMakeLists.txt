option(BUILD_LOG "Enable verbose log" OFF)
option(BUILD_OPENMP "Enable support for OpenMP" OFF)
option(BUILD_TBB "Enable support for TBB" OFF)
option(BUILD_FREE_LICENSE "Only use libraries with permissive licenses" OFF)

set(Boost_USE_STATIC_LIBS ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g")
set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS}")
set(CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG}")

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS
        "${CMAKE_CXX_FLAGS} -Wall -fopenmp -Wno-int-in-bool-context -Wno-sign-compare"
    )
    set(CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address"
    )
    set(CMAKE_LINKER_FLAGS
        "${CMAKE_LINKER_FLAGS}"
    )
    set(CMAKE_LINKER_FLAGS_DEBUG
        "${CMAKE_LINKER_FLAGS_DEBUG} -fsanitize=address"
    )
endif()

if(MSVC)
    add_definitions(-D_USE_MATH_DEFINES -D_CRT_SECURE_NO_WARNINGS)
    set(CMAKE_CXX_FLAGS
        "${CMAKE_CXX_FLAGS} /W3 /wd4267 /wd4244 /wd4018 /wd4800"
    )
endif()

# ---------------------------------------------------------------------------
# LEMON 1.3.1
# ---------------------------------------------------------------------------

set(LEMON_SOURCE_ROOT_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/3rd/lemon-1.3.1"
)

set(LEMON_SOURCE_DIR
    "${LEMON_SOURCE_ROOT_DIR}/lemon"
)

# LEMON's config.h is generated from config.h.in.
# This is required because LEMON headers include <lemon/config.h>.
set(LEMON_GENERATED_DIR
    "${CMAKE_CURRENT_BINARY_DIR}/3rd/lemon-1.3.1"
)

file(MAKE_DIRECTORY
    "${LEMON_GENERATED_DIR}/lemon"
)

set(PROJECT_VERSION "1.3.1")

configure_file(
    "${LEMON_SOURCE_DIR}/config.h.in"
    "${LEMON_GENERATED_DIR}/lemon/config.h"
)

unset(PROJECT_VERSION)

set(LEMON_INCLUDE_DIRS
    "${LEMON_SOURCE_ROOT_DIR}"
    "${LEMON_GENERATED_DIR}"
)

# Quadriflow sources
set(
    quadriflow_SRC
    adjacent-matrix.cpp
    dedge.cpp
    hierarchy.cpp
    loader.cpp
    localsat.cpp
    main.cpp
    merge-vertex.cpp
    optimizer.cpp
    parametrizer.cpp
    parametrizer-flip.cpp
    parametrizer-int.cpp
    parametrizer-mesh.cpp
    parametrizer-scale.cpp
    parametrizer-sing.cpp
    subdivide.cpp
)

if (WITH_CUDA)
    set(CMAKE_CUDA_ARCHITECTURES "75;86;89")
    enable_language(CUDA)
    list(APPEND quadriflow_SRC src/Optimizer.cu)
endif()

add_library(
    quadriflow
    ${quadriflow_SRC}
)

target_include_directories(quadriflow PUBLIC
    "${CMAKE_SOURCE_DIR}/include"
    "${CMAKE_SOURCE_DIR}/lib/Quadriflow/3rd/pcg32"
    ${LEMON_INCLUDE_DIRS}
)

if (BUILD_OPENMP)
    add_definitions(-DWITH_OMP)
endif()

if (BUILD_LOG)
    add_definitions(-DLOG_OUTPUT)
endif()

if (BUILD_TBB)
    add_definitions(-DWITH_TBB)
endif()

if (BUILD_FREE_LICENSE)
    add_definitions(-DEIGEN_MPL2_ONLY)
endif()

if (WITH_CUDA)
    find_package(CUDAToolkit REQUIRED)
    add_definitions(-DWITH_CUDA)
endif()

add_definitions(-DLOG_OUTPUT)

target_link_libraries(
    quadriflow
    Eigen3::Eigen
    Boost::boost
)

if (BUILD_TBB)
    target_link_libraries(
        quadriflow
        TBB::tbb
    )
endif()

if (WITH_CUDA)
    target_link_libraries(
        quadriflow
        CUDA::cudart
    )
endif()