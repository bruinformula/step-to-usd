# Copyright Contributors to the Open Image IO / OpenShadingLanguage / Pixar projects.
# SPDX-License-Identifier: BSD-3-Clause

###########################################################################
# CMake module to find OpenSubdiv
#
# Inputs / Hints:
#   OpenSubdiv_ROOT / OPENSUBDIV_ROOT_DIR / OPENSUBDIVHOME
#   OPENSUBDIV_USE_GPU - Set to TRUE to automatically search for osdGPU
#
# Components:
#   osdCPU 
#   osdGPU
#
# Output Variables:
#   OPENSUBDIV_FOUND            - True if requested components are found
#   OPENSUBDIV_INCLUDES         - Header search paths
#   OPENSUBDIV_LIBRARIES        - All found component libraries
#   OPENSUBDIV_VERSION          - Formatted version string 
#   OPENSUBDIV_HAS_CUDA         - True if CUDA evaluator is available
#   OPENSUBDIV_HAS_GLSL_COMPUTE - True if GLSL Compute evaluator is available
#   OPENSUBDIV_HAS_OPENCL       - True if OpenCL evaluator is available
#   OPENSUBDIV_HAS_METAL        - True if Metal evaluator is available
#
# Imported Targets:
#   OpenSubdiv::osdCPU
#   OpenSubdiv::osdGPU
###########################################################################

find_package(OpenSubdiv CONFIG QUIET)

if (TARGET OpenSubdiv::osdCPU OR TARGET OpenSubdiv::osdGPU)
    set (FOUND_OPENSUBDIV_WITH_CONFIG 1)
    if (NOT OpenSubdiv_FIND_QUIETLY)
        message (STATUS "Found CONFIG for OpenSubdiv (OpenSubdiv_VERSION=${OpenSubdiv_VERSION})")
    endif ()

    set (OPENSUBDIV_FOUND TRUE)
    set (OPENSUBDIV_VERSION ${OpenSubdiv_VERSION})

    if (TARGET OpenSubdiv::osdCPU)
        get_target_property (OPENSUBDIV_INCLUDES OpenSubdiv::osdCPU INTERFACE_INCLUDE_DIRECTORIES)
        list (APPEND OPENSUBDIV_LIBRARIES OpenSubdiv::osdCPU)
    endif ()
    if (TARGET OpenSubdiv::osdGPU)
        if (NOT OPENSUBDIV_INCLUDES)
            get_target_property (OPENSUBDIV_INCLUDES OpenSubdiv::osdGPU INTERFACE_INCLUDE_DIRECTORIES)
        endif ()
        list (APPEND OPENSUBDIV_LIBRARIES OpenSubdiv::osdGPU)
    endif ()

    # The official config doesn't publish these capability flag, but we can 
    # still infer them by checking
    # for the corresponding headers alongside whatever include dir we found.
    if (OPENSUBDIV_INCLUDES)
        macro (_check_osd_evaluator header variable)
            if (EXISTS "${OPENSUBDIV_INCLUDES}/opensubdiv/osd/${header}")
                set (${variable} TRUE)
            else ()
                set (${variable} FALSE)
            endif ()
        endmacro ()

        _check_osd_evaluator ("glComputeEvaluator.h"     OPENSUBDIV_HAS_GLSL_COMPUTE)
        _check_osd_evaluator ("glXFBEvaluator.h"          OPENSUBDIV_HAS_GLSL_TRANSFORM_FEEDBACK)
        _check_osd_evaluator ("cudaEvaluator.h"           OPENSUBDIV_HAS_CUDA)
        _check_osd_evaluator ("clEvaluator.h"             OPENSUBDIV_HAS_OPENCL)
        _check_osd_evaluator ("mtlEvaluator.h"            OPENSUBDIV_HAS_METAL)
        _check_osd_evaluator ("hlslPatchShaderSource.h"   OPENSUBDIV_HAS_HLSL)
    endif ()

else ()
    # Fallback: no OpenSubdivConfig.cmake was found, so locate things by hand

    # Resolve root search hints
    if (NOT OpenSubdiv_ROOT)
        if (NOT "$ENV{OpenSubdiv_ROOT}" STREQUAL "")
            set(OpenSubdiv_ROOT "$ENV{OpenSubdiv_ROOT}")
        elseif (NOT "$ENV{OPENSUBDIV_ROOT_DIR}" STREQUAL "")
            set(OpenSubdiv_ROOT "$ENV{OPENSUBDIV_ROOT_DIR}")
        elseif (NOT "$ENV{OPENSUBDIVHOME}" STREQUAL "")
            set(OpenSubdiv_ROOT "$ENV{OPENSUBDIVHOME}")
        endif()
    endif()

    set(_opensubdiv_SEARCH_DIRS
        ${OpenSubdiv_ROOT}
        /usr/local
        /opt/local
        /opt/lib/opensubdiv
        /opt/lib/osd
    )

    # Determine components to find
    if (NOT OpenSubdiv_FIND_COMPONENTS)
        set(OpenSubdiv_FIND_COMPONENTS osdCPU)
        if (OPENSUBDIV_USE_GPU)
            list(APPEND OpenSubdiv_FIND_COMPONENTS osdGPU)
        endif()
    endif()

    # Find headers
    find_path(OPENSUBDIV_INCLUDE_DIR
        NAMES opensubdiv/osd/mesh.h
        HINTS ${_opensubdiv_SEARCH_DIRS}
        PATH_SUFFIXES include
    )

    # Extract version cleanly (Fixes "3_7_0" invalid version string error)
    if (OPENSUBDIV_INCLUDE_DIR AND EXISTS "${OPENSUBDIV_INCLUDE_DIR}/opensubdiv/version.h")
        file(STRINGS "${OPENSUBDIV_INCLUDE_DIR}/opensubdiv/version.h" _major_line REGEX "^#define OPENSUBDIV_VERSION_MAJOR.*$")
        file(STRINGS "${OPENSUBDIV_INCLUDE_DIR}/opensubdiv/version.h" _minor_line REGEX "^#define OPENSUBDIV_VERSION_MINOR.*$")
        file(STRINGS "${OPENSUBDIV_INCLUDE_DIR}/opensubdiv/version.h" _patch_line REGEX "^#define OPENSUBDIV_VERSION_PATCH.*$")

        if (_major_line AND _minor_line AND _patch_line)
            string(REGEX MATCH "[0-9]+" _major "${_major_line}")
            string(REGEX MATCH "[0-9]+" _minor "${_minor_line}")
            string(REGEX MATCH "[0-9]+" _patch "${_patch_line}")
            set(OPENSUBDIV_VERSION "${_major}.${_minor}.${_patch}")
        else()
            file(STRINGS "${OPENSUBDIV_INCLUDE_DIR}/opensubdiv/version.h" _ver_line REGEX "^#define OPENSUBDIV_VERSION.*$")
            string(REGEX MATCH "[0-9]+_[0-9]+_[0-9]+" _raw_ver "${_ver_line}")
            string(REPLACE "_" "." OPENSUBDIV_VERSION "${_raw_ver}")
        endif()
    endif()

    # Find libraries for requested components (osdCPU, osdGPU)
    set(_opensubdiv_LIBRARIES)
    foreach(_comp ${OpenSubdiv_FIND_COMPONENTS})
        string(TOUPPER ${_comp} _comp_upper)

        find_library(OPENSUBDIV_${_comp_upper}_LIBRARY
            NAMES ${_comp} lib${_comp}
            HINTS ${_opensubdiv_SEARCH_DIRS}
            PATH_SUFFIXES lib lib64
        )

        if (OPENSUBDIV_${_comp_upper}_LIBRARY)
            set(OpenSubdiv_${_comp}_FOUND TRUE)
            list(APPEND _opensubdiv_LIBRARIES "${OPENSUBDIV_${_comp_upper}_LIBRARY}")
        else()
            set(OpenSubdiv_${_comp}_FOUND FALSE)
        endif()

        mark_as_advanced(OPENSUBDIV_${_comp_upper}_LIBRARY)
    endforeach()

    # Check GPU Backend Capabilities in Headers
    if (OPENSUBDIV_INCLUDE_DIR)
        macro(_check_osd_evaluator header variable)
            if (EXISTS "${OPENSUBDIV_INCLUDE_DIR}/opensubdiv/osd/${header}")
                set(${variable} TRUE)
            else()
                set(${variable} FALSE)
            endif()
        endmacro()

        _check_osd_evaluator("glComputeEvaluator.h" OPENSUBDIV_HAS_GLSL_COMPUTE)
        _check_osd_evaluator("glXFBEvaluator.h" OPENSUBDIV_HAS_GLSL_TRANSFORM_FEEDBACK)
        _check_osd_evaluator("cudaEvaluator.h" OPENSUBDIV_HAS_CUDA)
        _check_osd_evaluator("clEvaluator.h" OPENSUBDIV_HAS_OPENCL)
        _check_osd_evaluator("mtlEvaluator.h" OPENSUBDIV_HAS_METAL)
        _check_osd_evaluator("hlslPatchShaderSource.h" OPENSUBDIV_HAS_HLSL)
    endif()

    # Standard package args handling
    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(OpenSubdiv
        REQUIRED_VARS OPENSUBDIV_INCLUDE_DIR _opensubdiv_LIBRARIES
        VERSION_VAR   OPENSUBDIV_VERSION
        HANDLE_COMPONENTS
    )

    # Create Imported Targets
    if (OPENSUBDIV_FOUND)
        set(OPENSUBDIV_INCLUDES ${OPENSUBDIV_INCLUDE_DIR})
        set(OPENSUBDIV_LIBRARIES ${_opensubdiv_LIBRARIES})

        # CPU Target
        if (OPENSUBDIV_OSDCPU_LIBRARY AND NOT TARGET OpenSubdiv::osdCPU)
            add_library(OpenSubdiv::osdCPU UNKNOWN IMPORTED)
            set_target_properties(OpenSubdiv::osdCPU PROPERTIES
                IMPORTED_LOCATION "${OPENSUBDIV_OSDCPU_LIBRARY}"
                INTERFACE_INCLUDE_DIRECTORIES "${OPENSUBDIV_INCLUDE_DIR}"
            )
        endif()

        # GPU Target
        if (OPENSUBDIV_OSDGPU_LIBRARY AND NOT TARGET OpenSubdiv::osdGPU)
            add_library(OpenSubdiv::osdGPU UNKNOWN IMPORTED)
            set_target_properties(OpenSubdiv::osdGPU PROPERTIES
                IMPORTED_LOCATION "${OPENSUBDIV_OSDGPU_LIBRARY}"
                INTERFACE_INCLUDE_DIRECTORIES "${OPENSUBDIV_INCLUDE_DIR}"
            )
        endif()
    endif()

    mark_as_advanced(OPENSUBDIV_INCLUDE_DIR)
endif()