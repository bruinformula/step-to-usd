###########################################################################
# CMake module to find OpenCASCADE (OCCT)
#
# Inputs / Hints:
#   OpenCASCADE_ROOT / OPENCASCADE_ROOT_DIR / CASROOT
#
# Components:
#   TKernel TKMath TKG2d TKG3d TKBRep TKGeomBase TKTopAlgo (and other TK* libs)
#
# Output Variables:
#   OPENCASCADE_FOUND          - True if requested components are found
#   OPENCASCADE_INCLUDES       - Header search paths
#   OPENCASCADE_LIBRARIES      - All found component libraries
#   OPENCASCADE_VERSION        - Formatted version string 
#
# Imported Targets:
#   TKernel, TKMath, TKBRep, etc.
###########################################################################

# First, try to find the standard CMake config file provided by recent OCCT builds
find_package(OpenCASCADE CONFIG QUIET)

if (OpenCASCADE_FOUND OR TARGET TKernel)
    set (FOUND_OPENCASCADE_WITH_CONFIG 1)
    if (NOT OpenCASCADE_FIND_QUIETLY)
        message (STATUS "Found CONFIG for OpenCASCADE (OpenCASCADE_VERSION=${OpenCASCADE_VERSION})")
    endif ()

    set (OPENCASCADE_FOUND TRUE)
    set (OPENCASCADE_VERSION ${OpenCASCADE_VERSION})

    if (TARGET TKernel)
        get_target_property (OPENCASCADE_INCLUDES TKernel INTERFACE_INCLUDE_DIRECTORIES)
        
        # If components were requested, collect their targets. Otherwise, assume core libs.
        set(OPENCASCADE_LIBRARIES)
        if (OpenCASCADE_FIND_COMPONENTS)
            foreach(_comp ${OpenCASCADE_FIND_COMPONENTS})
                if (TARGET ${_comp})
                    list(APPEND OPENCASCADE_LIBRARIES ${_comp})
                endif()
            endforeach()
        else()
            set(OPENCASCADE_LIBRARIES TKernel TKMath TKG2d TKG3d TKBRep TKGeomBase TKTopAlgo)
        endif()
    endif ()

else ()
    # Fallback: no OpenCASCADEConfig.cmake was found, so locate things by hand

    # Resolve root search hints
    if (NOT OpenCASCADE_ROOT)
        if (NOT "$ENV{OpenCASCADE_ROOT}" STREQUAL "")
            set(OpenCASCADE_ROOT "$ENV{OpenCASCADE_ROOT}")
        elseif (NOT "$ENV{OPENCASCADE_ROOT_DIR}" STREQUAL "")
            set(OpenCASCADE_ROOT "$ENV{OPENCASCADE_ROOT_DIR}")
        elseif (NOT "$ENV{CASROOT}" STREQUAL "")
            set(OpenCASCADE_ROOT "$ENV{CASROOT}")
        endif()
    endif()

    set(_opencascade_SEARCH_DIRS
        ${OpenCASCADE_ROOT}
        /usr/local
        /opt/local
        /opt/opencascade
    )

    # Determine components to find (OCCT uses TK* prefix for modules)
    if (NOT OpenCASCADE_FIND_COMPONENTS)
        set(OpenCASCADE_FIND_COMPONENTS TKernel TKMath TKBRep TKG2d TKG3d TKGeomBase TKTopAlgo)
    endif()

    # Find headers using the core version header
    find_path(OPENCASCADE_INCLUDE_DIR
        NAMES Standard_Version.hxx
        HINTS ${_opencascade_SEARCH_DIRS}
        PATH_SUFFIXES include include/opencascade inc
    )

    # Extract version cleanly from Standard_Version.hxx
    if (OPENCASCADE_INCLUDE_DIR AND EXISTS "${OPENCASCADE_INCLUDE_DIR}/Standard_Version.hxx")
        file(STRINGS "${OPENCASCADE_INCLUDE_DIR}/Standard_Version.hxx" _major_line REGEX "^#define OCC_VERSION_MAJOR.*$")
        file(STRINGS "${OPENCASCADE_INCLUDE_DIR}/Standard_Version.hxx" _minor_line REGEX "^#define OCC_VERSION_MINOR.*$")
        file(STRINGS "${OPENCASCADE_INCLUDE_DIR}/Standard_Version.hxx" _patch_line REGEX "^#define OCC_VERSION_MAINTENANCE.*$")

        if (_major_line AND _minor_line AND _patch_line)
            string(REGEX MATCH "[0-9]+" _major "${_major_line}")
            string(REGEX MATCH "[0-9]+" _minor "${_minor_line}")
            string(REGEX MATCH "[0-9]+" _patch "${_patch_line}")
            set(OPENCASCADE_VERSION "${_major}.${_minor}.${_patch}")
        endif()
    endif()

    # Find libraries for requested components
    set(_opencascade_LIBRARIES)
    foreach(_comp ${OpenCASCADE_FIND_COMPONENTS})
        find_library(OPENCASCADE_${_comp}_LIBRARY
            NAMES ${_comp} lib${_comp}
            HINTS ${_opencascade_SEARCH_DIRS}
            PATH_SUFFIXES lib lib64
        )

        if (OPENCASCADE_${_comp}_LIBRARY)
            set(OpenCASCADE_${_comp}_FOUND TRUE)
            list(APPEND _opencascade_LIBRARIES "${OPENCASCADE_${_comp}_LIBRARY}")
        else()
            set(OpenCASCADE_${_comp}_FOUND FALSE)
        endif()

        mark_as_advanced(OPENCASCADE_${_comp}_LIBRARY)
    endforeach()

    # Standard package args handling
    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(OpenCASCADE
        REQUIRED_VARS OPENCASCADE_INCLUDE_DIR _opencascade_LIBRARIES
        VERSION_VAR   OPENCASCADE_VERSION
        HANDLE_COMPONENTS
    )

    # Create Imported Targets
    if (OPENCASCADE_FOUND)
        set(OPENCASCADE_INCLUDES ${OPENCASCADE_INCLUDE_DIR})
        set(OPENCASCADE_LIBRARIES ${_opencascade_LIBRARIES})

        foreach(_comp ${OpenCASCADE_FIND_COMPONENTS})
            if (OPENCASCADE_${_comp}_LIBRARY AND NOT TARGET ${_comp})
                add_library(${_comp} UNKNOWN IMPORTED)
                set_target_properties(${_comp} PROPERTIES
                    IMPORTED_LOCATION "${OPENCASCADE_${_comp}_LIBRARY}"
                    INTERFACE_INCLUDE_DIRECTORIES "${OPENCASCADE_INCLUDE_DIR}"
                )
            endif()
        endforeach()
    endif()

    mark_as_advanced(OPENCASCADE_INCLUDE_DIR)
endif()

if (OpenCASCADE_FOUND AND NOT TARGET OpenCASCADE::OpenCASCADE)
    add_library(OpenCASCADE::OpenCASCADE INTERFACE IMPORTED)
    target_include_directories(OpenCASCADE::OpenCASCADE INTERFACE "${OpenCASCADE_INCLUDES}")
    target_link_libraries(OpenCASCADE::OpenCASCADE INTERFACE "${OpenCASCADE_LIBRARIES}")
endif()