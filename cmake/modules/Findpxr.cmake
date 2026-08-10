
###########################################################################
# Module to find OpenUSD (Pixar's Universal Scene Description, a.k.a. "pxr")
#
# This module defines the following variables:
#  pxr_FOUND          - True if OpenUSD is found.
#  pxr_VERSION        - Full OpenUSD version, dotted (e.g. "0.25.11")
#  pxr_VERSION_CODE   - Raw encoded PXR_VERSION integer (e.g. 2511)
#  pxr_INCLUDES       - where to find pxr/pxr.h and friends
#  pxr_LIBRARIES      - list of OpenUSD libraries to link against
#  pxr_LIB_DIR        - where to find OpenUSD libraries
#  pxr_DIRECTORY      - the root of the OpenUSD install
#  pxr_MONOLITHIC     - True if using a single monolithic usd_ms/usd_m lib
#
# The following input symbols may be used to help guide the search:
#  pxr_ROOT           - the root of the OpenUSD installation (if custom)
#  pxr_STATIC         - if true, will prefer static OpenUSD libs

# Allow several conventional ways to point us at a custom install
if (NOT pxr_ROOT)
    if (OpenUSD_ROOT)
        set (pxr_ROOT ${OpenUSD_ROOT})
    elseif (DEFINED ENV{OpenUSD_ROOT})
        set (pxr_ROOT $ENV{OpenUSD_ROOT})
    elseif (USD_ROOT)
        set (pxr_ROOT ${USD_ROOT})
    elseif (DEFINED ENV{pxr_ROOT})
        set (pxr_ROOT $ENV{pxr_ROOT})
    elseif (DEFINED ENV{USD_ROOT})
        set (pxr_ROOT $ENV{USD_ROOT})
    endif ()
endif ()

# First choice: every OpenUSD build ships its own pxrConfig.cmake. Prefer
# that, since it already knows exactly which internal libraries exist
# (monolithic or not, with/without Python, etc) and how they depend on
# each other. 
find_package (pxr CONFIG QUIET
              HINTS ${pxr_ROOT}
              PATHS ${pxr_ROOT}/cmake)

if (pxr_FOUND OR PXR_FOUND)

    set (pxr_FOUND true)
    set (pxr_VERSION_CODE ${PXR_VERSION})
    set (pxr_VERSION "${PXR_MAJOR_VERSION}.${PXR_MINOR_VERSION}.${PXR_PATCH_VERSION}")
    set (pxr_INCLUDES ${PXR_INCLUDE_DIRS})
    set (pxr_LIBRARIES ${PXR_LIBRARIES})
    get_filename_component (pxr_DIRECTORY "${PXR_INCLUDE_DIRS}/.." ABSOLUTE)

    if (TARGET usd_ms OR TARGET usd_m)
        set (pxr_MONOLITHIC true)
    else ()
        set (pxr_MONOLITHIC false)
    endif ()

else () # Fallback: no pxrConfig.cmake was found, so locate things by hand.

    find_path (pxr_INCLUDE_DIR
               NAMES pxr/pxr.h
               HINTS ${pxr_ROOT}
               PATH_SUFFIXES include)

    if (pxr_INCLUDE_DIR)
        set (pxr_INCLUDES ${pxr_INCLUDE_DIR})
        get_filename_component (pxr_DIRECTORY "${pxr_INCLUDE_DIR}/.." ABSOLUTE)

        # Pull the encoded version out of pxr/pxr.h, e.g. "#define PXR_VERSION 2511"
        file (STRINGS "${pxr_INCLUDE_DIR}/pxr/pxr.h" _openusd_version_line
              REGEX "define PXR_VERSION ")
        string (REGEX MATCH "([0-9]+)" pxr_VERSION_CODE "${_openusd_version_line}")
        if (pxr_VERSION_CODE)
            math (EXPR pxr_VERSION_MAJOR "${pxr_VERSION_CODE} / 10000")
            math (EXPR pxr_VERSION_MINOR "(${pxr_VERSION_CODE} % 10000) / 100")
            math (EXPR pxr_VERSION_PATCH "${pxr_VERSION_CODE} % 100")
            set (pxr_VERSION "${pxr_VERSION_MAJOR}.${pxr_VERSION_MINOR}.${pxr_VERSION_PATCH}")
        endif ()
    endif ()

    # Prefer a monolithic library if one was built (common for USD builds
    # configured with PXR_BUILD_MONOLITHIC=ON).
    find_library (pxr_MONOLITHIC_LIBRARY
                  NAMES usd_ms usd_m
                  HINTS ${pxr_ROOT}
                  PATH_SUFFIXES lib)

    if (pxr_MONOLITHIC_LIBRARY)
        set (pxr_MONOLITHIC true)
        set (pxr_LIBRARIES ${pxr_MONOLITHIC_LIBRARY})
        get_filename_component (pxr_LIB_DIR "${pxr_MONOLITHIC_LIBRARY}" DIRECTORY)
    else ()
        set (pxr_MONOLITHIC false)
        # A reasonably complete set of the individual libraries that make up
        # a non-monolithic OpenUSD build. Not every project needs all of
        # these, but this is a safe "find everything available" default.
        set (_openusd_components
             arch tf gf js trace work plug vt ar kind sdf pcp usd
             usdGeom usdVol usdMedia usdShade usdLux usdRender usdHydra
             usdSkel usdUI usdUtils usdPhysics hf hio cameraUtil pxOsd
             glf hgi hgiInterop hd hdsi hdSt hdx usdImaging usdImagingGL)
        foreach (_openusd_lib ${_openusd_components})
            find_library (_pxr_${_openusd_lib}_LIBRARY
                          NAMES ${_openusd_lib}
                          HINTS ${pxr_ROOT}
                          PATH_SUFFIXES lib)
            if (_pxr_${_openusd_lib}_LIBRARY)
                list (APPEND pxr_LIBRARIES ${_pxr_${_openusd_lib}_LIBRARY})
                if (NOT pxr_LIB_DIR)
                    get_filename_component (pxr_LIB_DIR "${_pxr_${_openusd_lib}_LIBRARY}" DIRECTORY)
                endif ()
            endif ()
            mark_as_advanced (_pxr_${_openusd_lib}_LIBRARY)
        endforeach ()
    endif ()
endif ()

mark_as_advanced (
    pxr_INCLUDE_DIR
    pxr_MONOLITHIC_LIBRARY
    )

include (FindPackageHandleStandardArgs)
find_package_handle_standard_args (pxr
    REQUIRED_VARS
        pxr_INCLUDES
        pxr_LIBRARIES
    VERSION_VAR pxr_VERSION
    )

if (pxr_FOUND AND NOT TARGET pxr::pxr)
    add_library(pxr::pxr INTERFACE IMPORTED)
    target_include_directories(pxr::pxr INTERFACE "${pxr_INCLUDES}")
    target_link_libraries(pxr::pxr INTERFACE "${pxr_LIBRARIES}")
endif()