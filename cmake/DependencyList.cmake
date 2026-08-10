
include_guard()

# When not in VERBOSE mode, try to make things as quiet as possible
if (VERBOSE)
    set (PkgConfig_FIND_QUIETLY true)
    set (Threads_FIND_QUIETLY true)
endif ()

if (VERBOSE)
    message (STATUS "${ColorBoldWhite}")
    message (STATUS "* Checking for dependencies...")
    message (STATUS "*   - Missing a dependency 'Package'?")
    message (STATUS "*     Try cmake -DPackage_ROOT=path or set environment var Package_ROOT=path")
    message (STATUS "*   - To exclude an optional dependency (even if found),")
    message (STATUS "*     -DUSE_Package=OFF or set environment var USE_Package=OFF ")
    message (STATUS "${ColorReset}")
endif ()

if (VERBOSE)
    message (STATUS "CMAKE_PREFIX_PATH = ${CMAKE_PREFIX_PATH}")
endif ()

include (ExternalProject)

checked_find_package(TBB REQUIRED VERSION_MIN 2023 CONFIG)

# OpenSubdiv and USD imaging requires this
checked_find_package(OpenGL REQUIRED)

checked_find_package(OpenSubdiv REQUIRED 
    VERSION_MIN 3.5.1
    RECOMMEND_MIN 3.6.0
    PREFER_CONFIG
)

checked_find_package(pxr REQUIRED 
    VERSION_MIN 0.25.11
    RECOMMEND_MIN 0.26.03
)
