include_guard()

# The "outer namespace" defaults to the project name, but it can be overridden
# to allow custom builds that put everything inside a unique namespace that
# can't conflict with default builds.
set (${PROJ_NAME}_OUTER_NAMESPACE ${PROJ_NAME_UPPER} CACHE STRING
     "Customized outer namespace")
set (PROJ_NAMESPACE "${${PROJ_NAME}_OUTER_NAMESPACE}")  # synonym
if (NOT ${PROJ_NAME}_OUTER_NAMESPACE STREQUAL ${PROJ_NAME_UPPER})
    set (${PROJ_NAME}_CUSTOM_OUTER_NAMESPACE 1)
endif ()
# There is also an inner namespace that is either vMAJ_MIN or vMAJ_MIN_PATCH,
# depending on the setting of ${PROJ_NAME}_INNER_NAMESPACE_INCLUDE_PATCH.
option (${PROJ_NAME}_INNER_NAMESPACE_INCLUDE_PATCH
        "Should the inner namespace include the patch number" ${${PROJ_NAME_UPPER}_DEV_RELEASE})
if (${PROJ_NAME}_INNER_NAMESPACE_INCLUDE_PATCH)
    set (PROJ_VERSION_NAMESPACE "v${PROJECT_VERSION_MAJOR}_${PROJECT_VERSION_MINOR}_${PROJECT_VERSION_PATCH}")
else ()
    set (PROJ_VERSION_NAMESPACE "v${PROJECT_VERSION_MAJOR}_${PROJECT_VERSION_MINOR}")
endif ()
# PROJ_NAMESPACE_V combines the outer and inner namespaces into one symbol
set (PROJ_NAMESPACE_V "${PROJ_NAMESPACE}_${PROJ_VERSION_NAMESPACE}")

if (VERBOSE)
message(STATUS "Outer namespace PROJ_OUTER_NAMESPACE:   ${PROJ_NAMESPACE}")
message(STATUS "Inner namespace PROJ_VERSION_NAMESPACE: ${PROJ_VERSION_NAMESPACE}")
message(STATUS "Joint namespace PROJ_NAMESPACE_V:       ${PROJ_NAMESPACE_V}")
endif ()