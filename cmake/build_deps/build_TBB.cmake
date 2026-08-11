set_cache (TBB_BUILD_VERSION 2023.1.0 "oneTBB version for local builds")
set (TBB_GIT_REPOSITORY "https://github.com/uxlfoundation/oneTBB")
set (TBB_GIT_TAG "v${TBB_BUILD_VERSION}")
set (TBB_GIT_COMMIT "3046c8b0c29df995980003ea24f4d78c80ec0c8d")
set_cache (TBB_BUILD_SHARED_LIBS ${LOCAL_BUILD_SHARED_LIBS_DEFAULT}
           DOC "Should a local oneTBB build, if necessary, build shared libraries" ADVANCED)

string (MAKE_C_IDENTIFIER ${TBB_BUILD_VERSION} TBB_VERSION_IDENT)

build_dependency_with_cmake(TBB
    VERSION         ${TBB_BUILD_VERSION}
    GIT_REPOSITORY  ${TBB_GIT_REPOSITORY}
    GIT_TAG         ${TBB_GIT_TAG}
    GIT_COMMIT      ${TBB_GIT_COMMIT}
    VERBOSE         ${VERBOSE}
    CMAKE_ARGS
        -G "${CMAKE_GENERATOR}"
        -D BUILD_SHARED_LIBS=${TBB_BUILD_SHARED_LIBS}

        -D TBB_TEST=OFF
        -D TBB_EXAMPLES=OFF
        -D TBB_STRICT=OFF
        -D TBB4PY_BUILD=OFF
        -D TBBMALLOC_BUILD=OFF
)

set (TBB_ROOT "${${PROJ_NAME_UPPER}_LOCAL_DEPS_ROOT}/dist" CACHE PATH "Path to local TBB installation" FORCE)
set (TBB_REFIND TRUE)
set (TBB_REFIND_ARGS CONFIG)
set (TBB_REFIND_VERSION ${TBB_BUILD_VERSION})

if (TBB_BUILD_SHARED_LIBS)
    install_local_dependency_libs (TBB tbb)
endif ()