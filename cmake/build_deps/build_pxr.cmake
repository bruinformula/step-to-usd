set_cache (pxr_BUILD_VERSION 26.08 "OpenUSD version for local builds")
set (pxr_GIT_REPOSITORY "https://github.com/PixarAnimationStudios/OpenUSD")
set (pxr_GIT_TAG "v${pxr_BUILD_VERSION}")
set (pxr_GIT_COMMIT "ee47c679abde5b467a7b6a41f3b2285564a4222e")

set_cache (pxr_BUILD_SHARED_LIBS ${LOCAL_BUILD_SHARED_LIBS_DEFAULT}
           DOC "Should a local OpenUSD build, if necessary, build shared libraries" ADVANCED)

string (MAKE_C_IDENTIFIER ${pxr_BUILD_VERSION} pxr_VERSION_IDENT)

build_dependency_with_cmake(pxr
    VERSION         ${pxr_BUILD_VERSION}
    GIT_REPOSITORY  ${pxr_GIT_REPOSITORY}
    GIT_COMMIT      ${pxr_GIT_COMMIT}
    GIT_TAG         ${pxr_GIT_TAG}
    VERBOSE         ${VERBOSE}
    CMAKE_ARGS
        -G "${CMAKE_GENERATOR}"
        -D BUILD_SHARED_LIBS=${pxr_BUILD_SHARED_LIBS}
        -D PXR_BUILD_MONOLITHIC=ON

	-D PXR_BUILD_DOCUMENTATION=OFF
        -D PXR_BUILD_TESTS=OFF
        -D PXR_BUILD_EXAMPLES=OFF
        -D PXR_BUILD_TUTORIALS=OFF
        -D PXR_BUILD_USD_TOOLS=OFF
        -D PXR_BUILD_IMAGING=OFF
        -D PXR_BUILD_USD_IMAGING=OFF
        -D PXR_BUILD_USDVIEW=OFF
        -D PXR_ENABLE_PYTHON_SUPPORT=OFF
        -D PXR_ENABLE_GL_SUPPORT=OFF
        -D PXR_ENABLE_PTEX_SUPPORT=OFF
        -D PXR_ENABLE_OPENVDB_SUPPORT=OFF

        -D PXR_EXTERNAL_NAMESPACE=pxr_${PROJ_NAMESPACE_V}
        -D PXR_INTERNAL_NAMESPACE=pxr_${PROJ_NAMESPACE_V}_v${pxr_VERSION_IDENT}
)

set (pxr_ROOT "${${PROJ_NAME_UPPER}_LOCAL_DEPS_ROOT}/dist")
set (pxr_REFIND TRUE)
# pxr_REFIND_ARGS intentionally left unset: refind must go through Findpxr.cmake
# so that pxr::pxr INTERFACE target is created and pxr_VERSION is set to dotted form.

# DO NOT set pxr_REFIND_VERSION here. 
# Leaving it unset prevents find_package from looking for non-existent pxrConfigVersion.cmake
unset (pxr_REFIND_VERSION)