set_cache (OpenSubdiv_BUILD_VERSION 3_7_0 "OpenSubdiv version for local builds")
set (OpenSubdiv_GIT_REPOSITORY "https://github.com/PixarAnimationStudios/OpenSubdiv")
set (OpenSubdiv_GIT_TAG "v${OpenSubdiv_BUILD_VERSION}")
set (OpenSubdiv_GIT_COMMIT "9dab8a47bfbb1388ec8388fe61f5f916e6123f38")
set_cache (OpenSubdiv_BUILD_SHARED_LIBS ${LOCAL_BUILD_SHARED_LIBS_DEFAULT}
           DOC "Should a local OpenSubdiv build, if necessary, build shared libraries" ADVANCED)

string (MAKE_C_IDENTIFIER ${OpenSubdiv_BUILD_VERSION} OpenSubdiv_VERSION_IDENT)

build_dependency_with_cmake(OpenSubdiv
    VERSION         ${OpenSubdiv_BUILD_VERSION}
    GIT_REPOSITORY  ${OpenSubdiv_GIT_REPOSITORY}
    GIT_TAG         ${OpenSubdiv_GIT_TAG} 
    GIT_COMMIT      ${OpenSubdiv_GIT_COMMIT}
    VERBOSE         ${VERBOSE}
    CMAKE_ARGS
        -G "${CMAKE_GENERATOR}"
        -D BUILD_SHARED_LIBS=${OpenSubdiv_BUILD_SHARED_LIBS}

        -D NO_EXAMPLES=ON
        -D NO_TUTORIALS=ON
        -D NO_TESTS=ON
        -D NO_DOC=ON
        -D NO_REGRESSION=ON
        
        # GPU Backend Configuration ---
        -D NO_OPENGL=ON
        -D NO_CUDA=ON
        -D NO_METAL=ON
        -D NO_PTEX=ON
        -D NO_MACOS_FRAMEWORK=ON

        # Enable patch shader generation if using GPU evaluators
        -D OSD_PATCH_SHADER_SOURCE_GLSL=OFF
        -D OSD_PATCH_SHADER_SOURCE_HLSL=OFF
        -D OSD_PATCH_SHADER_SOURCE_MSL=OFF
)

set (OpenSubdiv_ROOT "${${PROJ_NAME_UPPER}_LOCAL_DEPS_ROOT}/dist")
set (OpenSubdiv_REFIND TRUE)
set (OpenSubdiv_REFIND_ARGS CONFIG)

# DO NOT set OpenSubdiv_REFIND_VERSION here. 
# Leaving it unset prevents find_package from looking for non-existent OpenSubdivConfigVersion.cmake
unset (OpenSubdiv_REFIND_VERSION)

if (OpenSubdiv_BUILD_SHARED_LIBS)
    install_local_dependency_libs (OpenSubdiv OpenSubdiv)
endif ()