
set_cache (OpenSubdiv_BUILD_VERSION 3_7_0 "OpenSubdiv version for local builds")
set (OpenSubdiv_GIT_REPOSITORY "https://github.com/PixarAnimationStudios/OpenSubdiv")
set (OpenSubdiv_GIT_TAG "v${OpenSubdiv_BUILD_VERSION}")
set (OpenSubdiv_GIT_COMMIT "b92869b932cba8cb4d13ce8210f2c2a4ca90476e")
set_cache (OpenSubdiv_BUILD_SHARED_LIBS ${LOCAL_BUILD_SHARED_LIBS_DEFAULT}
           DOC "Should a local OpenSubdiv build, if necessary, build shared libraries" ADVANCED)

string (MAKE_C_IDENTIFIER ${OpenSubdiv_BUILD_VERSION} OpenSubdiv_VERSION_IDENT)

set_cache (OPENSUBDIV_ENABLE_OPENGL ON  DOC "Enable OpenGL GPU evaluators in OpenSubdiv build")
set_cache (OPENSUBDIV_ENABLE_CUDA   OFF DOC "Enable CUDA GPU evaluators in OpenSubdiv build")
set_cache (OPENSUBDIV_ENABLE_METAL  OFF DOC "Enable Metal GPU evaluators in OpenSubdiv build")

if (OPENSUBDIV_ENABLE_OPENGL)
    set (_osd_no_opengl OFF)
    set (_osd_patch_glsl ON)
    set (_osd_patch_hlsl ON)
else ()
    set (_osd_no_opengl ON)
    set (_osd_patch_glsl OFF)
    set (_osd_patch_hlsl OFF)
endif ()

if (OPENSUBDIV_ENABLE_CUDA)
    set (_osd_no_cuda OFF)
else ()
    set (_osd_no_cuda ON)
endif ()


if (OPENSUBDIV_ENABLE_METAL)
    set (_osd_no_metal OFF)
    set (_osd_patch_msl ON)
else ()
    set (_osd_no_metal ON)
    set (_osd_patch_msl OFF)
endif ()

build_dependency_with_cmake(OpenSubdiv
    VERSION         ${OpenSubdiv_BUILD_VERSION}
    GIT_REPOSITORY  ${OpenSubdiv_GIT_REPOSITORY}
    # A changes on the dev branch enable building with cuda 13 
    # GIT_TAG         ${OpenSubdiv_GIT_TAG} 
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
        -D NO_OPENGL=${_osd_no_opengl}
        -D NO_CUDA=${_osd_no_cuda}
        -D NO_METAL=${_osd_no_metal}
        -D NO_PTEX=ON
        -D NO_MACOS_FRAMEWORK=ON

        # Enable patch shader generation if using GPU evaluators
        -D OSD_PATCH_SHADER_SOURCE_GLSL=${_osd_patch_glsl}
        -D OSD_PATCH_SHADER_SOURCE_HLSL=${_osd_patch_hlsl}
        -D OSD_PATCH_SHADER_SOURCE_MSL=${_osd_patch_msl}
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