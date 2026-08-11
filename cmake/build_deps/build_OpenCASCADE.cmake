set_cache (OpenCASCADE_BUILD_VERSION 7_9_3 "OpenCASCADE version for local builds")
set (OpenCASCADE_GIT_REPOSITORY "https://github.com/Open-Cascade-SAS/OCCT")
set (OpenCASCADE_GIT_TAG "V${OpenCASCADE_BUILD_VERSION}")
set (OpenCASCADE_GIT_COMMIT "a016080bf6738d6aeae020badee4e888ad1540a5")
set_cache (OpenCASCADE_BUILD_SHARED_LIBS ${LOCAL_BUILD_SHARED_LIBS_DEFAULT}
           DOC "Should a local OpenCASCADE build, if necessary, build shared libraries" ADVANCED)

string (MAKE_C_IDENTIFIER ${OpenCASCADE_BUILD_VERSION} OpenCASCADE_VERSION_IDENT)

build_dependency_with_cmake(OpenCASCADE
    VERSION         ${OpenCASCADE_BUILD_VERSION}
    GIT_REPOSITORY  ${OpenCASCADE_GIT_REPOSITORY}
    GIT_TAG         ${OpenCASCADE_GIT_TAG}
    GIT_COMMIT      ${OpenCASCADE_GIT_COMMIT}
    VERBOSE         ${VERBOSE}
    CMAKE_ARGS
        -G "${CMAKE_GENERATOR}"
        -D BUILD_SHARED_LIBS=${OpenCASCADE_BUILD_SHARED_LIBS}
        
        -D BUILD_MODULE_Draw=OFF
        -D BUILD_DOC_Overview=OFF
        -D BUILD_SAMPLES_MFC=OFF
        -D BUILD_SAMPLES_QT=OFF

        -D USE_FREETYPE=OFF
)

set (OpenCASCADE_ROOT "${${PROJ_NAME_UPPER}_LOCAL_DEPS_ROOT}/dist")
set (OpenCASCADE_REFIND TRUE)
set (OpenCASCADE_REFIND_ARGS CONFIG)
unset (OpenCASCADE_REFIND_VERSION)

if (OpenCASCADE_BUILD_SHARED_LIBS)
    install_local_dependency_libs (OpenCASCADE TKernel)
    install_local_dependency_libs (OpenCASCADE TKMath)
    install_local_dependency_libs (OpenCASCADE TKG2d)
    install_local_dependency_libs (OpenCASCADE TKG3d)
    install_local_dependency_libs (OpenCASCADE TKGeomBase)
    install_local_dependency_libs (OpenCASCADE TKBRep)
    install_local_dependency_libs (OpenCASCADE TKTopAlgo)
    install_local_dependency_libs (OpenCASCADE TKSTEP)
    install_local_dependency_libs (OpenCASCADE TKMesh)
endif ()