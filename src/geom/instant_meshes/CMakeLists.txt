option(INSTANT_MESHES_DEV_MODE "Instant meshes developer mode" OFF)

if (INSTANT_MESHES_DEV_MODE)
  add_definitions(-DDEV_MODE)
endif()

file(GLOB INSTANT_MESHES_CPP_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
)

add_library(AutolibInstantMeshes ${INSTANT_MESHES_CPP_SOURCES})

find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)

target_link_libraries(AutolibInstantMeshes PUBLIC Eigen3::Eigen)
target_link_libraries(AutolibInstantMeshes PUBLIC TBB::tbb)
