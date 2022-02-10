cmake_minimum_required(VERSION 3.16)
project(icp_analysis)               
set(CMAKE_CXX_STANDARD 17)          
set(CMAKE_BUILD_TYPE Release)
# add_compile_options(-g3)

find_package(PCL REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Ceres REQUIRED)
# add_subdirectory(lib/yaml-cpp)
include_directories(
    include
    lib
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(LIB_FILES 
  src/data_types/vertex_list.cpp
  src/data_loader/ply_loader.cpp
  src/data_loader/rgbd_loader.cpp
  src/optimizing/linear/symmetric.cpp
  src/optimizing/linear/point_to_plane.cpp
  src/optimizing/linear/point_to_point.cpp
  src/optimizing/linear_optimizer.cpp
  src/optimizing/lm_optimizer.cpp
  src/sampler/uniform_sampler.cpp
  src/sampler/random_sampler.cpp
  src/sampler/covariance_sampler.cpp
  src/sampler/copy_sampler.cpp
  src/discard/reject.cpp
  src/matcher/nearest_neighbor_matcher.cpp
)
set(SOURCE_FILES src/main.cpp)

add_library(FreeImageHelper src/external/FreeImageHelper.cpp)
add_library(ICPModules ${LIB_FILES})

target_link_libraries(FreeImageHelper freeimage)
target_link_libraries(ICPModules FreeImageHelper)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME}
  ICPModules
  FreeImageHelper
  Eigen3::Eigen
  ${PCL_LIBRARIES} 
  ${PCL_FILTERS_LIBRARIES}
  ${CERES_LIBRARIES}
)
