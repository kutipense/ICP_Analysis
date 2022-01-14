cmake_minimum_required(VERSION 3.16)
project(icp_analysis)               
set(CMAKE_CXX_STANDARD 17)          

find_package(PCL REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
    include
    lib
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(LIB_FILES src/data_loader/ply_loader.cpp src/data_loader/rgbd_loader.cpp)
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
)
