cmake_minimum_required(VERSION 3.16)
project(icp_analysis)               
set(CMAKE_CXX_STANDARD 17)          

find_package(PCL REQUIRED)

include_directories(
    include
    lib
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(LIB_FILES src/data_loader/ply_loader.cpp)
set(SOURCE_FILES src/main.cpp)

add_library(ICPModules ${LIB_FILES})

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ICPModules ${PCL_LIBRARIES} ${PCL_FILTERS_LIBRARIES})
