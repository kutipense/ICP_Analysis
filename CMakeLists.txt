project(icp_analysis)               

set(CMAKE_CXX_STANDARD 17)          

include_directories(
    include
    lib
)

set(LIB_FILES src/data_loader/ply_loader.cpp)
set(SOURCE_FILES src/main.cpp)

add_library(ICPModules ${LIB_FILES})

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ICPModules)
