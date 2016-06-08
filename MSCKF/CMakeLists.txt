cmake_minimum_required(VERSION 2.8)
project(demo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x -ffast-math -O3 -march=native -fopenmp -DNDEBUG")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")
find_package(OpenCV REQUIRED)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../Eigen/cmake")
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(demo utils.cpp imuState.cpp MSCKF.cpp main.cpp)

target_link_libraries(demo ${OpenCV_LIBS})
