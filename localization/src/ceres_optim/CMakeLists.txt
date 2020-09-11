cmake_minimum_required(VERSION 2.8)
project(localize)
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")
add_executable(localize localize.cpp)

target_link_libraries(localize ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})