cmake_minimum_required(VERSION 2.8.3)
project(dvs_vlc_ros)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  rospy
  roscpp
  dvs_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include 
  CATKIN_DEPENDS
    roscpp
    nav_msgs
    dvs_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/dvs_vlc_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)