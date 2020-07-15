cmake_minimum_required(VERSION 3.0.2)
set(PROJECT_NAME move_plugin)

project(${PROJECT_NAME})

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
)

find_package(gazebo REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES move_plugin
   CATKIN_DEPENDS gazebo_ros roscpp
#  DEPENDS system_lib
)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
  include
  ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}
)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

## Declare a C++ library
add_library(${PROJECT_NAME}_node SHARED src/MovePlugin.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
