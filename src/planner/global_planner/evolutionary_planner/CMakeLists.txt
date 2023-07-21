cmake_minimum_required(VERSION 3.0.2)
project(evolutionary_planner)

find_package(catkin REQUIRED COMPONENTS
  angles
  roscpp
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  navfn
  pluginlib
  tf2_geometry_msgs
  tf2_ros
  global_planner
)

catkin_package(
 INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/evolutionary_planner.cpp
  src/aco.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  global_planner
)
