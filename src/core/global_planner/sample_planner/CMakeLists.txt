cmake_minimum_required(VERSION 3.0.2)
project(sample_planner)

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
  visualization_msgs
  global_planner
  utils
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS global_planner utils
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/sample_planner.cpp
  src/rrt.cpp
  src/rrt_star.cpp
  src/rrt_connect.cpp
  src/informed_rrt.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
