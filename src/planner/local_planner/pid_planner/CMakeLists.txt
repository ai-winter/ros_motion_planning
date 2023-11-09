cmake_minimum_required(VERSION 3.0.2)
project(pid_planner)

find_package(catkin REQUIRED COMPONENTS
  angles
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  navfn
  pluginlib
  roscpp
  tf2_geometry_msgs
  tf2_ros
  base_local_planner
  local_planner
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS local_planner
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/pid_planner.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
