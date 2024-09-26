cmake_minimum_required(VERSION 3.0.2)
project(apf_controller)

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
  controller
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS controller
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/apf_controller.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
