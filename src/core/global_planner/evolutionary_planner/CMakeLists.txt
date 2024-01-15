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
  curve_generation
  utils
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS global_planner curve_generation utils
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/aco.cpp
  src/pso.cpp
  src/ga.cpp
  src/evolutionary_planner.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
