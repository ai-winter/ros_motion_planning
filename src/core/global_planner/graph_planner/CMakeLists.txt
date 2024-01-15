cmake_minimum_required(VERSION 3.0.2)
project(graph_planner)

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
  voronoi_layer
  utils
  curve_generation
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS global_planner voronoi_layer utils curve_generation
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/graph_planner.cpp
  src/a_star.cpp
  src/jump_point_search.cpp
  src/d_star.cpp
  src/lpa_star.cpp
  src/d_star_lite.cpp
  src/voronoi.cpp
  src/theta_star.cpp
  src/lazy_theta_star.cpp
  src/hybrid_a_star.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
