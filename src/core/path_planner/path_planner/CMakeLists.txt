cmake_minimum_required(VERSION 3.0.2)
project(path_planner)

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
  voronoi_layer
  common
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS voronoi_layer common
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB path_planner_src_files src/*.cpp)
file(GLOB graph_path_planner_src_files src/graph_planner/*.cpp)
file(GLOB sample_path_planner_src_files src/sample_planner/*.cpp)
file(GLOB evolutionary_path_planner_src_files src/evolutionary_planner/*.cpp)

## Declare a C++ library
add_library(${PROJECT_NAME}
  ${path_planner_src_files}
  ${graph_path_planner_src_files}
  ${sample_path_planner_src_files}
  ${evolutionary_path_planner_src_files}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
