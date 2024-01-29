cmake_minimum_required(VERSION 3.0.2)
project(local_planner)

find_package(catkin REQUIRED COMPONENTS
  angles
  roscpp
  costmap_2d
  geometry_msgs
  utils
  base_local_planner
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES local_planner
 CATKIN_DEPENDS utils
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/local_planner.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
