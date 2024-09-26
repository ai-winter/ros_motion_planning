cmake_minimum_required(VERSION 3.0.2)
project(controller)

find_package(catkin REQUIRED COMPONENTS
  angles
  roscpp
  costmap_2d
  geometry_msgs
  common
  base_local_planner
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES controller
 CATKIN_DEPENDS common
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/controller.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
