cmake_minimum_required(VERSION 3.0.2)
project(lazy_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  geometry_msgs
  nav_core
  pluginlib
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES lazy_planner
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/lazy_planner.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
