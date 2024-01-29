cmake_minimum_required(VERSION 3.0.2)
project(orca_planner)

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

set(RVO_LIBRARY RVO)

set(RVO_HEADERS
  include/RVO/Line.h
  include/RVO/RVO.h
  include/RVO/RVOSimulator.h
  include/RVO/Vector2.h
)

set(RVO_SOURCES
  include/RVO/Agent.cc
  include/RVO/Agent.h
  include/RVO/KdTree.cc
  include/RVO/KdTree.h
  include/RVO/Line.cc
  include/RVO/Obstacle.cc
  include/RVO/Obstacle.h
  include/RVO/RVOSimulator.cc
  include/RVO/Vector2.cc
)

add_library(${RVO_LIBRARY} ${RVO_HEADERS} ${RVO_SOURCES})

add_library(${PROJECT_NAME}
  src/orca_planner.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${RVO_LIBRARY}
)
