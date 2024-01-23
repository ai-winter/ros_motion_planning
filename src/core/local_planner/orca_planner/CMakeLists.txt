cmake_minimum_required(VERSION 3.0.2)
project(orca_planner)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_core
  pluginlib
  roscpp
  base_local_planner
)

catkin_package(
  INCLUDE_DIRS include
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
