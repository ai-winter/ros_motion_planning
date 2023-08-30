cmake_minimum_required(VERSION 3.0.2)
project(static_planner)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_core
  pluginlib
  roscpp
  base_local_planner
)

# uncomment the following 4 lines to use the Eigen library
# find_package(cmake_modules REQUIRED)
# find_package(Eigen3 REQUIRED)
# include_directories(${EIGEN3_INCLUDE_DIR})
# add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/static_planner.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
