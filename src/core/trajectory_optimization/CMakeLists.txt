cmake_minimum_required(VERSION 3.0.2)
project(trajectory_optimization)

find_package(catkin REQUIRED COMPONENTS
  angles
  roscpp
  voronoi_layer
  utils
)

find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS voronoi_layer utils
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/optimizer.cpp
  src/conjugate_optimizer.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
