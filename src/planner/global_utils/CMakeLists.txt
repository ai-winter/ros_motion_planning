cmake_minimum_required(VERSION 3.0.2)
project(global_utils)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES a_star
#  CATKIN_DEPENDS global_utils
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/global_planner.cpp
  src/utils.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
