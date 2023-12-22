cmake_minimum_required(VERSION 3.0.2)
project(dwa_planner)

find_package(catkin REQUIRED COMPONENTS
  angles
  base_local_planner
  cmake_modules
  costmap_2d
  dynamic_reconfigure
  nav_core
  nav_msgs
  pluginlib
  sensor_msgs
  roscpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

find_package(Eigen3 REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/DWAPlanner.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dwa_planner
  CATKIN_DEPENDS
    base_local_planner
    dynamic_reconfigure
    nav_msgs
    pluginlib
    sensor_msgs
    roscpp
    tf2
    tf2_ros
)

add_library(${PROJECT_NAME}
  src/dwa_planner.cpp
  src/dwa.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
