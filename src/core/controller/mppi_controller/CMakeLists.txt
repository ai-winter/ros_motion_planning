cmake_minimum_required(VERSION 3.0.2)
project(mppi_controller)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  angles
  costmap_2d
  geometry_msgs
  tf2_geometry_msgs
  nav_msgs
  nav_core
  pluginlib
  roscpp
  tf2_ros
  base_local_planner
  controller
  system_config
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS controller system_config
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/mppi_controller.cpp
  src/footprint_collision_checker.cpp
  src/noise_generator.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_library(mppi_critics SHARED
  src/critics/constraint_critic.cpp
  src/critics/cost_critic.cpp
)

target_link_libraries(mppi_critics
  ${catkin_LIBRARIES}
)
