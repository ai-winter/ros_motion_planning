cmake_minimum_required(VERSION 2.8.3)
project(gazebo_ped_visualizer_plugin)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  pedsim_msgs
  std_msgs
  gazebo_sfm_plugin
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)


include_directories(include)
include_directories(SYSTEM
  /usr/local/include  #to find lightsfm
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES PedestrianVisualPlugin
  CATKIN_DEPENDS gazebo_ros roscpp
)

add_library(PedestrianVisualPlugin src/pedestrian_visual_plugin.cpp)
add_dependencies(PedestrianVisualPlugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(PedestrianVisualPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}) #${Boost_LIBRARIES

install(TARGETS
  PedestrianVisualPlugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)



