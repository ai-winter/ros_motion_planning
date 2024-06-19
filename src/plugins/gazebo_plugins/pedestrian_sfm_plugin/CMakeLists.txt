cmake_minimum_required(VERSION 3.0.2)
project(gazebo_sfm_plugin)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  message_generation
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)

add_service_files(
  FILES
  ped_state.srv
)

generate_messages(
  DEPENDENCIES
)

include_directories(include)
include_directories(SYSTEM
  # /usr/local/include  #to find lightsfm
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
  LIBRARIES PedestrianSFMPlugin
  CATKIN_DEPENDS gazebo_ros roscpp
)


add_library(PedestrianSFMPlugin src/pedestrian_sfm_plugin.cpp)
add_dependencies(PedestrianSFMPlugin gazebo_sfm_plugin_generate_messages_cpp)
target_link_libraries(PedestrianSFMPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}) #${Boost_LIBRARIES

install(TARGETS
  PedestrianSFMPlugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

