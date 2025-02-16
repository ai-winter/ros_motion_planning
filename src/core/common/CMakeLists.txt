cmake_minimum_required(VERSION 3.0.2)
project(common)

get_filename_component(PROJECT_ROOT_DIR ${CMAKE_SOURCE_DIR} DIRECTORY)
include(${PROJECT_ROOT_DIR}/3rd/conanbuildinfo.cmake)
conan_basic_setup()

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
)

catkin_package(
	INCLUDE_DIRS   include
	LIBRARIES      ${PROJECT_NAME}
	CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

file(GLOB util_src_files src/util/*.cpp)
file(GLOB math_src_files_cpp src/math/*.cpp)
file(GLOB geometry_src_files src/geometry/*.cpp)
file(GLOB geometry_curve_src_files src/geometry/curve/*.cpp)

add_library(${PROJECT_NAME}
  ${util_src_files}
  ${math_src_files_cpp}
  ${geometry_src_files}
  ${geometry_curve_src_files}
)

add_dependencies(${PROJECT_NAME} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${CONAN_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CONAN_LIBS}
)

# install
install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)