cmake_minimum_required(VERSION 3.0.2)
project(common)

# set(CMAKE_CXX_STANDARD 11)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
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

# file(GLOB math_src_fils src/math/*.cc)
file(GLOB util_src_files src/util/*.cpp)
file(GLOB math_src_files_cpp src/math/*.cpp)
file(GLOB geometry_src_files src/geometry/*.cpp)
file(GLOB safety_corridor_src_files src/safety_corridor/*.cpp)

add_library(${PROJECT_NAME}
  # ${math_src_fils}
  ${util_src_files}
  ${math_src_files_cpp}
  ${geometry_src_files}
  ${safety_corridor_src_files}
)

add_dependencies(${PROJECT_NAME} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  glog
  gflags
  protobuf
)

# add_definitions("-Wall -g")
# find_package(Python3 COMPONENTS Development NumPy)
# add_executable(convex_safety_corridor_test test/convex_safety_corridor_test.cpp)
# target_link_libraries(convex_safety_corridor_test
#   ${catkin_LIBRARIES}
#   ${GTEST_LIBRARIES}
#   Python3::Python
#   Python3::NumPy
#   ${PROJECT_NAME} 
# )

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