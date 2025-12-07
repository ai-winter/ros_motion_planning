cmake_minimum_required(VERSION 2.8.3)
project(system_config)

add_compile_options(-std=c++17)

find_package(Protobuf 3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  common
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES      ${PROJECT_NAME}
  CATKIN_DEPENDS common
)

add_library(${PROJECT_NAME}
  src/proto_util.cpp
  src/system_config.cpp
)


set(PROTO_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/protos)
file(MAKE_DIRECTORY ${PROTO_OUTPUT_DIR})

set(PROTO_SRCS "")
set(PROTO_HDRS "")

# common related protos
file(GLOB_RECURSE COMMON_PROTOS "${CMAKE_CURRENT_SOURCE_DIR}/system_config/common_protos/*.proto")
protobuf_generate(
  TARGET ${PROJECT_NAME}
  LANGUAGE cpp
  OUT_VAR COMMON_PROTO_SRCS
  PROTOS ${COMMON_PROTOS}
)
list(APPEND PROTO_SRCS ${COMMON_PROTO_SRCS})

# path planner related protos
file(GLOB_RECURSE PATH_PLANNER_PROTOS "${CMAKE_CURRENT_SOURCE_DIR}/system_config/path_planner_protos/*.proto")
protobuf_generate(
  TARGET ${PROJECT_NAME}
  LANGUAGE cpp
  OUT_VAR PATH_PLANNER_PROTO_SRCS
  PROTOS ${PATH_PLANNER_PROTOS}
)
list(APPEND PROTO_SRCS ${PATH_PLANNER_PROTO_SRCS})

# controller related protos
file(GLOB CONTROLLER_PROTOS "${CMAKE_CURRENT_SOURCE_DIR}/system_config/controller_protos/*.proto")
protobuf_generate(
  TARGET ${PROJECT_NAME}
  LANGUAGE cpp
  OUT_VAR CONTROLLER_PROTO_SRCS
  PROTOS ${CONTROLLER_PROTOS}
)
list(APPEND PROTO_SRCS ${CONTROLLER_PROTO_SRCS})

# system configure related protos
file(GLOB SYSTEM_CONFIG_PROTOS "${CMAKE_CURRENT_SOURCE_DIR}/system_config/*.proto")
protobuf_generate(
  TARGET ${PROJECT_NAME}
  LANGUAGE cpp
  OUT_VAR SYSTEM_CONFIG_PROTO_SRCS
  PROTOS ${SYSTEM_CONFIG_PROTOS}
)
list(APPEND PROTO_SRCS ${SYSTEM_CONFIG_PROTO_SRCS})


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${PROTOBUF_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}
)