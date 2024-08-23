cmake_minimum_required(VERSION 2.8.3)
project(spencer_social_relation_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
   FILES
   SocialRelation.msg
   SocialRelations.msg
   SocialActivity.msg
   SocialActivities.msg
)

# Generate services in the 'srv' folder
#add_service_files(
#  FILES
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
   sensor_msgs
   nav_msgs
   std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs geometry_msgs message_runtime 
)
