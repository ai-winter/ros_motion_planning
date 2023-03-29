cmake_minimum_required(VERSION 2.8.3)
project(spencer_tracking_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  geometry_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
   FILES
   DetectedPerson.msg
   DetectedPersons.msg
   CompositeDetectedPerson.msg
   CompositeDetectedPersons.msg
   TrackedPerson.msg
   TrackedPersons.msg
   TrackedPerson2d.msg
   TrackedPersons2d.msg
   TrackedGroup.msg
   TrackedGroups.msg
   ImmDebugInfo.msg
   ImmDebugInfos.msg
   TrackingTimingMetrics.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  GetPersonTrajectories.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime 
)
