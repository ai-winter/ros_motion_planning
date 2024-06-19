cmake_minimum_required(VERSION 2.8.3)
project(pedsim_msgs)

# message and service dependencies
set(MESSAGE_DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

find_package(catkin REQUIRED COMPONENTS message_generation ${MESSAGE_DEPENDENCIES})

include_directories(${catkin_INCLUDE_DIRS})

## Generate messages in the 'msg' folder
add_message_files( DIRECTORY msg
  FILES
  AgentState.msg
  AgentStates.msg
  AgentGroup.msg
  AgentGroups.msg
  AgentForce.msg
  LineObstacle.msg
  LineObstacles.msg
  TrackedPerson.msg
  TrackedPersons.msg
  TrackedGroup.msg
  TrackedGroups.msg
  SocialRelation.msg
  SocialRelations.msg
  SocialActivity.msg
  SocialActivities.msg
  Waypoint.msg
  Waypoints.msg
)

# generate the messages
generate_messages(DEPENDENCIES ${MESSAGE_DEPENDENCIES})


#Declare package run-time dependencies
catkin_package(
    CATKIN_DEPENDS roscpp rospy message_runtime ${MESSAGE_DEPENDENCIES}
)
