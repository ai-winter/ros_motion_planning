cmake_minimum_required(VERSION 3.0.2)
project(sim_env)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	message_generation
	controller_manager
	gazebo_ros
	rviz
)

catkin_package(
   CATKIN_DEPENDS urdf xacro
)