cmake_minimum_required(VERSION 2.8.3)
project(dynamic_xml_config)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
)

catkin_package(
    
)

catkin_install_python(PROGRAMS
    plugins/obstacles_generate_ros.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)