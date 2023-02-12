#!/usr/bin/python
#-*- coding: utf-8 -*-

''' 
******************************************************************************************
*  Copyright (C) 2023 Yang Haodong, All Rights Reserved                                  *
*                                                                                        *
*  @brief    generate launch file dynamicly based on user configure.                     *
*  @author   Haodong Yang                                                                *
*  @version  1.0.1                                                                       *
*  @date     2022.07.06                                                                  *
*  @license  GNU General Public License (GPL)                                            *
******************************************************************************************
'''
import yaml
from xml.dom import minidom
import sys, os

# get the root path of package(src layer)
temp_path = os.path.split(os.path.realpath(__file__))[0]
root_path = temp_path + "/../../"

# @breif: parser user configure file(.yaml)
#
# @param[in]: r_path    the relative path to root
# @retval: user configuer tree
def yamlParser(r_path):
    # the file name to parser(.yaml)
    fileName = sys.argv[1]
    with open(root_path + r_path + fileName + '.yaml', 'r') as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
    return data

# parser user data
user_config = yamlParser('user_config/')
robots_num = len(user_config["robots_config"])

# @breif: create configure file of package `sim_env` dynamically
#
# @param[in]: name      configure file's name
# @param[in]: r_path    the relative path to root
# @retval: None
def writeEnvMainXml(name, r_path):
    # create .launch(ie .xml) document
    doc = minidom.Document()
    root = doc.createElement("launch")
    # creat `main` node
    include_main = doc.createElement("include")
    include_main.setAttribute("file", "$(find sim_env)/launch/config.launch")
    arg_world = doc.createElement("arg")
    arg_world.setAttribute("name", "world")
    arg_world.setAttribute("value", user_config["world"])
    arg_map = doc.createElement("arg")
    arg_map.setAttribute("name", "map")
    arg_map.setAttribute("value", user_config["map"])
    arg_num = doc.createElement("arg")
    arg_num.setAttribute("name", "robot_number")
    arg_num.setAttribute("value", str(robots_num))
    arg_rviz = doc.createElement("arg")
    arg_rviz.setAttribute("name", "rviz_file")
    arg_rviz.setAttribute("value", user_config["rviz_file"])
    include_main.appendChild(arg_world)
    include_main.appendChild(arg_map)
    include_main.appendChild(arg_num)
    include_main.appendChild(arg_rviz)
    # append `main` node to `root` and then append `root` to document
    root.appendChild(include_main)
    doc.appendChild(root)
    with open(root_path + r_path + name + '.launch', 'w') as f:
        doc.writexml(f, addindent="", newl="\n")

# @breif: create configure file to start robots dynamically
# @param[in]: name   configure file's name
# @param[in]: r_path the relative path to root
# @retval: None
def writeRobotsXml(name, r_path):
    # create .launch(ie .xml) document
    doc = minidom.Document()
    root = doc.createElement("launch")
    # create list for parameters
    # setting the number of robots
    arg_num = doc.createElement("arg")
    arg_num.setAttribute("name", "robot_number")
    arg_num.setAttribute("default", str(robots_num))
    root.appendChild(arg_num)
    # setting the parameters of robots
    for i in range(robots_num):
        arg_type = doc.createElement("arg")
        arg_global_planner = doc.createElement("arg")
        arg_local_planner = doc.createElement("arg")
        arg_x_pos = doc.createElement("arg")
        arg_y_pos = doc.createElement("arg")
        arg_z_pos = doc.createElement("arg")
        arg_yaw = doc.createElement("arg")
        arg_type.setAttribute("name", "robot" + str(i + 1) + "_type")
        arg_type.setAttribute("value", user_config["robots_config"][i]["robot" + str(i + 1) + "_type"])
        arg_global_planner.setAttribute("name", "robot" + str(i + 1) + "_global_planner")
        arg_global_planner.setAttribute("value", user_config["robots_config"][i]["robot" + str(i + 1) + "_global_planner"])
        arg_local_planner.setAttribute("name", "robot" + str(i + 1) + "_local_planner")
        arg_local_planner.setAttribute("value", user_config["robots_config"][i]["robot" + str(i + 1) + "_local_planner"])
        arg_x_pos.setAttribute("name", "robot" + str(i + 1) + "_x_pos")
        arg_x_pos.setAttribute("value", user_config["robots_config"][i]["robot" + str(i + 1) + "_x_pos"])
        arg_y_pos.setAttribute("name", "robot" + str(i + 1) + "_y_pos")
        arg_y_pos.setAttribute("value", user_config["robots_config"][i]["robot" + str(i + 1) + "_y_pos"])
        arg_z_pos.setAttribute("name", "robot" + str(i + 1) + "_z_pos")
        arg_z_pos.setAttribute("value", user_config["robots_config"][i]["robot" + str(i + 1) + "_z_pos"])
        arg_yaw.setAttribute("name", "robot" + str(i + 1) + "_yaw")
        arg_yaw.setAttribute("value", user_config["robots_config"][i]["robot" + str(i + 1) + "_yaw"])
        root.appendChild(arg_type)
        root.appendChild(arg_global_planner)
        root.appendChild(arg_local_planner)
        root.appendChild(arg_x_pos)
        root.appendChild(arg_y_pos)
        root.appendChild(arg_z_pos)
        root.appendChild(arg_yaw)
    # create starting node
    start = doc.createElement("include")
    start.setAttribute("file", "$(find sim_env)/launch/app/environment_single.launch.xml")
    arg_type = doc.createElement("arg")
    arg_global_planner = doc.createElement("arg")
    arg_local_planner = doc.createElement("arg")
    arg_ns = doc.createElement("arg")
    arg_start_ns = doc.createElement("arg")
    arg_x_pos = doc.createElement("arg")
    arg_y_pos = doc.createElement("arg")
    arg_z_pos = doc.createElement("arg")
    arg_yaw = doc.createElement("arg")
    arg_type.setAttribute("name", "robot")
    arg_type.setAttribute("value", "$(eval arg('robot' + str(arg('robot_number')) + '_type'))")
    arg_global_planner.setAttribute("name", "global_planner")
    arg_global_planner.setAttribute("value", "$(eval arg('robot' + str(arg('robot_number')) + '_global_planner'))")
    arg_local_planner.setAttribute("name", "local_planner")
    arg_local_planner.setAttribute("value", "$(eval arg('robot' + str(arg('robot_number')) + '_local_planner'))")
    arg_ns.setAttribute("name", "robot_namespace")
    arg_ns.setAttribute("value", "robot$(arg robot_number)")
    arg_start_ns.setAttribute("name", "start_ns")
    arg_x_pos.setAttribute("name", "robot_x")
    arg_x_pos.setAttribute("value", "$(eval arg('robot' + str(arg('robot_number')) + '_x_pos'))")
    arg_y_pos.setAttribute("name", "robot_y")
    arg_y_pos.setAttribute("value", "$(eval arg('robot' + str(arg('robot_number')) + '_y_pos'))")
    arg_z_pos.setAttribute("name", "robot_z")
    arg_z_pos.setAttribute("value", "$(eval arg('robot' + str(arg('robot_number')) + '_z_pos'))")
    arg_yaw.setAttribute("name", "robot_yaw")
    arg_yaw.setAttribute("value", "$(eval arg('robot' + str(arg('robot_number')) + '_yaw'))")
    if robots_num > 1:
        arg_start_ns.setAttribute("value", "true")
    else:
        arg_start_ns.setAttribute("value", "false")
    start.appendChild(arg_type)
    start.appendChild(arg_global_planner)
    start.appendChild(arg_local_planner)
    start.appendChild(arg_ns)
    start.appendChild(arg_start_ns)
    start.appendChild(arg_x_pos)
    start.appendChild(arg_y_pos)
    start.appendChild(arg_z_pos)
    start.appendChild(arg_yaw)
    root.appendChild(start)
    # create judgement node
    judge = doc.createElement("include")
    judge.setAttribute("file", "$(find sim_env)/launch/include/robots/start_robots.launch.xml")
    judge.setAttribute("if", "$(eval arg('robot_number') > 1)")
    arg = doc.createElement("arg")
    arg.setAttribute("name", "robot_number")
    arg.setAttribute("value", "$(eval arg('robot_number') - 1)")
    judge.appendChild(arg)
    root.appendChild(judge)

    # append `root` to document
    doc.appendChild(root)
    with open(root_path + r_path + name + '.launch.xml', 'w') as f:
        doc.writexml(f, addindent="", newl="\n")

writeEnvMainXml("main", 'sim_env/launch/')
writeRobotsXml("start_robots", 'sim_env/launch/include/robots/')