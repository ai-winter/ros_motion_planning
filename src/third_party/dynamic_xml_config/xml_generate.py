#!/usr/bin/python
#-*- coding: utf-8 -*-

''' 
******************************************************************************************
*  Copyright (C) 2022 Yang Haodong, All Rights Reserved                                  *
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

# 获得功能包根目录(src级)
tempPath = os.path.split(os.path.realpath(__file__))[0]
rootPath = tempPath + "/../../"

# @breif:解析yaml用户配置文件
# @param[in]: 文件相对功能包根目录的路径
# @retval: 用户配置树
def yamlParser(rPath):
    # 要解析的yaml文件名
    fileName = sys.argv[1]
    with open(rootPath + rPath + fileName + '.yaml', 'r') as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
    return data

# 解析用户配置数据
userConfig = yamlParser('user_config/')
robotsNum = len(userConfig["robots_config"])

# @breif: 生成环境包入口文件
# @param[in]: name  入口文件名
# @param[in]: rPath 文件相对功能包根目录的路径
# @retval: None
def writeEnvMainXml(name, rPath):
    # 创建launch文件文档
    doc = minidom.Document()
    root = doc.createElement("launch")
    # 创建主文件节点
    includeMain = doc.createElement("include")
    includeMain.setAttribute("file", "$(find sim_env)/launch/config.launch")
    argWorld = doc.createElement("arg")
    argWorld.setAttribute("name", "world")
    argWorld.setAttribute("value", userConfig["world"])
    argMap = doc.createElement("arg")
    argMap.setAttribute("name", "map")
    argMap.setAttribute("value", userConfig["map"])
    argNum = doc.createElement("arg")
    argNum.setAttribute("name", "robot_number")
    argNum.setAttribute("value", str(robotsNum))
    argRviz = doc.createElement("arg")
    argRviz.setAttribute("name", "rviz_file")
    argRviz.setAttribute("value", userConfig["rviz_file"])
    includeMain.appendChild(argWorld)
    includeMain.appendChild(argMap)
    includeMain.appendChild(argNum)
    includeMain.appendChild(argRviz)
    # 添加根节点到文档
    root.appendChild(includeMain)
    doc.appendChild(root)
    with open(rootPath + rPath + name + '.launch', 'w') as f:
        doc.writexml(f, addindent="", newl="\n")

# @breif: 生成机器人启动文件
# @param[in]: name  入口配置文件名
# @param[in]: rPath 文件相对功能包根目录的路径
# @retval: None
def writeRobotsXml(name, rPath):
    # 创建launch文件文档
    doc = minidom.Document()
    root = doc.createElement("launch")
    # 创建参数列表
    # 配置机器人数量
    argNum = doc.createElement("arg")
    argNum.setAttribute("name", "robot_number")
    argNum.setAttribute("default", str(robotsNum))
    root.appendChild(argNum)
    # 配置机器人属性
    for i in range(robotsNum):
        argType = doc.createElement("arg")
        argGlobalPlanner = doc.createElement("arg")
        argLocalPlanner = doc.createElement("arg")
        argXPos = doc.createElement("arg")
        argYPos = doc.createElement("arg")
        argZPos = doc.createElement("arg")
        argYaw = doc.createElement("arg")
        argType.setAttribute("name", "robot" + str(i + 1) + "_type")
        argType.setAttribute("value", userConfig["robots_config"][i]["robot" + str(i + 1) + "_type"])
        argGlobalPlanner.setAttribute("name", "robot" + str(i + 1) + "_global_planner")
        argGlobalPlanner.setAttribute("value", userConfig["robots_config"][i]["robot" + str(i + 1) + "_global_planner"])
        argLocalPlanner.setAttribute("name", "robot" + str(i + 1) + "_local_planner")
        argLocalPlanner.setAttribute("value", userConfig["robots_config"][i]["robot" + str(i + 1) + "_local_planner"])
        argXPos.setAttribute("name", "robot" + str(i + 1) + "_x_pos")
        argXPos.setAttribute("value", userConfig["robots_config"][i]["robot" + str(i + 1) + "_x_pos"])
        argYPos.setAttribute("name", "robot" + str(i + 1) + "_y_pos")
        argYPos.setAttribute("value", userConfig["robots_config"][i]["robot" + str(i + 1) + "_y_pos"])
        argZPos.setAttribute("name", "robot" + str(i + 1) + "_z_pos")
        argZPos.setAttribute("value", userConfig["robots_config"][i]["robot" + str(i + 1) + "_z_pos"])
        argYaw.setAttribute("name", "robot" + str(i + 1) + "_yaw")
        argYaw.setAttribute("value", userConfig["robots_config"][i]["robot" + str(i + 1) + "_yaw"])
        root.appendChild(argType)
        root.appendChild(argGlobalPlanner)
        root.appendChild(argLocalPlanner)
        root.appendChild(argXPos)
        root.appendChild(argYPos)
        root.appendChild(argZPos)
        root.appendChild(argYaw)
    # 创建递归启动节点
    start = doc.createElement("include")
    start.setAttribute("file", "$(find sim_env)/launch/app/environment_single.launch.xml")
    argType = doc.createElement("arg")
    argGlobalPlanner = doc.createElement("arg")
    argLocalPlanner = doc.createElement("arg")
    argNs = doc.createElement("arg")
    argStartNs = doc.createElement("arg")
    argXPos = doc.createElement("arg")
    argYPos = doc.createElement("arg")
    argZPos = doc.createElement("arg")
    argYaw = doc.createElement("arg")
    argType.setAttribute("name", "robot")
    argType.setAttribute("value", "$(eval arg('robot' + str(arg('robot_number')) + '_type'))")
    argGlobalPlanner.setAttribute("name", "global_planner")
    argGlobalPlanner.setAttribute("value", "$(eval arg('robot' + str(arg('robot_number')) + '_global_planner'))")
    argLocalPlanner.setAttribute("name", "local_planner")
    argLocalPlanner.setAttribute("value", "$(eval arg('robot' + str(arg('robot_number')) + '_local_planner'))")
    argNs.setAttribute("name", "robot_namespace")
    argNs.setAttribute("value", "robot$(arg robot_number)")
    argStartNs.setAttribute("name", "start_ns")
    argXPos.setAttribute("name", "robot_x")
    argXPos.setAttribute("value", "$(eval arg('robot' + str(arg('robot_number')) + '_x_pos'))")
    argYPos.setAttribute("name", "robot_y")
    argYPos.setAttribute("value", "$(eval arg('robot' + str(arg('robot_number')) + '_y_pos'))")
    argZPos.setAttribute("name", "robot_z")
    argZPos.setAttribute("value", "$(eval arg('robot' + str(arg('robot_number')) + '_z_pos'))")
    argYaw.setAttribute("name", "robot_yaw")
    argYaw.setAttribute("value", "$(eval arg('robot' + str(arg('robot_number')) + '_yaw'))")
    if robotsNum > 1:
        argStartNs.setAttribute("value", "true")
    else:
        argStartNs.setAttribute("value", "false")
    start.appendChild(argType)
    start.appendChild(argGlobalPlanner)
    start.appendChild(argLocalPlanner)
    start.appendChild(argNs)
    start.appendChild(argStartNs)
    start.appendChild(argXPos)
    start.appendChild(argYPos)
    start.appendChild(argZPos)
    start.appendChild(argYaw)
    root.appendChild(start)
    # 创建判断节点
    judge = doc.createElement("include")
    judge.setAttribute("file", "$(find sim_env)/launch/include/robots/start_robots.launch.xml")
    judge.setAttribute("if", "$(eval arg('robot_number') > 1)")
    arg = doc.createElement("arg")
    arg.setAttribute("name", "robot_number")
    arg.setAttribute("value", "$(eval arg('robot_number') - 1)")
    judge.appendChild(arg)
    root.appendChild(judge)

    # 添加根节点到文档
    doc.appendChild(root)
    with open(rootPath + rPath + name + '.launch.xml', 'w') as f:
        doc.writexml(f, addindent="", newl="\n")

writeEnvMainXml("main", 'sim_env/launch/')
writeRobotsXml("start_robots", 'sim_env/launch/include/robots/')