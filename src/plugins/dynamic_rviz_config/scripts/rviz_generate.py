#!/usr/bin/python
# -*- coding: utf-8 -*-

""" 
******************************************************************************************
*  Copyright (C) 2023 Yang Haodong, All Rights Reserved                                  *
*                                                                                        *
*  @brief    rviz basic pannel configure class.                                          *
*  @author   Haodong Yang                                                                *
*  @version  1.0.1                                                                       *
*  @date     2022.07.06                                                                  *
*  @license  GNU General Public License (GPL)                                            *
******************************************************************************************
"""
import os
import sys

dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, dir_path)

from config import RVizConfig
from displays import Displays

# the number of robots
robot_num = int(sys.argv[1])

r = RVizConfig("cache", os.path.split(os.path.realpath(__file__))[0] + "/../../../sim_env/rviz/")
# set Rviz standard tools
r.setStdTools()
# visualize Rviz models
d = Displays()
# load map and grid
d.addMap()
d.addGrid()
# set transform
d.addTf()

# single robot
if robot_num == 1:
    # append robots' model
    d.addModel()
    # append camera image
    d.addImage()
    # append laser
    d.addLaserscan(topic="/scan", name="LaserScan", color=(255, 0, 0), size=0.03, alpha=1)
    # append `initialpose` setting panel
    r.appendToolInit("/initialpose")
    # append `move_base_simple/goal` setting panel
    r.appendToolGoal("/move_base_simple/goal")
else:
    # multi-robot
    for i in range(robot_num):
        meta_robot = Displays()
        # append robots' model
        meta_robot.addModel(parameter="robot" + str(i + 1) + "/robot_description", tf_prefix="robot" + str(i + 1))
        # append camera image
        meta_robot.addImage(topic="/robot" + str(i + 1) + "/camera/rgb/image_raw", name="Image" + str(i + 1))
        # append laser
        meta_robot.addLaserscan(topic="/robot" + str(i + 1) + "/scan", name="LaserScan" + str(i + 1), color=(255, 0, 0), size=0.03, alpha=1)
        # append path
        meta_robot.addPath(topic=None, name="Path", color=(32, 74, 135))
        d.addGroup("Robot" + str(i + 1), meta_robot)
        # append `initialpose` setting panel
        r.appendToolInit("/robot" + str(i + 1) + "/initialpose")
        # ppend `move_base_simple/goal` setting panel
        r.appendToolGoal("/robot" + str(i + 1) + "/move_base_simple/goal")

# 设置Rviz标准面板
r.setStdPanels()
# 设置Rviz模型显示
r.setDisplays(d)
# 设置Rviz基坐标系
r.setFixedFrame()
# 启动Rviz
r.run()
