#!/usr/bin/python
#-*- coding: utf-8 -*-

''' 
******************************************************************************************
*  Copyright (C) 2022 Yang Haodong, All Rights Reserved                                  *
*                                                                                        *
*  @brief    rviz basic pannel configure class.                                          *
*  @author   Haodong Yang                                                                *
*  @version  1.0.1                                                                       *
*  @date     2022.07.06                                                                  *
*  @license  GNU General Public License (GPL)                                            *
******************************************************************************************
'''
# 添加环境变量 
import sys, os
dirPath = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, dirPath)

# 导入自定义文件
from config import RVizConfig
from displays import Displays

# 机器人数量
robotNum = int(sys.argv[1])

r = RVizConfig("cache", os.path.split(os.path.realpath(__file__))[0] + "/../../../sim_env/rviz/")
# 设置Rviz标准工具集
r.setStdTools()
# Rviz模型可视化
d = Displays()
# 加载地图
d.addMap()
d.addGrid()
# 增加坐标变换
d.addTf()

# 单个机器人
if robotNum == 1:
    # 添加机器人模型
    d.addModel()
    # 添加摄像头
    d.addImage()
    # 添加激光雷达
    d.addLaserscan(topic="/scan", name="LaserScan", color=(255, 0, 0), size=0.03, alpha=1)
    # 添加初始位置设置
    r.appendToolInit('/initialpose')
    # 添加目标导航点设置
    r.appendToolGoal('/move_base_simple/goal')
else:
    # 生成多个机器人可视化组
    for i in range(robotNum):
        metaRobot = Displays()
        # 添加机器人模型
        metaRobot.addModel(parameter="robot" + str(i + 1) + "/robot_description", tf_prefix="robot" + str(i + 1))
        # 添加摄像头
        metaRobot.addImage(topic="/robot" + str(i + 1) + "/camera/rgb/image_raw", name="Image" + str(i + 1))
        # 添加激光雷达
        metaRobot.addLaserscan(topic="/robot" + str(i + 1) + "/scan", name="LaserScan" + str(i + 1), color=(255, 0, 0), size=0.03, alpha=1)
        # 添加路径
        metaRobot.addPath(topic=None, name="Path", color=(32, 74, 135))
        d.addGroup("Robot" + str(i + 1), metaRobot)
        # 添加初始位置设置
        r.appendToolInit('/robot' + str(i + 1) + '/initialpose')
        # 添加目标导航点设置
        r.appendToolGoal('/robot' + str(i + 1) +'/move_base_simple/goal')

# 设置Rviz标准面板
r.setStdPanels()
# 设置Rviz模型显示
r.setDisplays(d)
# 设置Rviz基坐标系
r.setFixedFrame()
# 启动Rviz
r.run()
