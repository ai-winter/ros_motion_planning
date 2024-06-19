#!/usr/bin/python
# -*- coding: utf-8 -*-

""" 
******************************************************************************************
*  Copyright (C) 2023 Yang Haodong, All Rights Reserved                                  *
*                                                                                        *
*  @brief    generate launch file dynamicly based on user configure.                     *
*  @author   Haodong Yang                                                                *
*  @version  1.0.2                                                                       *
*  @date     2023.03.15                                                                  *
*  @license  GNU General Public License (GPL)                                            *
******************************************************************************************
"""
import rospy
import sys, os
sys.path.append(os.path.abspath(os.path.join(__file__, "../../")))

from plugins import ObstacleGenerator

if __name__ == "__main__":
    obstacles = ObstacleGenerator()

    rospy.init_node("spawn_obstacles")
    obstacles.spawn()