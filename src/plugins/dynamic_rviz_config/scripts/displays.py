#!/usr/bin/python
# -*- coding: utf-8 -*-

""" 
******************************************************************************************
*  Copyright (C) 2023 Yang Haodong, All Rights Reserved                                  *
*                                                                                        *
*  @brief    rviz display pannel configure class.                                        *
*  @author   Haodong Yang                                                                *
*  @version  1.0.1                                                                       *
*  @date     2022.07.06                                                                  *
*  @license  GNU General Public License (GPL)                                            *
******************************************************************************************
"""


class Displays(list):
    def __init__(self):
        None

    def addDisplay(self, name, class_name, topic=None, color=None, fields={}, enabled=True):
        d = {"Name": name, "Class": class_name, "Enabled": enabled}
        if topic:
            d["Topic"] = topic
        if color:
            d["Color"] = "%d; %d; %d" % color
        d.update(fields)
        self.append(d)

    def addMap(self, topic="/map", name="Map", alpha=None, scheme=None):
        fields = {}
        if alpha:
            fields["Alpha"] = alpha
        if scheme:
            fields["Color Scheme"] = scheme
        self.addDisplay(name, "rviz/Map", topic, fields=fields)

    def addGrid(self):
        self.addDisplay("Grid", "rviz/Grid")

    def addTf(self, scale=None):
        fields = {}
        if scale:
            fields["Marker Scale"] = scale
        self.addDisplay("TF", "rviz/TF", fields=fields, enabled=False)

    def addGroup(self, name, displays):
        self.addDisplay(name, "rviz/Group", fields={"Displays": displays})

    def addModel(self, parameter="robot_description", tf_prefix=None):
        fields = {"Robot Description": parameter}
        if tf_prefix:
            fields["TF Prefix"] = tf_prefix
        self.addDisplay("RobotModel", "rviz/RobotModel", fields=fields)

    def addLaserscan(self, topic="/base_scan", name=None, color=(46, 255, 0), size=0.1, alpha=None):
        if name is None:
            name = topic
        fields = {"Size (m)": size, "Style": "Spheres", "Color Transformer": "FlatColor"}
        if alpha:
            fields["Alpha"] = alpha
        self.addDisplay(name, "rviz/LaserScan", topic, color, fields)

    def addPoseArray(self, topic="/particlecloud", color=None):
        self.addDisplay("AMCL Cloud", "rviz/PoseArray", topic, color)

    def addFootprint(self, topic, color=(0, 170, 255)):
        self.addDisplay("Robot Footprint", "rviz/Polygon", topic, color)

    def addPath(self, topic, name, color=None):
        self.addDisplay(name, "rviz/Path", topic, color)

    def addPose(self, topic, name="Current Goal", color=None, arrow_shape=None):
        fields = {}
        if arrow_shape:
            fields["Head Length"] = arrow_shape[0]
            fields["Head Radius"] = arrow_shape[1]
            fields["Shaft Length"] = arrow_shape[2]
            fields["Shaft Radius"] = arrow_shape[3]
            fields["Shape"] = "Arrow"
        self.addDisplay(name, "rviz/Pose", topic, color, fields)

    def addImage(self, topic="/camera/rgb/image_raw", name="Image"):
        self.addDisplay(name, "rviz/Image", topic, enabled=False)


import yaml


def display_representer(dumper, data):
    return dumper.represent_sequence("tag:yaml.org,2002:seq", list(data))


yaml.add_representer(Displays, display_representer)
