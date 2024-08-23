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
import yaml


class RVizConfig:
    def __init__(self, rviz_name, path, base_file=None):
        # Rviz visualization manager
        self.VIZ_MAN = "Visualization Manager"
        # Rviz standard tools
        self.STD_TOOLS = ["rviz/Interact", "rviz/MoveCamera", "rviz/Select", "rviz/FocusCamera", "rviz/Measure", "rviz/PublishPoint"]
        # Rviz standard panels
        self.STD_PANELS = ["rviz/Displays", "rviz/Tool Properties"]
        # Rviz file name
        self.file_name = rviz_name
        # Rviz cache
        self.cache_path = path
        # Rviz pre-configure
        self.data = yaml.load(base_file) if base_file else {}

    def __repr__(self):
        return yaml.dump(self.data, default_flow_style=False)

    # @breif: set Rviz configure path
    #
    # @param[in]: path  Rviz configure path
    # @return: None
    def setFileCache(self, path):
        self.cache_path = path

    # @breif: extract all configure data of `rviz Visualization Manager`
    #
    # @param[in]: None
    # @return: all configure data of `rviz Visualization Manager`
    def getVisualization(self):
        if self.VIZ_MAN not in self.data:
            self.data[self.VIZ_MAN] = {}
        return self.data[self.VIZ_MAN]

    # @breif: set display format
    # @param[in]: displays  display format
    # @return: True if successful else False
    def setDisplays(self, displays):
        if "Displays" not in self.getVisualization():
            self.data[self.VIZ_MAN]["Displays"] = displays
            print("Displays have been set successfully!")
            return True
        print("Displays setting failed! please call displays delete first!")
        return False

    # @breif: clear display
    # @param[in]: None
    # @return: True if successful else False
    def delDisplays(self):
        if "Displays" in self.getVisualization():
            del self.data[self.VIZ_MAN]["Displays"]
            print("Displays have been deleted successfully!")
            return True
        print("Displays delete failed! please call displays setting first!")
        return False

    # @breif: set Rviz standard tools
    # @param[in]: None
    # @return: True if successful else False
    def setStdTools(self):
        v = self.getVisualization()
        if "Tools" not in v:
            v["Tools"] = []
            for tool in self.STD_TOOLS:
                v["Tools"].append({"Class": tool})
            print("standard tools have been set successfully!")
            return True
        print("standard tools setting failed! please call standard tools delete first!")
        return False

    # @breif: clear Rviz standard tools
    # @param[in]: None
    # @return: True if successful else False
    def delStdTools(self):
        if "Tools" in self.getVisualization():
            del self.data[self.VIZ_MAN]["Tools"]
            print("standard tools have been deleted successfully!")
            return True
        print("standard tools delete failed! please call standard tools setting first!")
        return False

    # @breif: set Rviz standard panels
    # @param[in]: None
    # @return: True if successful else False
    def setStdPanels(self):
        if "Panels" not in self.data:
            self.data["Panels"] = []
            for tool in self.STD_PANELS:
                self.data["Panels"].append({"Class": tool, "Name": tool.split("/")[-1]})
                print("standard panels have been set successfully!")
            return True
        print("standard panels setting failed! please call standard panels delete first!")
        return False

    # @breif: clear Rviz standard panels
    # @param[in]: None
    # @return: True if successful else False
    def delStdPanels(self):
        if "Panels" in self.data:
            del self.data["Panels"]
            print("standard panels have been deleted successfully!")
            return True
        print("standard panels delete failed! please call standard panels setting first!")
        return False

    # @breif: set base coordinate
    # @param[in]: frame base coordinate, default is `map`
    # @return: None
    def setFixedFrame(self, frame="map"):
        v = self.getVisualization()
        if "Global Options" not in v:
            v["Global Options"] = {"Fixed Frame": frame}
        else:
            v["Global Options"]["Fixed Frame"] = frame

    # @breif: set the topics of Rviz standard tools
    # @param[in]: name  Rviz standard tools classes
    # @param[in]: topic  the topic of Rviz standard tools relatively
    # @return: None
    def setToolTopic(self, name, topic):
        for m in self.data[self.VIZ_MAN]["Tools"]:
            if m.get("Class", "") == name:
                m["Topic"] = topic

    # @breif: append navigation tools and their topics
    # @param[in]: None
    # @return: None
    def appendToolGoal(self, topic):
        v = self.getVisualization()
        if "Tools" not in v:
            v["Tools"] = []
        v["Tools"].append({"Class": "rviz/SetGoal", "Topic": topic})

    # @breif: append initial Pose and its topic
    # @param[in]: None
    # @return: None
    def appendToolInit(self, topic):
        v = self.getVisualization()
        if "Tools" not in v:
            v["Tools"] = []
        v["Tools"].append({"Class": "rviz/SetInitialPose", "Topic": topic})

    # @breif: run Rviz and apply user configure
    # @param[in]: None
    # @return: None
    def run(self):
        with open(self.cache_path + self.file_name + ".rviz", "w") as fp:
            fp.write(str(self))
        import subprocess

        subprocess.call(["rosrun", "rviz", "rviz", "-d", self.cache_path + self.file_name + ".rviz"])
