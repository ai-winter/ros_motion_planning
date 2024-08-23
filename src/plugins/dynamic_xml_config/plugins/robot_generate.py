#!/usr/bin/python
# -*- coding: utf-8 -*-

""" 
******************************************************************************************
*  Copyright (C) 2023 Yang Haodong, All Rights Reserved                                  *
*                                                                                        *
*  @brief    generate launch file dynamicly based on user configure.                     *
*  @author   Haodong Yang                                                                *
*  @version  1.0.2                                                                       *
*  @date     2023.07.19                                                                  *
*  @license  GNU General Public License (GPL)                                            *
******************************************************************************************
"""
import xml.etree.ElementTree as ET

from .xml_generate import XMLGenerator


class RobotGenerator(XMLGenerator):
    def __init__(self) -> None:
        super().__init__()

    def __str__(self) -> str:
        return "Robots Generator"

    def plugin(self):
        """
        Implement of robots starting application.
        """
        app_register = []
        self.writeRobotsXml(self.root_path + "sim_env/launch/include/robots/start_robots.launch.xml")
        return app_register

    def writeRobotsXml(self, path):
        """
        Create configure file to start robots dynamically.

        Parameters
        ----------
        path: str
            the path of file(.launch.xml) to write.
        """

        def getRobotArg(name: str, index: int = -1):
            if index == -1:
                e = RobotGenerator.createElement("arg", props={"name": name, "value": "$(eval arg('robot' + str(arg('agent_id')) + '_" + name + "'))"})
            else:
                e = RobotGenerator.createElement(
                    "arg",
                    props={
                        "name": "robot" + str(index + 1) + "_" + name,
                        "value": self.user_cfg["robots_config"][index]["robot" + str(index + 1) + "_" + name],
                    },
                )
            return e

        # root
        launch = RobotGenerator.createElement("launch")

        # setting the number of robots
        if "robots_config" in self.user_cfg.keys():
            robots_num = len(self.user_cfg["robots_config"])
        else:
            robots_num = 0
            raise ValueError("There is no robot!")

        launch.append(RobotGenerator.createElement("arg", props={"name": "agent_number", "default": str(robots_num)}))
        launch.append(RobotGenerator.createElement("arg", props={"name": "agent_id", "default": str(robots_num)}))

        # setting the parameters of robots
        for i in range(robots_num):
            # robotic type
            launch.append(getRobotArg("type", i))
            # global planner
            launch.append(getRobotArg("global_planner", i))
            # local planner
            launch.append(getRobotArg("local_planner", i))
            # robotic pose
            launch.append(getRobotArg("x_pos", i))
            launch.append(getRobotArg("y_pos", i))
            launch.append(getRobotArg("z_pos", i))
            launch.append(getRobotArg("yaw", i))

        # create starting node
        include = RobotGenerator.createElement("include", props={"file": "$(find sim_env)/launch/app/environment_single.launch.xml"})
        include.append(RobotGenerator.createElement("arg", props={"name": "agent_number", "value": "$(arg agent_number)"}))
        include.append(RobotGenerator.createElement("arg", props={"name": "agent_id", "value": "$(arg agent_id)"}))
        include.append(RobotGenerator.createElement("arg", props={"name": "robot", "value": "$(eval arg('robot' + str(arg('agent_id')) + '_type'))"}))
        # planner
        include.append(getRobotArg("global_planner"))
        include.append(getRobotArg("local_planner"))
        # namespace
        include.append(RobotGenerator.createElement("arg", props={"name": "robot_namespace", "value": "robot$(arg agent_id)"}))
        if robots_num > 1:
            include.append(RobotGenerator.createElement("arg", props={"name": "start_ns", "value": "true"}))
        else:
            include.append(RobotGenerator.createElement("arg", props={"name": "start_ns", "value": "false"}))
        # pose
        include.append(RobotGenerator.createElement("arg", props={"name": "robot_x", "value": "$(eval arg('robot' + str(arg('agent_id')) + '_x_pos'))"}))
        include.append(RobotGenerator.createElement("arg", props={"name": "robot_y", "value": "$(eval arg('robot' + str(arg('agent_id')) + '_y_pos'))"}))
        include.append(RobotGenerator.createElement("arg", props={"name": "robot_z", "value": "$(eval arg('robot' + str(arg('agent_id')) + '_z_pos'))"}))
        include.append(RobotGenerator.createElement("arg", props={"name": "robot_yaw", "value": "$(eval arg('robot' + str(arg('agent_id')) + '_yaw'))"}))

        # recursive start
        cycle = RobotGenerator.createElement("include", props={"file": "$(find sim_env)/launch/include/robots/start_robots.launch.xml", "if": "$(eval arg('agent_id') > 1)"})
        cycle.append(RobotGenerator.createElement("arg", props={"name": "agent_id", "value": "$(eval arg('agent_id') - 1)"}))

        launch.append(include)
        launch.append(cycle)
        RobotGenerator.indent(launch)

        with open(path, "wb+") as f:
            ET.ElementTree(launch).write(f, encoding="utf-8", xml_declaration=True)
