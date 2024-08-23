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
import xml.etree.ElementTree as ET

import rospy
import tf
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Point, Pose, Quaternion

from .xml_generate import XMLGenerator

class ObstacleGenerator(XMLGenerator):
    def __init__(self) -> None:
        super().__init__()
        if "plugins" in self.user_cfg.keys() and "obstacles" in self.user_cfg["plugins"] and self.user_cfg["plugins"]["obstacles"]:
            self.obs_cfg = ObstacleGenerator.yamlParser(self.root_path + "user_config/" + self.user_cfg["plugins"]["obstacles"])
        else:
            self.obs_cfg = None

        self.box_obs, self.cylinder_obs, self.sphere_obs = [], [], []

    def plugin(self):
        """
        Implement of obstacles application.
        """
        app_register = []
        if not self.obs_cfg is None:
            """app register"""
            # obstacles generation
            obs_gen = ObstacleGenerator.createElement(
                "node",
                props={
                    "pkg": "dynamic_xml_config",
                    "type": "obstacles_generate_ros.py",
                    "name": "obstacles_generate",
                    "args": "user_config.yaml",
                    "output": "screen",
                },
            )
            app_register.append(obs_gen)

        return app_register

    def spawn(self) -> None:
        """
        ROS service for spawning static obstacles in Gazebo world.
        """
        rospy.wait_for_service("gazebo/spawn_urdf_model")
        proxy = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)

        for obs in self.obs_cfg["obstacles"]:
            pose = [float(i) for i in obs["pose"].split()]
            x, y, z, r, p, yaw = pose
            orient = tf.transformations.quaternion_from_euler(r, p, yaw)
            orient = Quaternion(orient[0], orient[1], orient[2], orient[3])
            params = obs["props"]
            params["c"] = obs["color"]
            urdf = self.constructURDF(obs["type"], **params)

            if obs["type"] == "BOX":
                z = z if z else obs["props"]["h"] / 2
                pose = Pose(Point(x=x, y=y, z=z), orient)
                proxy(obs["type"] + str(len(self.box_obs)), urdf, "", pose, "world")
                self.box_obs.append(obs)
            elif obs["type"] == "CYLINDER":
                z = z if z else obs["props"]["h"] / 2
                pose = Pose(Point(x=x, y=y, z=z), orient)
                proxy(obs["type"] + str(len(self.cylinder_obs)), urdf, "", pose, "world")
                self.cylinder_obs.append(obs)
            elif obs["type"] == "SPHERE":
                z = z if z else obs["props"]["r"]
                pose = Pose(Point(x=x, y=y, z=z), orient)
                proxy(obs["type"] + str(len(self.sphere_obs)), urdf, "", pose, "world")
                self.sphere_obs.append(obs)

    def constructURDF(self, model_type: str, **kwargs) -> str:
        """
        Construct URDF of specific type of obstacle.

        Parameters
        ----------
        model_type: str
            specific type of obstacle. Optional is `BOX`, `CYLINDER` and `SPHERE`
        kwargs: dict
            parameters of obstacle, such as mass, color, height, etc.

        Return
        ----------
        urdf_str: str
            URDF of obstacle
        """
        # parameters checking
        if model_type.upper() == "BOX":
            assert (
                "m" in kwargs and "w" in kwargs and "d" in kwargs and "h" in kwargs and "c" in kwargs
            ), "Parameters of {} are `m`, `w`, `d`, `h`, `c` which mean mass, \
                    width, depth, height and color, ".format(
                model_type
            )
            ixx = (kwargs["m"] / 12.0) * (pow(kwargs["d"], 2) + pow(kwargs["h"], 2))
            iyy = (kwargs["m"] / 12.0) * (pow(kwargs["w"], 2) + pow(kwargs["h"], 2))
            izz = (kwargs["m"] / 12.0) * (pow(kwargs["w"], 2) + pow(kwargs["d"], 2))
            geometry = ObstacleGenerator.createElement("geometry")
            geometry.append(
                ObstacleGenerator.createElement(model_type.lower(), props={"size": "{:f} {:f} {:f}".format(kwargs["w"], kwargs["d"], kwargs["h"])})
            )
        elif model_type.upper() == "CYLINDER":
            assert (
                "m" in kwargs and "r" in kwargs and "h" in kwargs and "c" in kwargs
            ), "Parameters of {} are `m`, `r`, `h`, `c` which mean mass, \
                    radius, height and color, ".format(
                model_type
            )
            ixx = (kwargs["m"] / 12.0) * (3 * pow(kwargs["r"], 2) + pow(kwargs["h"], 2))
            iyy = (kwargs["m"] / 12.0) * (3 * pow(kwargs["r"], 2) + pow(kwargs["h"], 2))
            izz = (kwargs["m"] * pow(kwargs["r"], 2)) / 2.0
            geometry = ObstacleGenerator.createElement("geometry")
            geometry.append(ObstacleGenerator.createElement(model_type.lower(), props={"length": str(kwargs["h"]), "radius": str(kwargs["r"])}))
        elif model_type.upper() == "SPHERE":
            assert (
                "m" in kwargs and "r" in kwargs and "c" in kwargs
            ), "Parameters of {} are `m`, `r`, `h`, `c` which mean mass, \
                    radius and color, ".format(
                model_type
            )
            ixx = (2 * kwargs["m"] * pow(kwargs["r"], 2)) / 5.0
            iyy = (2 * kwargs["m"] * pow(kwargs["r"], 2)) / 5.0
            izz = (2 * kwargs["m"] * pow(kwargs["r"], 2)) / 5.0
            geometry = ObstacleGenerator.createElement("geometry")
            geometry.append(ObstacleGenerator.createElement("sphere", props={"radius": str(kwargs["r"])}))
        else:
            raise NotImplementedError

        # URDF generation dynamically
        static_obs = ObstacleGenerator.createElement("robot", props={"name": model_type.lower()})
        link = ObstacleGenerator.createElement("link", props={"name": model_type.lower() + "_link"})

        inertial = ObstacleGenerator.createElement("inertial")
        inertial.append(ObstacleGenerator.createElement("origin", props={"xyz": "0 0 0", "rpy": "0 0 0"}))
        inertial.append(ObstacleGenerator.createElement("mass", props={"value": str(kwargs["m"])}))
        inertial.append(
            ObstacleGenerator.createElement(
                "inertia", props={"ixx": str(ixx), "ixy": "0.0", "ixz": "0.0", "iyy": str(iyy), "iyz": "0.0", "izz": str(izz)}
            )
        )

        collision = ObstacleGenerator.createElement("collision")
        collision.append(ObstacleGenerator.createElement("origin", props={"xyz": "0 0 0", "rpy": "0 0 0"}))

        collision.append(geometry)

        visual = ObstacleGenerator.createElement("visual")
        visual.append(ObstacleGenerator.createElement("origin", props={"xyz": "0 0 0", "rpy": "0 0 0"}))
        visual.append(geometry)
        r, g, b, a = ObstacleGenerator.color(kwargs["c"])
        visual.append(ObstacleGenerator.createElement("color", props={"rgba": "{:f} {:f} {:f} {:f}".format(r, g, b, a)}))

        link.append(inertial)
        link.append(collision)
        link.append(visual)

        gazebo = ObstacleGenerator.createElement("gazebo", props={"reference": model_type.lower() + "_link"})
        gazebo.append(ObstacleGenerator.createElement("kp", text=str(100000.0)))
        gazebo.append(ObstacleGenerator.createElement("kd", text=str(100000.0)))
        gazebo.append(ObstacleGenerator.createElement("mu1", text=str(10.0)))
        gazebo.append(ObstacleGenerator.createElement("mu2", text=str(10.0)))
        gazebo.append(ObstacleGenerator.createElement("material", text="Gazebo/" + kwargs["c"]))

        static_obs.append(link)
        static_obs.append(gazebo)

        ObstacleGenerator.indent(static_obs)

        return ET.tostring(static_obs).decode()

    @staticmethod
    def color(color_name):
        if color_name == "Blue":
            return (0, 0, 0.8, 1)
        elif color_name == "Red":
            return (0.8, 0, 0, 1)
        elif color_name == "Green":
            return (0, 0.8, 0, 1)
        elif color_name == "Grey":
            return (0.75, 0.75, 0.75, 1)
        elif color_name == "White":
            return (1.0, 1.0, 1.0, 1)
        elif color_name == "Black":
            return (0, 0, 0, 1)
        else:
            return (0.75, 0.75, 0.75, 1)
