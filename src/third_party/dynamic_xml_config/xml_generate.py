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
import yaml
import xml.etree.ElementTree as ET
import sys, os


class XMLGenerator(object):
    def __init__(self) -> None:
        # get the root path of package(src layer)
        self.root_path = os.path.split(os.path.realpath(__file__))[0] + "/../../"
        # user configure
        self.user_cfg = XMLGenerator.yamlParser(self.root_path + "user_config/" + sys.argv[1])

    def writeEnvMainXml(self, path):
        """
        Create configure file of package `sim_env` dynamically.

        Parameters
        ----------
        path: str
            the path of file(.launch.xml) to write.
        """
        # root
        launch = ET.Element("launch")

        # include
        include = XMLGenerator.createElement("include", props={"file": "$(find sim_env)/launch/config.launch"})

        if not "pedestrians" in self.user_cfg or not self.user_cfg["pedestrians"] or not self.user_cfg["world"]:
            world_cfg = {"name": "world", "value": self.user_cfg["world"]}
        else:
            world_cfg = {"name": "world", "value": self.user_cfg["world"] + "_with_pedestrians"}
        include.append(XMLGenerator.createElement("arg", props=world_cfg))
        include.append(XMLGenerator.createElement("arg", props={"name": "map", "value": self.user_cfg["map"]}))
        include.append(XMLGenerator.createElement("arg", props={"name": "robot_number", "value": str(len(self.user_cfg["robots_config"]))}))
        include.append(XMLGenerator.createElement("arg", props={"name": "rviz_file", "value": self.user_cfg["rviz_file"]}))

        launch.append(include)
        XMLGenerator.indent(launch)

        with open(path, "wb+") as f:
            ET.ElementTree(launch).write(f, encoding="utf-8", xml_declaration=True)

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
                e = XMLGenerator.createElement(
                    "arg", props={"name": name, "value": "$(eval arg('robot' + str(arg('robot_number')) + '_" + name + "'))"}
                )
            else:
                e = XMLGenerator.createElement(
                    "arg",
                    props={
                        "name": "robot" + str(index + 1) + "_" + name,
                        "value": self.user_cfg["robots_config"][index]["robot" + str(index + 1) + "_" + name],
                    },
                )
            return e

        # root
        launch = ET.Element("launch")

        # setting the number of robots
        robots_num = len(self.user_cfg["robots_config"])
        launch.append(XMLGenerator.createElement("arg", props={"name": "robot_number", "default": str(robots_num)}))

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
        include = XMLGenerator.createElement("include", props={"file": "$(find sim_env)/launch/app/environment_single.launch.xml"})
        include.append(
            XMLGenerator.createElement("arg", props={"name": "robot", "value": "$(eval arg('robot' + str(arg('robot_number')) + '_type'))"})
        )
        # planner
        include.append(getRobotArg("global_planner"))
        include.append(getRobotArg("local_planner"))
        # namespace
        include.append(XMLGenerator.createElement("arg", props={"name": "robot_namespace", "value": "robot$(arg robot_number)"}))
        if robots_num > 1:
            include.append(XMLGenerator.createElement("arg", props={"name": "start_ns", "value": "true"}))
        else:
            include.append(XMLGenerator.createElement("arg", props={"name": "start_ns", "value": "false"}))
        # pose
        include.append(
            XMLGenerator.createElement("arg", props={"name": "robot_x", "value": "$(eval arg('robot' + str(arg('robot_number')) + '_x_pos'))"})
        )
        include.append(
            XMLGenerator.createElement("arg", props={"name": "robot_y", "value": "$(eval arg('robot' + str(arg('robot_number')) + '_y_pos'))"})
        )
        include.append(
            XMLGenerator.createElement("arg", props={"name": "robot_z", "value": "$(eval arg('robot' + str(arg('robot_number')) + '_z_pos'))"})
        )
        include.append(
            XMLGenerator.createElement("arg", props={"name": "robot_yaw", "value": "$(eval arg('robot' + str(arg('robot_number')) + '_yaw'))"})
        )

        # recursive start
        cycle = XMLGenerator.createElement(
            "include", props={"file": "$(find sim_env)/launch/include/robots/start_robots.launch.xml", "if": "$(eval arg('robot_number') > 1)"}
        )
        cycle.append(XMLGenerator.createElement("arg", props={"name": "robot_number", "value": "$(eval arg('robot_number') - 1)"}))

        launch.append(include)
        launch.append(cycle)
        XMLGenerator.indent(launch)

        with open(path, "wb+") as f:
            ET.ElementTree(launch).write(f, encoding="utf-8", xml_declaration=True)

    def writePedestrianWorld(self, path):
        """
        Create configure file to construct pedestrians environment.

        Parameters
        ----------
        path: str
            the path of file(.launch.xml) to write.
        """

        def createHuman(config, index):
            """
            Create human element of world file.

            Parameters
            ----------
            config: dict
                configure data structure.
            index: int
                human index

            Return
            ----------
            human: ET.Element
                human element ptr
            """

            def createCollision(name, scale, pose=None):
                if pose:
                    props = {"scale": " ".join("%s" % s for s in scale), "pose": " ".join("%s" % p for p in pose)}
                else:
                    props = {"scale": " ".join("%s" % s for s in scale)}
                return XMLGenerator.createElement("collision", text=name, props=props)

            # configure
            sfm = config["social_force"]
            human = config["pedestrians"][index]

            # root: actor
            actor = XMLGenerator.createElement("actor", props={"name": human["name"]})

            # human initial pose
            pose = XMLGenerator.createElement("pose", text=human["pose"])

            # human skin
            skin = XMLGenerator.createElement("skin")
            skin.append(XMLGenerator.createElement("filename", text="walk.dae"))
            skin.append(XMLGenerator.createElement("scale", text="1.0"))

            animation = XMLGenerator.createElement("animation", props={"name": "walking"})
            animation.append(XMLGenerator.createElement("filename", text="walk.dae"))
            animation.append(XMLGenerator.createElement("scale", text="1.0"))
            animation.append(XMLGenerator.createElement("interpolate_x", text="true"))

            # plugin
            if not index:
                plugin_visual = XMLGenerator.createElement("plugin", props={"name": "pedestrian_visual", "filename": "libPedestrianVisualPlugin.so"})
            else:
                plugin_visual = None
                
            plugin = XMLGenerator.createElement("plugin", props={"name": human["name"] + "_plugin", "filename": "libPedestrianSFMPlugin.so"})
            plugin.append(createCollision("LHipJoint_LeftUpLeg_collision", [0.01, 0.001, 0.001]))
            plugin.append(createCollision("LeftUpLeg_LeftLeg_collision", [8.0, 8.0, 1.0]))
            plugin.append(createCollision("LeftLeg_LeftFoot_collision", [10.0, 10.0, 1.5]))
            plugin.append(createCollision("LeftFoot_LeftToeBase_collision", [4.0, 4.0, 1.5]))
            plugin.append(createCollision("RHipJoint_RightUpLeg_collision", [0.01, 0.001, 0.001]))
            plugin.append(createCollision("RightUpLeg_RightLeg_collision", [8.0, 8.0, 1.0]))
            plugin.append(createCollision("RightLeg_RightFoot_collision", [10.0, 10.0, 1.5]))
            plugin.append(createCollision("RightFoot_RightToeBase_collision", [4.0, 4.0, 1.5]))
            plugin.append(createCollision("Spine_Spine1_collision", [0.01, 0.001, 0.001]))
            plugin.append(createCollision("Neck_Neck1_collision", [0.01, 0.001, 0.001]))
            plugin.append(createCollision("Neck1_Head_collision", [5.0, 5.0, 3.0]))
            plugin.append(createCollision("LeftShoulder_LeftArm_collision", [0.01, 0.001, 0.001]))
            plugin.append(createCollision("LeftArm_LeftForeArm_collision", [5.0, 5.0, 1.0]))
            plugin.append(createCollision("LeftForeArm_LeftHand_collision", [5.0, 5.0, 1.0]))
            plugin.append(createCollision("LeftFingerBase_LeftHandIndex1_collision", [4.0, 4.0, 3.0]))
            plugin.append(createCollision("RightShoulder_RightArm_collision", [0.01, 0.001, 0.001]))
            plugin.append(createCollision("RightArm_RightForeArm_collision", [5.0, 5.0, 1.0]))
            plugin.append(createCollision("RightForeArm_RightHand_collision", [5.0, 5.0, 1.0]))
            plugin.append(createCollision("RightFingerBase_RightHandIndex1_collision", [4.0, 4.0, 3.0]))
            plugin.append(createCollision("LowerBack_Spine_collision", [12.0, 20.0, 5.0], [0.05, 0, 0, 0, -0.2, 0]))

            plugin.append(XMLGenerator.createElement("velocity", text=str(human["velocity"])))
            plugin.append(XMLGenerator.createElement("radius", text=str(human["radius"])))
            plugin.append(XMLGenerator.createElement("cycle", text=str(human["cycle"])))
            plugin.append(XMLGenerator.createElement("animation_factor", text=str(sfm["animation_factor"])))
            plugin.append(XMLGenerator.createElement("people_distance", text=str(sfm["people_distance"])))
            plugin.append(XMLGenerator.createElement("goal_weight", text=str(sfm["goal_weight"])))
            plugin.append(XMLGenerator.createElement("obstacle_weight", text=str(sfm["obstacle_weight"])))
            plugin.append(XMLGenerator.createElement("social_weight", text=str(sfm["social_weight"])))
            plugin.append(XMLGenerator.createElement("group_gaze_weight", text=str(sfm["group_gaze_weight"])))
            plugin.append(XMLGenerator.createElement("group_coh_weight", text=str(sfm["group_coh_weight"])))
            plugin.append(XMLGenerator.createElement("group_rep_weight", text=str(sfm["group_rep_weight"])))

            ignore_obstacles = XMLGenerator.createElement("ignore_obstacles")
            for model in human["ignore"].values():
                ignore_obstacles.append(XMLGenerator.createElement("model", text=model))

            trajectory = XMLGenerator.createElement("trajectory")
            for goal in human["trajectory"].values():
                trajectory.append(XMLGenerator.createElement("goalpoint", text=goal))

            plugin.append(ignore_obstacles)
            plugin.append(trajectory)

            actor.append(pose)
            actor.append(skin)
            actor.append(animation)
            actor.append(plugin)
            if not plugin_visual is None:
                actor.append(plugin_visual)
            
            XMLGenerator.indent(actor)

            return actor

        if not "pedestrians" in self.user_cfg or not self.user_cfg["pedestrians"] or not self.user_cfg["world"]:
            return
        else:
            ped_cfg = XMLGenerator.yamlParser(self.root_path + "user_config/" + self.user_cfg["pedestrians"])

        world_file = self.root_path + "/sim_env/worlds/" + self.user_cfg["world"] + ".world"
        tree = ET.parse(world_file)
        world = tree.getroot().find("world")

        human_num = len(ped_cfg["pedestrians"])
        for i in range(human_num):
            world.append(createHuman(ped_cfg, i))

        with open(path, "wb+") as f:
            tree.write(f, encoding="utf-8", xml_declaration=True)

    @staticmethod
    def yamlParser(path: str) -> dict:
        """
        Parser user configure file(.yaml).

        Parameters
        ----------
        path: str
            the path of file(.yaml).

        Return
        ----------
        data: dict
            user configuer tree
        """
        with open(path, "r") as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
        return data

    @staticmethod
    def indent(elem: ET.Element, level: int = 0, tab_size: int = 2) -> None:
        """
        Format the generated xml document.

        Parameters
        ----------
        elem: ET.Element
        level: int
            indent level.
        """
        i = "\n" + level * " " * tab_size
        if len(elem):
            if not elem.text or not elem.text.strip():
                elem.text = i + " " * tab_size
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
            for elem in elem:
                XMLGenerator.indent(elem, level + 1)
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
        else:
            if level and (not elem.tail or not elem.tail.strip()):
                elem.tail = i

    @staticmethod
    def createElement(name: str, text: str = None, props: dict = {}) -> ET.Element:
        e = ET.Element(name, attrib=props)
        if not text is None:
            e.text = text
        return e


xml_gen = XMLGenerator()
xml_gen.writeEnvMainXml(xml_gen.root_path + "sim_env/launch/main.launch")
xml_gen.writeRobotsXml(xml_gen.root_path + "sim_env/launch/include/robots/start_robots.launch.xml")
xml_gen.writePedestrianWorld(xml_gen.root_path + "sim_env/worlds/" + xml_gen.user_cfg["world"] + "_with_pedestrians.world")
