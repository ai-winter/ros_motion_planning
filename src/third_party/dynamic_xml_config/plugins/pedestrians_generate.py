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

from .xml_generate import XMLGenerator


class PedGenerator(XMLGenerator):
    def __init__(self) -> None:
        super().__init__()
        if "plugins" in self.user_cfg.keys() and "pedestrians" in self.user_cfg["plugins"] and self.user_cfg["plugins"]["pedestrians"]:
            self.ped_cfg = PedGenerator.yamlParser(self.root_path + "user_config/" + self.user_cfg["plugins"]["pedestrians"])
        else:
            self.ped_cfg = None

    def __str__(self) -> str:
        return "Pedestrians Generator"

    def plugin(self):
        """
        Implement of pedestrian application.
        """
        app_register = []
        if not self.ped_cfg is None:
            """dynamic confiugre"""
            ped_path = self.root_path + "sim_env/worlds/" + self.user_cfg["world"] + "_with_pedestrians.world"
            self.writePedestrianWorld(ped_path)

            """app register"""
            # world generation
            ped_world = PedGenerator.createElement("arg", props={"name": "world_parameter", "value": self.user_cfg["world"] + "_with_pedestrians"})
            app_register.append(ped_world)
        else:
            # world generation
            ped_world = PedGenerator.createElement("arg", props={"name": "world_parameter", "value": self.user_cfg["world"]})
            app_register.append(ped_world)

        return app_register

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
                return PedGenerator.createElement("collision", text=name, props=props)

            # configure
            sfm = config["social_force"]
            human = config["pedestrians"]["ped_property"][index]
            upd_rate = config["pedestrians"]["update_rate"]

            # root: actor
            actor = PedGenerator.createElement("actor", props={"name": human["name"]})

            # human initial pose
            pose = PedGenerator.createElement("pose", text=human["pose"])

            # human skin
            skin = PedGenerator.createElement("skin")
            skin.append(PedGenerator.createElement("filename", text="walk.dae"))
            skin.append(PedGenerator.createElement("scale", text="1.0"))

            animation = PedGenerator.createElement("animation", props={"name": "walking"})
            animation.append(PedGenerator.createElement("filename", text="walk.dae"))
            animation.append(PedGenerator.createElement("scale", text="1.0"))
            animation.append(PedGenerator.createElement("interpolate_x", text="true"))

            # plugin
            if not index:
                plugin_visual = PedGenerator.createElement("plugin", props={"name": "pedestrian_visual", "filename": "libPedestrianVisualPlugin.so"})
                plugin_visual.append(PedGenerator.createElement("update_rate", text=str(upd_rate)))
            else:
                plugin_visual = None

            plugin = PedGenerator.createElement("plugin", props={"name": human["name"] + "_plugin", "filename": "libPedestrianSFMPlugin.so"})
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

            plugin.append(PedGenerator.createElement("velocity", text=str(human["velocity"])))
            plugin.append(PedGenerator.createElement("radius", text=str(human["radius"])))
            plugin.append(PedGenerator.createElement("cycle", text=str(human["cycle"])))
            plugin.append(PedGenerator.createElement("animation_factor", text=str(sfm["animation_factor"])))
            plugin.append(PedGenerator.createElement("people_distance", text=str(sfm["people_distance"])))
            plugin.append(PedGenerator.createElement("goal_weight", text=str(sfm["goal_weight"])))
            plugin.append(PedGenerator.createElement("obstacle_weight", text=str(sfm["obstacle_weight"])))
            plugin.append(PedGenerator.createElement("social_weight", text=str(sfm["social_weight"])))
            plugin.append(PedGenerator.createElement("group_gaze_weight", text=str(sfm["group_gaze_weight"])))
            plugin.append(PedGenerator.createElement("group_coh_weight", text=str(sfm["group_coh_weight"])))
            plugin.append(PedGenerator.createElement("group_rep_weight", text=str(sfm["group_rep_weight"])))

            if "time_delay" in human.keys():
                plugin.append(PedGenerator.createElement("time_delay", text=str(human["time_delay"])))

            ignore_obstacles = PedGenerator.createElement("ignore_obstacles")
            for model in human["ignore"].values():
                ignore_obstacles.append(PedGenerator.createElement("model", text=model))

            trajectory = PedGenerator.createElement("trajectory")
            for goal in human["trajectory"].values():
                trajectory.append(PedGenerator.createElement("goalpoint", text=goal))

            plugin.append(ignore_obstacles)
            plugin.append(trajectory)

            actor.append(pose)
            actor.append(skin)
            actor.append(animation)
            actor.append(plugin)

            if not plugin_visual is None:
                actor.append(plugin_visual)

            PedGenerator.indent(actor)

            return actor

        if not self.ped_cfg is None:
            world_file = self.root_path + "sim_env/worlds/" + self.user_cfg["world"] + ".world"
            tree = ET.parse(world_file)
            world = tree.getroot().find("world")

            human_num = len(self.ped_cfg["pedestrians"]["ped_property"])
            for i in range(human_num):
                world.append(createHuman(self.ped_cfg, i))

            with open(path, "wb+") as f:
                tree.write(f, encoding="utf-8", xml_declaration=True)
