#!/usr/bin/python
# -*- coding: utf-8 -*-

""" 
******************************************************************************************
*  Copyright (C) 2023 Yang Haodong, All Rights Reserved                                  *
*                                                                                        *
*  @brief    generate launch file dynamicly based on user configure.                     *
*  @author   Haodong Yang                                                                *
*  @version  1.0.0                                                                       *
*  @date     2023.12.29                                                                  *
*  @license  GNU General Public License (GPL)                                            *
******************************************************************************************
"""
import yaml
from .xml_generate import XMLGenerator

class MapsGenerator(XMLGenerator):
	def __init__(self) -> None:
		super().__init__()
		if "plugins" in self.user_cfg.keys() and "map_layers" in self.user_cfg["plugins"] and self.user_cfg["plugins"]["map_layers"]:
			self.maps_cfg = MapsGenerator.yamlParser(self.root_path + "user_config/" + self.user_cfg["plugins"]["map_layers"])
		else:
			self.maps_cfg = None

	def plugin(self):
		"""
		Implement of maps layer application.
		"""
		app_register = []
		if not self.maps_cfg is None:
			if "global_costmap" in self.maps_cfg.keys():
				global_maps_list = self.maps_cfg["global_costmap"]
				global_cfg_path = self.root_path + "sim_env/config/costmap/global_costmap_plugins.yaml"
				global_maps_cfg = {
					"global_costmap": {
						"plugins": [
							{"name": name, "type": self.name2type(name)}
							for name in global_maps_list
						]
					}
				}
				with open(global_cfg_path, 'w', encoding='utf-8') as f:
					yaml.dump(global_maps_cfg, f, default_flow_style=None, sort_keys=False)

			if "local_costmap" in self.maps_cfg.keys():
				local_maps_list = self.maps_cfg["local_costmap"]
				loacl_cfg_path = self.root_path + "sim_env/config/costmap/local_costmap_plugins.yaml"
				local_maps_cfg = {
						"local_costmap": {
							"plugins": [
								{"name": name, "type": self.name2type(name)}
								for name in local_maps_list
							]
						}
					}
				with open(loacl_cfg_path, 'w', encoding='utf-8') as f:
					yaml.dump(local_maps_cfg, f, default_flow_style=None, sort_keys=False)

		return app_register
	

	def name2type(self, name: str) -> str:
		if name == "static_map":
			return "costmap_2d::StaticLayer"
		elif name == "voxel_layer":
			return "costmap_2d::VoxelLayer"
		elif name == "obstacle_layer":
			return "costmap_2d::ObstacleLayer"
		elif name == "voronoi_layer":
			return "costmap_2d::VoronoiLayer"
		elif name == "inflation_layer":
			return "costmap_2d::InflationLayer"
		else:
			raise NotImplementedError("The name of map layer is invalid, please correct it!")