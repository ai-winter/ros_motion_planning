'''
@file: rrt_connected.py
@breif: RRT-Connected motion planning
@author: Winter
@update: 2023.1.17
'''
import os, sys
import math
import numpy as np

sys.path.append(os.path.abspath(os.path.join(__file__, "../../")))

from .rrt import RRT
from utils import Env, Node

class RRT(RRT):
    def __init__(self, start: tuple, goal: tuple, env: Env, max_dist: float, 
        sample_num: int, goal_sample_rate: float = 0.05) -> None:
        super().__init__(start, goal, env, max_dist, sample_num, goal_sample_rate)
        # Sampled list forward
        self.sample_list_f = [self.start]
        # Sampled list backward
        self.sample_list_b = [self.goal]
    
    def plan(self):
        pass