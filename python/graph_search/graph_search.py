'''
@file: graph_search.py
@breif: Base class for planner based on graph searching
@author: Winter
@update: 2023.1.13
'''
from abc import abstractmethod, ABC
import math
import sys, os
sys.path.append(os.path.abspath(os.path.join(__file__, "../../")))

from utils import Env, Node

class GraphSearcher(ABC):
    '''
    Base class for planner based on graph searching.

    Parameters
    ----------
    start: tuple
        start point coordinate
    goal: tuple
        goal point coordinate
    env: Env
        environment
    heuristic_type: str
        heuristic function type, default is euclidean
    '''
    def __init__(self, start: tuple, goal: tuple, env: Env, heuristic_type: str="euclidean") -> None:
        self.start = Node(start, start, 0, 0)
        self.goal = Node(goal, goal, 0, 0)
        self.env = env
        self.heuristic_type = heuristic_type

        # allowed motions
        self.motions = self.env.motions
        # obstacles
        self.obstacles = self.env.obstacle_map

    def h(self, node: Node) -> float:
        '''
        Calculate heuristic.

        Parameters
        ----------
        node: Node
            current node

        Return
        ----------
        h: float
            heuristic function value of node
        '''
        if self.heuristic_type == "manhattan":
            return abs(self.goal.current[0] - node.current[0]) + abs(self.goal.current[1] - node.current[1])
        elif self.heuristic_type == "euclidean":
            return math.hypot(self.goal.current[0] - node.current[0], self.goal.current[1] - node.current[1])

    @abstractmethod
    def plan(self):
        '''
        Interface for planning.
        '''
        pass
