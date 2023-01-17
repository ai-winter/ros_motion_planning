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

from utils import Env, Node, Plot

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
        self.obstacles = self.env.obstacles
        # graph handler
        self.plot = Plot(self.start.current, self.goal.current, self.env)

    def h(self, node: Node, goal: Node) -> float:
        '''
        Calculate heuristic.

        Parameters
        ----------
        node: Node
            current node
        goal: Node
            goal node

        Return
        ----------
        h: float
            heuristic function value of node
        '''
        if self.heuristic_type == "manhattan":
            return abs(goal.current[0] - node.current[0]) + abs(goal.current[1] - node.current[1])
        elif self.heuristic_type == "euclidean":
            return math.hypot(goal.current[0] - node.current[0], goal.current[1] - node.current[1])

    def cost(self, node1: Node, node2: Node) -> float:
        '''
        Calculate cost for this motion.
        '''
        if self.isCollision(node1, node2):
            return float("inf")
        return math.hypot(node2.current[0] - node1.current[0], node2.current[1] - node1.current[1])

    def isCollision(self, node1: Node, node2: Node) -> bool:
        '''
        Judge collision when moving from node1 to node2.

        Parameters
        ----------
        node1, node2: Node

        Return
        ----------
        collision: bool
            True if collision exists else False
        '''
        if node1.current in self.obstacles or node2.current in self.obstacles:
            return True

        x1, y1 = node1.current
        x2, y2 = node2.current

        if x1 != x2 and y1 != y2:
            if x2 - x1 == y1 - y2:
                s1 = (min(x1, x2), min(y1, y2))
                s2 = (max(x1, x2), max(y1, y2))
            else:
                s1 = (min(x1, x2), max(y1, y2))
                s2 = (max(x1, x2), min(y1, y2))
            if s1 in self.obstacles or s2 in self.obstacles:
                return True
        return False

    @abstractmethod
    def plan(self):
        '''
        Interface for planning.
        '''
        pass

    @abstractmethod
    def run(self):
        '''
        Interface for running both plannig and animation.
        '''
        pass
