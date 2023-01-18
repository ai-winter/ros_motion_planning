'''
@file: rrt.py
@breif: RRT motion planning
@author: Winter
@update: 2023.1.17
'''
import os, sys
import math
import numpy as np

sys.path.append(os.path.abspath(os.path.join(__file__, "../../")))

from .sample_search import SampleSearcher
from utils import Env, Node

class RRT(SampleSearcher):
    '''
    Class for RRT motion planning.
    [1] Rapidly-Exploring Random Trees: A New Tool for Path Planning

    Parameters
    ----------
    start: tuple
        start point coordinate
    goal: tuple
        goal point coordinate
    env: Env
        environment
    max_dist: float
        Maximum expansion distance one step
    sample_num: int
        Maximum number of sample points
    goal_sample_rate: float
        heuristic sample

    Examples
    ----------
    >>> from utils import Map
    >>> from sample_search import RRT
    >>> start = (5, 5)
    >>> goal = (45, 25)
    >>> env = Map(51, 31)
    >>> planner = RRT(start, goal, env)
    >>> planner.run()
    '''
    def __init__(self, start: tuple, goal: tuple, env: Env, max_dist: float, 
        sample_num: int, goal_sample_rate: float=0.05) -> None:
        super().__init__(start, goal, env)
        # Maximum expansion distance one step
        self.max_dist = max_dist
        # Maximum number of sample points
        self.sample_num = sample_num
        # heuristic sample
        self.goal_sample_rate = goal_sample_rate
        # Sampled list
        self.sample_list = [self.start]

    def __str__(self) -> str:
        return "Rapidly-exploring Random Tree(RRT)"

    def plan(self):
        '''
        RRT motion plan function.

        Return
        ----------
        cost: float
            path cost
        path: list
            planning path
        '''
        # main loop
        for _ in range(self.sample_num):
            # generate a random node in the map
            node_rand = self.generateRandomNode()

            # visited
            if node_rand in self.sample_list:
                continue
            
            # generate new node
            node_new = self.getNearest(self.sample_list, node_rand)
            if node_new:
                self.sample_list.append(node_new)
                dist = self.dist(node_new, self.goal)
                # goal found
                if dist <= self.max_dist and not self.isCollision(node_new, self.goal):
                    self.goal.parent = node_new.current
                    self.goal.g = node_new.g + self.dist(self.goal, node_new)
                    self.sample_list.append(self.goal)
                    return self.extractPath(self.sample_list)
        return 0, None

    def run(self) -> None:
        '''
        Running both plannig and animation.
        '''
        cost, path = self.plan()
        self.plot.animation(path, str(self), cost, self.sample_list)

    def generateRandomNode(self) -> Node:
        '''
        Generate a random node to extend exploring tree.

        Return
        ----------
        node: Node
            a random node based on sampling
        '''
        if np.random.random() > self.goal_sample_rate:
            current = (np.random.uniform(self.delta, self.env.x_range - self.delta),
                    np.random.uniform(self.delta, self.env.y_range - self.delta))
            return Node(current, None, 0, 0)
        return self.goal

    def getNearest(self, node_list: list, node: Node) -> Node:
        '''
        Get the node from `node_list` that is nearest to `node`.

        Parameters
        ----------
        node_list: list
            exploring list
        node: Node
            currently generated node

        Return
        ----------
        node: Node
            nearest node 
        '''
        # find nearest neighbor
        dist = [self.dist(node, nd) for nd in node_list]
        node_near = node_list[int(np.argmin(dist))]

        # regular and generate new node
        dist, theta = self.dist(node_near, node), self.angle(node_near, node)
        dist = min(self.max_dist, dist)
        node_new = Node((node_near.current[0] + dist * math.cos(theta),
                        (node_near.current[1] + dist * math.sin(theta))),
                         node_near.current, node_near.g + dist, 0)
        
        # obstacle check
        if self.isCollision(node_new, node_near):
            return None
        return node_new

    def extractPath(self, closed_set):
        '''
        Extract the path based on the CLOSED set.

        Parameters
        ----------
        closed_set: list
            CLOSED set

        Return
        ----------
        cost: float
            the cost of planning path
        path: list
            the planning path
        '''
        node = closed_set[closed_set.index(self.goal)]
        path = [node.current]
        cost = node.g
        while node != self.start:
            node_parent = closed_set[closed_set.index(Node(node.parent, None, None, None))]
            node = node_parent
            path.append(node.current)

        return cost, path
