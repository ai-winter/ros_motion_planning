'''
@file: rrt_connected.py
@breif: RRT-Connected motion planning
@author: Winter
@update: 2023.1.17
'''
import os, sys
import math

sys.path.append(os.path.abspath(os.path.join(__file__, "../../")))

from .rrt import RRT
from utils import Env, Node

class RRTConnect(RRT):
    '''
    Class for RRT-Connect motion planning.
    [1] RRT-Connect: An Efficient Approach to Single-Query Path Planning

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
    >>> from sample_search import RRTConnect
    >>> start = (5, 5)
    >>> goal = (45, 25)
    >>> env = Map(51, 31)
    >>> planner = RRTConnect(start, goal, env)
    >>> planner.run()
    '''
    def __init__(self, start: tuple, goal: tuple, env: Env, max_dist: float, 
        sample_num: int, goal_sample_rate: float = 0.05) -> None:
        super().__init__(start, goal, env, max_dist, sample_num, goal_sample_rate)
        # Sampled list forward
        self.sample_list_f = [self.start]
        # Sampled list backward
        self.sample_list_b = [self.goal]
    
    def __str__(self) -> str:
        return "RRT-Connect"

    def plan(self):
        '''
        RRT-Connected motion plan function.

        Return
        ----------
        cost: float
            path cost
        path: list
            planning path
        '''
        for _ in range(self.sample_num):
            # generate a random node in the map
            node_rand = self.generateRandomNode()            
            # generate new node
            node_new = self.getNearest(self.sample_list_f, node_rand)
            if node_new:
                self.sample_list_f.append(node_new)
                # backward exploring
                node_new_b = self.getNearest(self.sample_list_b, node_new)
                if node_new_b:
                    self.sample_list_b.append(node_new_b)
                    # greedy extending
                    while True:
                        dist = min(self.max_dist, self.dist(node_new, node_new_b))
                        theta = self.angle(node_new_b, node_new)
                        node_new_b2 = Node((node_new_b.current[0] + dist * math.cos(theta),
                                           (node_new_b.current[1] + dist * math.sin(theta))),
                                            node_new_b.current, node_new_b.g + dist, 0)
                        if not self.isCollision(node_new_b2, node_new_b):
                            self.sample_list_b.append(node_new_b2)
                            node_new_b = node_new_b2
                        else:
                            break

                        if node_new_b == node_new:
                            return self.extractPath(node_new)

            if len(self.sample_list_b) < len(self.sample_list_f):
                self.sample_list_f, self.sample_list_b = self.sample_list_b, self.sample_list_f

        return 0, None

    def run(self):
        '''
        Running both plannig and animation.

        '''
        cost, path = self.plan()

        expand = []
        tree_size = max(len(self.sample_list_f), len(self.sample_list_b))
        for k in range(tree_size):
            if k < len(self.sample_list_f):
                expand.append(self.sample_list_f[k])
            if k < len(self.sample_list_b):
                expand.append(self.sample_list_b[k])

        self.plot.animation(path, str(self), cost, expand)

    def extractPath(self, boundary: Node):
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
        if self.start in self.sample_list_b:
            self.sample_list_f, self.sample_list_b = self.sample_list_b, self.sample_list_f

        # forward
        node = self.sample_list_f[self.sample_list_f.index(boundary)]
        path_f = [node.current]
        cost = node.g
        while node != self.start:
            node_parent = self.sample_list_f[self.sample_list_f.index(
                            Node(node.parent, None, None, None))]
            node = node_parent
            path_f.append(node.current)

        # backward
        node = self.sample_list_b[self.sample_list_b.index(boundary)]
        path_b = []
        cost += node.g
        while node != self.goal:
            node_parent = self.sample_list_b[self.sample_list_b.index(
                            Node(node.parent, None, None, None))]
            node = node_parent
            path_b.append(node.current)        

        return cost, list(reversed(path_f)) + path_b