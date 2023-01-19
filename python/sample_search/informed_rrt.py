'''
@file: informed_rrt.py
@breif: Informed RRT* motion planning
@author: Winter
@update: 2023.1.18
'''
import os, sys
import numpy as np
from functools import partial
import matplotlib.pyplot as plt

sys.path.append(os.path.abspath(os.path.join(__file__, "../../")))

from .rrt_star import RRTStar
from utils import Env, Node

class ellipse:
    '''
    Ellipse sampling.
    '''    
    @staticmethod
    def transform(a: float, c: float, p1: tuple, p2: tuple) -> np.ndarray:
        # center
        center_x = (p1[0] + p2[0]) / 2
        center_y = (p1[1] + p2[1]) / 2

        # rotation
        theta = - np.arctan2(p2[1] - p1[1], p2[0] - p1[0])

        # transform
        b = np.sqrt(a ** 2 - c ** 2)
        T = np.array([[ a * np.cos(theta), b * np.sin(theta), center_x],
                      [-a * np.sin(theta), b * np.cos(theta), center_y],
                      [                 0,                 0,        1]])
        return T

class InformedRRT(RRTStar):
    '''
    Class for Informed RRT* motion planning.
    [1] Optimal Sampling-based Path Planning Focused via Direct
        Sampling of an Admissible Ellipsoidal heuristic

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
    r: float
        optimization radius
    goal_sample_rate: float
        heuristic sample

    Examples
    ----------
    >>> from utils import Map
    >>> from sample_search import InformedRRT
    >>> start = (5, 5)
    >>> goal = (45, 25)
    >>> env = Map(51, 31)
    >>> planner = InformedRRT(start, goal, env)
    >>> planner.run()
    '''
    def __init__(self, start: tuple, goal: tuple, env: Env, max_dist: float,
                 sample_num: int, r: float, goal_sample_rate: float = 0.05) -> None:
        super().__init__(start, goal, env, max_dist, sample_num, goal_sample_rate)
        # optimization radius
        self.r = r
        # best planning cost
        self.c_best = float("inf")
        # distance between start and goal
        self.c_min = self.dist(self.start, self.goal)
        # ellipse sampling
        self.transform = partial(ellipse.transform, c=self.c_min / 2, p1=start, p2=goal)
    
    def __str__(self) -> str:
        return "Informed RRT*"

    def plan(self):
        '''
        Informed-RRT* motion plan function.

        Return
        ----------
        cost: float
            path cost
        path: list
            planning path
        '''
        # generate a random node in the map
        node_rand = self.generateRandomNode()

        # visited
        if node_rand in self.sample_list:
            return 0, None

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
        best_cost, best_path = float("inf"), None

        # main loop
        for i in range(self.sample_num):
            cost, path = self.plan()
            # update
            if path and cost < best_cost:
                self.c_best = best_cost = cost
                best_path = path
            # animation
            if  i % 30 == 0:
                self.animation(best_path, best_cost)
        plt.show()

    def generateRandomNode(self) -> Node:
        '''
        Generate a random node to extend exploring tree.

        Return
        ----------
        node: Node
            a random node based on sampling
        '''
        # ellipse sample
        if self.c_best < float("inf"):
            while True:
                # unit ball sample
                p = np.array([.0, .0, 1.])
                while True:
                    x, y = np.random.uniform(-1, 1), np.random.uniform(-1, 1)
                    if x ** 2 + y ** 2 < 1:
                        p[0], p[1] = x, y
                        break
                # transform to ellipse
                p_star = self.transform(self.c_best / 2) @ p.T
                if self.delta <= p_star[0] <= self.env.x_range - self.delta and \
                   self.delta <= p_star[1] <= self.env.y_range - self.delta:
                    return Node((p_star[0], p_star[1]), None, 0, 0)
        # random sample
        else:
            return super().generateRandomNode()

    def animation(self, path, cost):
        self.plot.clean()
        name = str(self) + "\ncost: " + str(cost)
        self.plot.plotEnv(name)
        for x in self.sample_list:
            if x.parent:
                plt.plot([x.parent[0], x.current[0]], [x.parent[1], x.current[1]], 
                    color="#dddddd", linestyle="-")
        if self.c_best < float("inf"):
            self.drawEllipse()
        if path:
            self.plot.plotPath(path)
        plt.pause(0.01)

    def drawEllipse(self):
        t = np.arange(0, 2 * np.pi + 0.1, 0.1)
        x = [np.cos(it) for it in t]
        y = [np.sin(it) for it in t]
        z = [1 for _ in t]
        fx = self.transform(self.c_best / 2) @ np.array([x, y, z])
        plt.plot(fx[0, :], fx[1, :], linestyle='--', color='darkorange', linewidth=2)