'''
@file: rrt_star.py
@breif: RRT-Star motion planning
@author: Winter
@update: 2023.1.18
'''
import os, sys

sys.path.append(os.path.abspath(os.path.join(__file__, "../../")))

from .rrt import RRT
from utils import Env, Node

class RRTStar(RRT):
    '''
    Class for RRT-Star motion planning.
    [1] Sampling-based algorithms for optimal motion planning

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
    >>> from sample_search import RRTStar
    >>> start = (5, 5)
    >>> goal = (45, 25)
    >>> env = Map(51, 31)
    >>> planner = RRTStar(start, goal, env)
    >>> planner.run()
    '''
    def __init__(self, start: tuple, goal: tuple, env: Env, max_dist: float,
                 sample_num: int, r: float, goal_sample_rate: float = 0.05) -> None:
        super().__init__(start, goal, env, max_dist, sample_num, goal_sample_rate)
        # optimization radius
        self.r = r
    
    def __str__(self) -> str:
        return "RRT*"
    
    def getNearest(self, node_list: list, node: Node) -> Node:
        '''
        Get the node from `node_list` that is nearest to `node` with optimization.

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
        node_new = super().getNearest(node_list, node)
        if node_new:
            #  rewire optimization
            for node_n in node_list:
                #  inside the optimization circle
                new_dist = self.dist(node_n, node_new)
                if new_dist < self.r:
                    cost = node_n.g + new_dist
                    #  update new sample node's cost and parent
                    if node_new.g > cost and not self.isCollision(node_n, node_new):
                        node_new.parent = node_n.current
                        node_new.g = cost
                    else:
                        #  update nodes' cost inside the radius
                        cost = node_new.g + new_dist
                        if node_n.g > cost and not self.isCollision(node_n, node_new):
                            node_n.parent = node_new.current
                            node_n.g = cost
                else:
                    continue
            return node_new
        else:
            return None 
        