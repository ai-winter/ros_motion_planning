'''
@file: gbfs.py
@breif: Greedy Best First Search motion planning
@author: Winter
@update: 2023.1.13
'''
import os, sys
import heapq

sys.path.append(os.path.abspath(os.path.join(__file__, "../../")))

from .a_star import AStar
from utils import Env

class GBFS(AStar):
    def __init__(self, start: tuple, goal: tuple, env: Env, heuristic_type: str = "euclidean") -> None:
        super().__init__(start, goal, env, heuristic_type)
    
    def __str__(self) -> str:
        return "Greedy Best First Search(GBFS)"

    def plan(self):
        '''
        Class for Greedy Best First Search.

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

        Examples
        ----------
        >>> from utils import Env
        >>> from graph_search import GBFS
        >>> start = (5, 5)
        >>> goal = (45, 25)
        >>> env = Env(51, 31)
        >>> planner = GBFS(start, goal, env)
        >>> path, expand = planner.plan()
        '''
        # OPEN set with priority and CLOSED set
        OPEN = []
        heapq.heappush(OPEN, self.start)
        CLOSED = []

        while OPEN:
            node = heapq.heappop(OPEN)

            # exists in CLOSED set
            if node in CLOSED:
                continue

            # goal found
            if node == self.goal:
                CLOSED.append(node)
                return self.extractPath(CLOSED), CLOSED

            for node_n in self.getNeighbor(node):
             
                # hit the obstacle
                if node_n.current in self.obstacles:
                    continue
                
                # exists in CLOSED set
                if node_n in CLOSED:
                    continue
                
                node_n.parent = node.current
                node_n.h = self.h(node_n)
                node_n.g = 0

                # goal found
                if node_n == self.goal:
                    heapq.heappush(OPEN, node_n)
                    break
                
                # update OPEN set
                heapq.heappush(OPEN, node_n)
            
            CLOSED.append(node)
        return [], []