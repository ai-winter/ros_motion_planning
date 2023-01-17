'''
@file: a_star.py
@breif: A* motion planning
@author: Winter
@update: 2023.1.13
'''
import os, sys
import math
import heapq

sys.path.append(os.path.abspath(os.path.join(__file__, "../../")))

from .graph_search import GraphSearcher
from utils import Env, Node

class AStar(GraphSearcher):
    '''
    Class for A* motion planning.

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
    >>> from graph_search import AStar
    >>> start = (5, 5)
    >>> goal = (45, 25)
    >>> env = Env(51, 31)
    >>> planner = AStar(start, goal, env)
    >>> planner.run()
    '''
    def __init__(self, start: tuple, goal: tuple, env: Env, heuristic_type: str = "euclidean") -> None:
        super().__init__(start, goal, env, heuristic_type)

    def __str__(self) -> str:
        return "A*"

    def plan(self):
        '''
        A* motion plan function.
        [1] A Formal Basis for the heuristic Determination of Minimum Cost Paths

        Return
        ----------
        path: list
            planning path
        expand: list
            all nodes that planner has searched
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
                # exists in CLOSED set
                if node_n in CLOSED:
                    continue
                
                node_n.parent = node.current
                node_n.h = self.h(node_n, self.goal)

                # goal found
                if node_n == self.goal:
                    heapq.heappush(OPEN, node_n)
                    break
                
                # update OPEN set
                heapq.heappush(OPEN, node_n)
            
            CLOSED.append(node)
        return [], []

    def getNeighbor(self, node: Node) -> list:
        '''
        Find neighbors of node.

        Parameters
        ----------
        node: Node
            current node

        Return
        ----------
        neighbors: list
            neighbors of current node
        '''
        return [node + motion for motion in self.motions
                if not self.isCollision(node, node + motion)]

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
        cost = 0
        node = closed_set[closed_set.index(self.goal)]
        path = [node.current]
        while node != self.start:
            index = closed_set.index(Node(node.parent, None, None, None))
            cost += math.hypot(node.current[0] - node.parent[0], node.current[1] - node.parent[1])
            node = closed_set[index]
            path.append(node.current)
        return cost, path

    def run(self):
        '''
        Running both plannig and animation.
        '''
        (cost, path), expand = self.plan()
        self.plot.animation(path, str(self), cost, expand)
