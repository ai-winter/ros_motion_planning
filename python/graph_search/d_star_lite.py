'''
@file: d_star_lite.py
@breif: D* Lite motion planning
@author: Winter
@update: 2023.1.17
'''
import os, sys
import heapq

sys.path.append(os.path.abspath(os.path.join(__file__, "../../")))

from .graph_search import GraphSearcher
from .lpa_star import LPAStar, LNode
from utils import Env

class DStarLite(LPAStar):
    '''
    Class for D* Lite motion planning.
    [1] D* Lite

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
    >>> from utils import Grid
    >>> from graph_search import DStarLite
    >>> start = (5, 5)
    >>> goal = (45, 25)
    >>> env = Grid(51, 31)
    >>> planner = DStarLite(start, goal, env)
    >>> planner.run()
    '''
    def __init__(self, start: tuple, goal: tuple, env: Env, heuristic_type: str = "euclidean") -> None:
        GraphSearcher.__init__(self, start, goal, env, heuristic_type)
        # start and goal
        self.start = LNode(start, float('inf'), float('inf'), None)
        self.goal = LNode(goal, float('inf'), 0.0, None)
        # correction
        self.km = 0
        # OPEN set and expand zone
        self.U, self.EXPAND = [], []

        # intialize global information, record history infomation of map grids
        self.map = [LNode(s, float("inf"), float("inf"), None) for s in self.env.grid_map]
        self.map[self.map.index(self.goal)] = self.goal
        self.map[self.map.index(self.start)] = self.start
        # OPEN set with priority
        self.goal.key = self.calculateKey(self.goal)
        heapq.heappush(self.U, self.goal)

    def __str__(self) -> str:
        return "D* Lite"

    def OnPress(self, event):
        '''
        Mouse button callback function.
        '''
        x, y = int(event.xdata), int(event.ydata)
        if x < 0 or x > self.env.x_range - 1 or y < 0 or y > self.env.y_range - 1:
            print("Please choose right area!")
        else:
            print("Change position: x = {}, y = {}".format(x, y))

            cur_start, new_start = self.start, self.start
            update_start = True
            cost, count = 0, 0
            path = [self.start.current]
            self.EXPAND = []

            while cur_start != self.goal:
                neighbors = [node_n for node_n in self.getNeighbor(cur_start)
                    if not self.isCollision(cur_start, node_n)]
                next_node = min(neighbors, key=lambda n: n.g)
                path.append(next_node.current)
                cost += self.cost(cur_start, next_node)
                count += 1
                cur_start = next_node

                if update_start:
                    update_start = False
                    self.km = self.h(cur_start, new_start)
                    new_start = cur_start

                    node_change = self.map[self.map.index(LNode((x, y), None, None, None))]
                    if (x, y) not in self.obstacles:
                        self.obstacles.add((x, y))
                    else:
                        self.obstacles.remove((x, y))
                        self.updateVertex(node_change)
                    
                    self.env.update(self.obstacles)
                    for node_n in self.getNeighbor(node_change):
                        self.updateVertex(node_n)

                    self.computeShortestPath()    
        
            # animation
            self.plot.clean()
            self.plot.animation(path, str(self), cost, self.EXPAND)
            self.plot.update()

    def computeShortestPath(self) -> None:
        '''
        Perceived dynamic obstacle information to optimize global path.
        '''
        while True:
            node = min(self.U, key=lambda node: node.key)
            if node.key >= self.calculateKey(self.start) and \
                    self.start.rhs == self.start.g:
                break

            self.U.remove(node)
            self.EXPAND.append(node)

            # affected by obstacles
            if node.key < self.calculateKey(node):
                node.key = self.calculateKey(node)
                heapq.heappush(self.U, node)
            # Locally over-consistent -> Locally consistent
            elif node.g > node.rhs:
                node.g = node.rhs
                for node_n in self.getNeighbor(node):
                    self.updateVertex(node_n)
            # Locally under-consistent -> Locally over-consistent
            else:
                node.g = float("inf")
                self.updateVertex(node)
                for node_n in self.getNeighbor(node):
                    self.updateVertex(node_n)

    def updateVertex(self, node: LNode) -> None:
        '''
        Update the status and the current cost to node and it's neighbor.
        '''
        # greed correction(reverse searching)
        if node != self.goal:
            node.rhs = min([node_n.g + self.cost(node_n, node)
                        for node_n in self.getNeighbor(node)])

        if node in self.U:
            self.U.remove(node)

        # Locally unconsistent nodes should be added into OPEN set (set U)
        if node.g != node.rhs:
            node.key = self.calculateKey(node)
            heapq.heappush(self.U, node)

    def calculateKey(self, node: LNode) -> list:
        '''
        Calculate priority of node.
        ''' 
        return [min(node.g, node.rhs) + self.h(node, self.start) + self.km,
                min(node.g, node.rhs)]

    def extractPath(self):
        '''
        Extract the path based on greedy policy.

        Return
        ----------
        cost: float
            the cost of planning path
        path: list
            the planning path
        '''
        node = self.start
        path = [node.current]
        cost, count = 0, 0
        while node != self.goal:
            neighbors = [node_n for node_n in self.getNeighbor(node) if not self.isCollision(node, node_n)]
            next_node = min(neighbors, key=lambda n: n.g)
            path.append(next_node.current)
            cost += self.cost(node, next_node)
            node = next_node
            count += 1
            if count == 1000:
                return cost, []
        return cost, list(path)