'''
@file: lpa_star.py
@breif: Lifelong Planning A* motion planning
@author: Winter
@update: 2023.1.16
'''
import os, sys
import heapq

sys.path.append(os.path.abspath(os.path.join(__file__, "../../")))

from .graph_search import GraphSearcher
from utils import Env, Node

class LNode(Node):
    '''
    Class for LPA* nodes.

    Parameters
    ----------
    current: tuple
        current coordinate
    g: float
        minimum cost moving from start(predict)
    rhs: float
        minimum cost moving from start(value)
    key: list
        priority
    '''
    def __init__(self, current: tuple, g: float, rhs: float, key: list) -> None:
        self.current = current
        self.g = g
        self.rhs = rhs
        self.key = key

    def __add__(self, node):
        return LNode((self.current[0] + node.current[0], self.current[1] + node.current[1]), 
                      self.g, self.rhs, self.key)

    def __lt__(self, node) -> bool:
        return self.key < node.key

    def __str__(self) -> str:
        return "----------\ncurrent:{}\ng:{}\nrhs:{}\nkey:{}\n----------" \
            .format(self.current, self.g, self.rhs, self.key)

class LPAStar(GraphSearcher):
    '''
    Class for LPA* motion planning.
    [1] Lifelong Planning A*

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
    >>> from graph_search import LPAStar
    >>> start = (5, 5)
    >>> goal = (45, 25)
    >>> env = Grid(51, 31)
    >>> planner = LPAStar(start, goal, env)
    >>> planner.run()
    '''
    def __init__(self, start: tuple, goal: tuple, env: Env, heuristic_type: str = "euclidean") -> None:
        super().__init__(start, goal, env, heuristic_type)
        # start and goal
        self.start = LNode(start, float('inf'), 0.0, None)
        self.goal = LNode(goal, float('inf'), float('inf'), None)
        # OPEN set and expand zone
        self.U, self.EXPAND = [], []

        # intialize global information, record history infomation of map grids
        self.map = [LNode(s, float("inf"), float("inf"), None) for s in self.env.grid_map]
        self.map[self.map.index(self.goal)] = self.goal
        self.map[self.map.index(self.start)] = self.start
        # OPEN set with priority
        self.start.key = self.calculateKey(self.start)
        heapq.heappush(self.U, self.start)

    def __str__(self) -> str:
        return "Lifelong Planning A*"

    def plan(self):
        '''
        LPA* dynamic motion planning function.
        '''
        self.computeShortestPath()
        return self.extractPath(), None

    def run(self) -> None:
        '''
        Running both plannig and animation.
        '''
        # static planning
        (cost, path), _ = self.plan()        
        
        # animation
        self.plot.connect('button_press_event', self.OnPress)
        self.plot.animation(path, str(self), cost=cost)

    def OnPress(self, event):
        '''
        Mouse button callback function.
        '''
        x, y = int(event.xdata), int(event.ydata)
        if x < 0 or x > self.env.x_range - 1 or y < 0 or y > self.env.y_range - 1:
            print("Please choose right area!")
        else:
            print("Change position: x = {}, y = {}".format(x, y))
            self.EXPAND = []
            node_change = self.map[self.map.index(LNode((x, y), None, None, None))]

            if (x, y) not in self.obstacles:
                self.obstacles.add((x, y))
            else:
                self.obstacles.remove((x, y))
                self.updateVertex(node_change)
            
            self.env.update(self.obstacles)

            for node_n in self.getNeighbor(node_change):
                self.updateVertex(node_n)

            (cost, path), _ = self.plan()        
        
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
            if node.key >= self.calculateKey(self.goal) and \
                    self.goal.rhs == self.goal.g:
                break

            self.U.remove(node)
            self.EXPAND.append(node)

            # Locally over-consistent -> Locally consistent
            if node.g > node.rhs:
                node.g = node.rhs
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
        # greed correction
        if node != self.start:
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
        return [min(node.g, node.rhs) + self.h(node, self.goal),
                min(node.g, node.rhs)]

    def getNeighbor(self, node: LNode) -> list:
        '''
        Find neighbors of node.

        Parameters
        ----------
        node: DNode
            current node

        Return
        ----------
        neighbors: list
            neighbors of current node
        '''
        neighbors = []
        for motion in self.motions:
            n = self.map[self.map.index(node + motion)]
            if n.current not in self.obstacles:
                neighbors.append(n)
        return neighbors

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
        node = self.goal
        path = [node.current]
        cost, count = 0, 0
        while node != self.start:
            neighbors = [node_n for node_n in self.getNeighbor(node) if not self.isCollision(node, node_n)]
            next_node = min(neighbors, key=lambda n: n.g)
            path.append(next_node.current)
            cost += self.cost(node, next_node)
            node = next_node
            count += 1
            if count == 1000:
                return cost, []
        return cost, list(reversed(path))

