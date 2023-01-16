'''
@file: env.py
@breif: 2-dimension environment and graph node
@author: Winter
@update: 2023.1.13
'''
from math import sqrt

class Node(object):
    '''
    Class for searching nodes.

    Parameters
    ----------
    current: tuple
        current coordinate
    parent: tuple
        coordinate of parent node
    g: float
        path cost
    h: float
        heuristic cost

    Examples
    ----------
    >>> from env import Node
    >>> node1 = Node((1, 0), (2, 3), 1, 2)
    >>> node2 = Node((1, 0), (2, 5), 2, 8)
    >>> node3 = Node((2, 0), (1, 6), 3, 1)
    ...
    >>> node1 + node2
    >>> Node((2, 0), (2, 3), 3, 2)
    ...
    >>> node1 == node2
    >>> True
    ...
    >>> node1 != node3
    >>> True
    '''
    def __init__(self, current: tuple, parent: tuple, g: float, h: float) -> None:
        self.current = current
        self.parent = parent
        self.g = g
        self.h = h
    
    def __add__(self, node):
        return Node((self.current[0] + node.current[0], self.current[1] + node.current[1]), 
                     self.parent, self.g + node.g, self.h)

    def __eq__(self, node) -> bool:
        return self.current == node.current
    
    def __ne__(self, node) -> bool:
        return not self.__eq__(node)

    def __lt__(self, node) -> bool:
        return self.g + self.h < node.g + node.h or \
                (self.g + self.h == node.g + node.h and self.h < node.h)

    def __str__(self) -> str:
        return "----------\ncurrent:{}\nparent:{}\ng:{}\nh:{}\n----------" \
            .format(self.current, self.parent, self.g, self.h)

class Env:
    '''
    Class for building 2-d workspace of robots.

    Parameters
    ----------
    x_range: int
        x-axis range of enviroment
    y_range: int
        y-axis range of environmet

    Examples
    ----------
    >>> from utils import Env
    >>> env = Env(30, 40)
    '''
    def __init__(self, x_range: int, y_range: int) -> None:
        # size of environment
        self.x_range = x_range  
        self.y_range = y_range

        # allowed motions
        self.motions = [Node((-1, 0), None, 1, None), Node((-1, 1),  None, sqrt(2), None),
                        Node((0, 1),  None, 1, None), Node((1, 1),   None, sqrt(2), None),
                        Node((1, 0),  None, 1, None), Node((1, -1),  None, sqrt(2), None),
                        Node((0, -1), None, 1, None), Node((-1, -1), None, sqrt(2), None)]
        # obstacles
        self.obstacles = self.initObstacles()

    @property
    def grid_map(self) -> set:
        return {(i, j) for i in range(self.x_range) for j in range(self.y_range)}

    def initObstacles(self) -> set:
        '''
        Initialize obstacles' positions.

        Return
        ----------
        obstacles: set
            map of obstacles
        '''
        x, y = self.x_range, self.y_range
        obstacles = set()

        # boundary of environment
        for i in range(x):
            obstacles.add((i, 0))
            obstacles.add((i, y - 1))
        for i in range(y):
            obstacles.add((0, i))
            obstacles.add((x - 1, i))

        # user-defined obstacles
        for i in range(10, 21):
            obstacles.add((i, 15))
        for i in range(15):
            obstacles.add((20, i))
        for i in range(15, 30):
            obstacles.add((30, i))
        for i in range(16):
            obstacles.add((40, i))

        return obstacles

    def updateObstacles(self, obstacles):
        self.obstacles = obstacles
