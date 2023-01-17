'''
@file: jps.py
@breif: Jump Point Search motion planning
@author: Winter
@update: 2023.1.14
'''
import os, sys
import heapq

sys.path.append(os.path.abspath(os.path.join(__file__, "../../")))

from .a_star import AStar
from utils import Env, Node

class JPS(AStar):
    '''
    Class for JPS motion planning.

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
    >>> from graph_search import JPS
    >>> start = (5, 5)
    >>> goal = (45, 25)
    >>> env = Grid(51, 31)
    >>> planner = JPS(start, goal, env)
    >>> planner.run()
    '''
    def __init__(self, start: tuple, goal: tuple, env: Env, heuristic_type: str = "euclidean") -> None:
        super().__init__(start, goal, env, heuristic_type)
    
    def __str__(self) -> str:
        return "Jump Point Search(JPS)"

    def plan(self):
        '''
        JPS motion plan function.
        [1] Online Graph Pruning for Pathfinding On Grid Maps

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

            jp_list = []
            for motion in self.motions:
                jp = self.jump(node, motion)
                # exists and not in CLOSED set
                if jp and jp not in CLOSED:
                    jp.parent = node.current
                    jp.h = self.h(jp, self.goal)
                    jp_list.append(jp)

            for jp in jp_list:
                # update OPEN set
                heapq.heappush(OPEN, jp)

                # goal found
                if jp == self.goal:
                    break
            
            CLOSED.append(node)
        return [], []

    def jump(self, node: Node, motion: Node):
        '''
        Jumping search recursively.

        Parameters
        ----------
        node: Node
            current node
        motion: Node
            the motion that current node executes

        Return
        ----------
        jump_point: Node
            jump point or None if searching fails
        '''
        # explore a new node
        new_node = node + motion
        new_node.parent = node.current
        new_node.h = self.h(new_node, self.goal)

        # hit the obstacle
        if new_node.current in self.obstacles:
            return None

        # goal found
        if new_node == self.goal:
            return new_node

        # diagonal
        if motion.current[0] and motion.current[1]:
            # if exists jump point at horizontal or vertical
            x_dir = Node((motion.current[0], 0), None, 1, None)
            y_dir = Node((0, motion.current[1]), None, 1, None)
            if self.jump(new_node, x_dir) or self.jump(new_node, y_dir):
                return new_node
            
        # if exists forced neighbor
        if self.detectForceNeighbor(new_node, motion):
            return new_node
        else:
            return self.jump(new_node, motion)

    def detectForceNeighbor(self, node, motion):
        '''
        Detect forced neighbor of node.

        Parameters
        ----------
        node: Node
            current node
        motion: Node
            the motion that current node executes

        Return
        ----------
        flag: bool
            True if current node has forced neighbor else Flase
        '''
        x, y = node.current
        x_dir, y_dir = motion.current

        # horizontal
        if x_dir and not y_dir:
            if (x, y + 1) in self.obstacles and \
                (x + x_dir, y + 1) not in self.obstacles:
                return True
            if (x, y - 1) in self.obstacles and \
                (x + x_dir, y - 1) not in self.obstacles:
                return True
        
        # vertical
        if not x_dir and y_dir:
            if (x + 1, y) in self.obstacles and \
                (x + 1, y + y_dir) not in self.obstacles:
                return True
            if (x - 1, y) in self.obstacles and \
                (x - 1, y + y_dir) not in self.obstacles:
                return True
        
        # diagonal
        if x_dir and y_dir:
            if (x - x_dir, y) in self.obstacles and \
                (x - x_dir, y + y_dir) not in self.obstacles:
                return True
            if (x, y - y_dir) in self.obstacles and \
                (x + x_dir, y - y_dir) not in self.obstacles:
                return True
        
        return False

