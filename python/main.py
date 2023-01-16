'''
@file: main.py
@breif: application entry
@author: Winter
@update: 2023.1.13
'''
from utils import Env
from graph_search import AStar, Dijkstra, GBFS, JPS, DStar

if __name__ == '__main__':
    # build environment
    start = (5, 5)
    goal = (45, 25)
    env = Env(51, 31)

    # creat planner
    # planner = AStar(start, goal, env)
    # planner = Dijkstra(start, goal, env)
    # planner = GBFS(start, goal, env)
    # planner = JPS(start, goal, env)
    planner = DStar(start, goal, env)

    # animation
    planner.run()