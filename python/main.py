'''
@file: main.py
@breif: application entry
@author: Winter
@update: 2023.1.13
'''
from utils import Env, Plot
from graph_search import AStar, Dijkstra, GBFS, JPS

if __name__ == '__main__':
    # build environment
    start = (5, 5)
    goal = (45, 25)
    env = Env(51, 31)

    # creat plot handler
    plot = Plot(start, goal, env)

    # creat planner
    # planner = AStar(start, goal, env)
    # planner = Dijkstra(start, goal, env)
    # planner = GBFS(start, goal, env)
    planner = JPS(start, goal, env)

    # animation
    (cost, path), expand = planner.plan()
    plot.animation(path, expand, str(planner), cost)
