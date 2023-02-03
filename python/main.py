'''
@file: main.py
@breif: application entry
@author: Winter
@update: 2023.1.13
'''
from utils import Grid, Map
from graph_search import AStar, Dijkstra, GBFS, JPS
from graph_search import DStar, LPAStar, DStarLite

from sample_search import RRT, RRTConnect, RRTStar, InformedRRT

if __name__ == '__main__':
    '''
    graph search
    '''
    # # build environment
    # start = (5, 5)
    # goal = (45, 25)
    # env = Grid(51, 31)

    # # creat planner
    # # planner = AStar(start, goal, env)
    # # planner = Dijkstra(start, goal, env)
    # # planner = GBFS(start, goal, env)
    # # planner = JPS(start, goal, env)
    # # planner = DStar(start, goal, env)
    # # planner = LPAStar(start, goal, env)
    # planner = DStarLite(start, goal, env)

    # # animation
    # planner.run()

    # ========================================================

    '''
    sample search
    '''
    # build environment
    start = (18, 8)
    goal = (37, 18)
    env = Map(51, 31)

    # creat planner
    # planner = RRT(start, goal, env, max_dist=0.5, sample_num=10000)
    # planner = RRTConnect(start, goal, env, max_dist=0.5, sample_num=10000)
    planner = RRTStar(start, goal, env, max_dist=0.5, r=10, sample_num=10000)
    # planner = InformedRRT(start, goal, env, max_dist=0.5, r=12, sample_num=1500)

    # animation
    planner.run()