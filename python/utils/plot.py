"""
Plot tools 2D
@author: huiming zhou
"""

import matplotlib.pyplot as plt
from .env import Env, Node


class Plot:
    def __init__(self, start, goal, env: Env):
        self.xI, self.xG = start, goal
        self.start = Node(start, start, 0, 0)
        self.goal = Node(goal, goal, 0, 0)
        self.env = env
        self.obs = self.env.obstacle_map

    def update_obs(self, obs):
        self.obs = obs

    def animation(self, path, expand, name):
        self.plotGrid(name)
        self.plotExpand(expand)
        self.plotPath(path)
        plt.show()

    def plotGrid(self, name):
        obs_x = [x[0] for x in self.obs]
        obs_y = [x[1] for x in self.obs]
        plt.plot(self.start.current[0], self.start.current[1], marker="s", color="#ff0000")
        plt.plot(self.goal.current[0], self.goal.current[1], marker="s", color="#1155cc")
        plt.plot(obs_x, obs_y, "sk")
        plt.title(name)
        plt.axis("equal")

    def plotExpand(self, expand):
        if self.start in expand:
            expand.remove(self.start)
        if self.goal in expand:
            expand.remove(self.goal)

        count = 0
        for x in expand:
            count += 1
            plt.plot(x.current[0], x.current[1], color="#dddddd", marker='s')
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])
            if count < len(expand) / 3:         length = 20
            elif count < len(expand) * 2 / 3:   length = 30
            else:                               length = 40
            if count % length == 0:             plt.pause(0.001)
        plt.pause(0.01)

    def plotPath(self, path):
        path_x = [path[i][0] for i in range(len(path))]
        path_y = [path[i][1] for i in range(len(path))]
        plt.plot(path_x, path_y, linewidth='2', color='#13ae00')
        plt.plot(self.start.current[0], self.start.current[1], marker="s", color="#ff0000")
        plt.plot(self.goal.current[0], self.goal.current[1], marker="s", color="#1155cc")


    @staticmethod
    def color_list():
        cl_v = ['silver',
                'wheat',
                'lightskyblue',
                'royalblue',
                'slategray']
        cl_p = ['gray',
                'orange',
                'deepskyblue',
                'red',
                'm']
        return cl_v, cl_p

    @staticmethod
    def color_list_2():
        cl = ['silver',
              'steelblue',
              'dimgray',
              'cornflowerblue',
              'dodgerblue',
              'royalblue',
              'plum',
              'mediumslateblue',
              'mediumpurple',
              'blueviolet',
              ]
        return cl
