
# Introduction

`Motion planning` plans the state sequence of the robot without conflict between the start and goal. 

`Motion planning` mainly includes `Path planning` and `Trajectory planning`.

* `Path Planning`: It's based on path constraints (such as obstacles), planning the optimal path sequence for the robot to travel without conflict between the start and goal.
* `Trajectory planning`: It plans the motion state to approach the global path based on kinematics, dynamics constraints and path sequence.

This repository provides the implement of common `Motion planning` algorithm, welcome your star & fork & PR.

The theory analysis can be found at [motion-planning](https://blog.csdn.net/frigidwinter/category_11410243.html)

# Quick Start

For ROS C++ version, execute the following commands

```shell
cd ./ros
catkin_make
source ./devel/setup.bash
roslaunch sim_env main.launch global_planner:=d_star local_planner:=dwa
```

For python version, open `./python/main.py` and select the algorithm, for example

```python
if __name__ == '__main__':
    '''
    sample search
    '''
    # build environment
    start = (18, 8)
    goal = (37, 18)
    env = Map(51, 31)

    planner = InformedRRT(start, goal, env, max_dist=0.5, r=12, sample_num=1500)

    # animation
    planner.run()
```

For matlab version, open `./matlab/simulation_global.mlx` or `./matlab/simulation_local.mlx` and select the algorithm, for example

```matlab
clear all;
clc;

% load environment
load("gridmap_20x20_scene1.mat");
map_size = size(grid_map);
G = 1;

% start and goal
start = [3, 2];
goal = [18, 29];

% planner
planner_name = "rrt";

planner = str2func(planner_name);
[path, flag, cost, expand] = planner(grid_map, start, goal);

% visualization
clf;
hold on

% plot grid map
plot_grid(grid_map);
% plot expand zone
plot_expand(expand, map_size, G, planner_name);
% plot path
plot_path(path, G);
% plot start and goal
plot_square(start, map_size, G, "#f00");
plot_square(goal, map_size, G, "#15c");
% title
title([planner_name, "cost:" + num2str(cost)]);

hold off
```

# Version
## Global Planner

Planner      |    C++    | Python    | Matlab
------------ | --------- | --------- | -----------------
**GBFS**                 | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/motion_planning/blob/master/ros/src/planner/graph_planner/src/a_star.cpp)   | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/motion_planning/blob/master/python/graph_search/gbfs.py)   | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/motion_planning/blob/master/matlab/graph_search/gbfs.m)   |
**Dijkstra**                 | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/motion_planning/blob/master/ros/src/planner/graph_planner/src/a_star.cpp)  | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/motion_planning/blob/master/python/graph_search/dijkstra.py) | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/motion_planning/blob/master/matlab/graph_search/dijkstra.m) |
**A***                 | [![Status](https://img.shields.io/badge/done-v1.1-brightgreen)](https://github.com/ai-winter/motion_planning/blob/master/ros/src/planner/graph_planner/src/a_star.cpp) | ![Status](https://img.shields.io/badge/done-v1.0-brightgreen) | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/motion_planning/blob/master/matlab/graph_search/a_star.m) | 
**JPS**                 | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/motion_planning/blob/master/ros/src/planner/graph_planner/src/jump_point_search.cpp) | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/motion_planning/blob/master/python/graph_search/jps.py) | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/motion_planning/blob/master/matlab/graph_search/jps.m) |
**D***                 | ![Status](https://img.shields.io/badge/develop-v1.0-red) | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/motion_planning/blob/master/python/graph_search/d_star.py) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |
**LPA***                 | ![Status](https://img.shields.io/badge/develop-v1.0-red) | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/motion_planning/blob/master/python/graph_search/lpa_star.py) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |
**D\* Lite**                 | ![Status](https://img.shields.io/badge/develop-v1.0-red) | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)]((https://github.com/ai-winter/motion_planning/blob/master/python/graph_search/d_star_lite.py)) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |
**RRT**                 | [![Status](https://img.shields.io/badge/done-v1.1-brightgreen)](https://github.com/ai-winter/motion_planning/blob/master/ros/src/planner/sample_planner/src/rrt.cpp) | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/motion_planning/blob/master/python/sample_search/rrt.py) | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/motion_planning/blob/master/matlab/sample_search/rrt.m) |
**RRT***                 | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/motion_planning/blob/master/ros/src/planner/sample_planner/src/rrt_star.cpp) | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/motion_planning/blob/master/python/sample_search/rrt_star.py) | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/motion_planning/blob/master/matlab/sample_search/rrt_star.m) |
**Informed RRT**                 | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/motion_planning/blob/master/ros/src/planner/sample_planner/src/informed_rrt.cpp) | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/motion_planning/blob/master/python/sample_search/informed_rrt.py) | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/motion_planning/blob/master/matlab/sample_search/informed_rrt.m) |
**RRT-Connect**                 | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/motion_planning/blob/master/ros/src/planner/sample_planner/src/rrt_connect.cpp) | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/motion_planning/blob/master/python/sample_search/rrt_connect.py) | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/motion_planning/blob/master/matlab/sample_search/rrt_connect.m) |

## Local Planner
| Planner | C++                                                      | Python                                                   | Matlab                                                   |
| ------- | -------------------------------------------------------- | -------------------------------------------------------- | -------------------------------------------------------- |
| **PID** | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |
| **APF** | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |
| **DWA** | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/motion_planning/blob/master/ros/src/planner/local_planner/dwa_planner/src/dwa.cpp) | ![Status](https://img.shields.io/badge/develop-v1.0-red) | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/motion_planning/blob/master/matlab/local_planner/dwa.m) |
| **TEB** | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |
| **MPC** | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |
| **Lattice** | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |

## Intelligent Algorithm

| Planner | C++                                                      | Python                                                   | Matlab                                                   |
| ------- | -------------------------------------------------------- | -------------------------------------------------------- | -------------------------------------------------------- |
| **ACO** | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |
| **GA**  | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |
| **PSO**  | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |
| **ABC** | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |


# Animation

## Global Planner

Planner      |    C++    | Python    | Matlab
------------ | --------- | --------- | -----------------
**GBFS**                 | ![Status](https://img.shields.io/badge/gif-none-yellow)   | ![gbfs_python.png](gif/gbfs_python.png)   | ![gbfs_matlab.png](gif/gbfs_matlab.png)  |
**Dijkstra**                 | ![Status](https://img.shields.io/badge/gif-none-yellow)  |![dijkstra_python.png](gif/dijkstra_python.png) | ![dijkstra_matlab.png](gif/dijkstra_matlab.png) |
**A***                 | ![Status](https://img.shields.io/badge/gif-none-yellow) | ![a_star_python.png](gif/a_star_python.png) | ![a_star.png](gif/a_star_matlab.png)| 
**JPS**                 | ![Status](https://img.shields.io/badge/gif-none-yellow) |![jps_python.png](gif/jps_python.png) | ![jps_matlab.png](gif/jps_matlab.png) |
**D***                 | ![Status](https://img.shields.io/badge/gif-none-yellow) | ![d_star_python.png](gif/d_star_python.png)|![Status](https://img.shields.io/badge/gif-none-yellow) |
**LPA***                 | ![Status](https://img.shields.io/badge/gif-none-yellow) | ![lpa_star_python.png](gif/lpa_star_python.png) | ![Status](https://img.shields.io/badge/gif-none-yellow) |
**D\* Lite**                 | ![Status](https://img.shields.io/badge/gif-none-yellow) | ![d_star_lite_python.png](gif/d_star_lite_python.png) |![Status](https://img.shields.io/badge/gif-none-yellow) |
**RRT**                 | ![rrt_ros.gif](gif/rrt_ros.gif) | ![rrt_python.png](gif/rrt_python.png) | ![rrt_matlab.png](gif/rrt_matlab.png) |
**RRT***                 | ![Status](https://img.shields.io/badge/gif-none-yellow)| ![rrt_star_python.png](gif/rrt_star_python.png) | ![rrt_star_matlab.png](gif/rrt_star_matlab.png)|
**Informed RRT**                 | ![Status](https://img.shields.io/badge/gif-none-yellow) | ![informed_rrt_python.png](gif/informed_rrt_python.png) | ![informed_rrt_matlab.png](gif/informed_rrt_matlab.png) |
**RRT-Connect**                 | ![Status](https://img.shields.io/badge/gif-none-yellow) | ![rrt_connect_python.png](gif/rrt_connect_python.png) | ![rrt_connect_matlab.png](gif/rrt_connect_matlab.png) |


## Local Planner
| Planner | C++                                                      | Python                                                   | Matlab                                                   |
| ------- | -------------------------------------------------------- | -------------------------------------------------------- | -------------------------------------------------------- |
| **PID** | ![Status](https://img.shields.io/badge/gif-none-yellow) | ![Status](https://img.shields.io/badge/gif-none-yellow) | ![Status](https://img.shields.io/badge/gif-none-yellow) |
| **APF** | ![Status](https://img.shields.io/badge/gif-none-yellow) | ![Status](https://img.shields.io/badge/gif-none-yellow) | ![Status](https://img.shields.io/badge/gif-none-yellow) |
| **DWA** | ![Status](https://img.shields.io/badge/gif-none-yellow) | ![Status](https://img.shields.io/badge/gif-none-yellow) | ![dwa_matlab.gif](gif/dwa_matlab.gif) | 


# Papers
## Search-based Planning
* [A*: ](https://ieeexplore.ieee.org/document/4082128) A Formal Basis for the heuristic Determination of Minimum Cost Paths
* [JPS:](https://ojs.aaai.org/index.php/AAAI/article/view/7994) Online Graph Pruning for Pathfinding On Grid Maps
* [Lifelong Planning A*: ](https://www.cs.cmu.edu/~maxim/files/aij04.pdf) Lifelong Planning A*
* [D*: ](http://web.mit.edu/16.412j/www/html/papers/original_dstar_icra94.pdf) Optimal and Efficient Path Planning for Partially-Known Environments
* [D* Lite: ](http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf) D* Lite

## Sample-based Planning
* [RRT: ](http://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf) Rapidly-Exploring Random Trees: A New Tool for Path Planning
* [RRT-Connect: ](http://www-cgi.cs.cmu.edu/afs/cs/academic/class/15494-s12/readings/kuffner_icra2000.pdf) RRT-Connect: An Efficient Approach to Single-Query Path Planning
* [RRT*: ](https://journals.sagepub.com/doi/abs/10.1177/0278364911406761) Sampling-based algorithms for optimal motion planning
* [Informed RRT*: ](https://arxiv.org/abs/1404.2334) Optimal Sampling-based Path Planning Focused via Direct Sampling of an Admissible Ellipsoidal heuristic

## Local Planning

* [DWA: ](https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf) The Dynamic Window Approach to Collision Avoidance

# Update
| Date      | Update                                                                                                                                                                        |
| --------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 2023.1.13 | cost of motion nodes is set to `NEUTRAL_COST`, which is unequal to that of heuristics, so there is no difference between A* and Dijkstra. This bug has been solved in A* C++ v1.1 |
|2023.1.18| update RRT C++ v1.1, adding heuristic judgement when generating random nodes

# Acknowledgment
* Our robot and world models are from [
Dataset-of-Gazebo-Worlds-Models-and-Maps](https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps) and [
aws-robomaker-small-warehouse-world](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world). Thanks for these open source models sincerely.
* Our visualization and animation framework of Python Version refers to [https://github.com/zhm-real/PathPlanning](https://github.com/zhm-real/PathPlanning). Thanks sincerely.