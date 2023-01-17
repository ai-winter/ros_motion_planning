
# Introduction

# Install

# Version
## Global Planner

Planner      |    C++    | Python    | Matlab
------------ | --------- | --------- | -----------------
**GBFS**                 | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/913982779/ros_motion_planning/blob/master/ros/src/planner/graph_planner/src/a_star.cpp)   | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/913982779/ros_motion_planning/blob/master/python/graph_search/gbfs.py)   | ![Status](https://img.shields.io/badge/develop-v1.0-red)   |
**Dijkstra**                 | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/913982779/ros_motion_planning/blob/master/ros/src/planner/graph_planner/src/a_star.cpp)  | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/913982779/ros_motion_planning/blob/master/python/graph_search/dijkstra.py) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |
**A***                 | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/913982779/ros_motion_planning/blob/master/ros/src/planner/graph_planner/src/a_star.cpp) | ![Status](https://img.shields.io/badge/done-v1.0-brightgreen) | ![Status](https://img.shields.io/badge/develop-v1.0-red) | 
**JPS**                 | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/913982779/ros_motion_planning/blob/master/ros/src/planner/graph_planner/src/jump_point_search.cpp) | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/913982779/ros_motion_planning/blob/master/python/graph_search/jps.py) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |
**D***                 | ![Status](https://img.shields.io/badge/develop-v1.0-red) | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/913982779/ros_motion_planning/blob/master/python/graph_search/d_star.py) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |
**LPA***                 | ![Status](https://img.shields.io/badge/develop-v1.0-red) | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/913982779/ros_motion_planning/blob/master/python/graph_search/lpa_star.py) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |
**D\* Lite**                 | ![Status](https://img.shields.io/badge/develop-v1.0-red) | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)]((https://github.com/913982779/ros_motion_planning/blob/master/python/graph_search/d_star_lite.py)) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |
**RRT**                 | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/913982779/ros_motion_planning/blob/master/ros/src/planner/sample_planner/src/rrt.cpp) | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |
**RRT***                 | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/913982779/ros_motion_planning/blob/master/ros/src/planner/sample_planner/src/rrt_star.cpp) | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |
**Informed RRT**                 | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |
**RRT-Connected**                 | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |

## Local Planner
| Planner | C++                                                      | Python                                                   | Matlab                                                   |
| ------- | -------------------------------------------------------- | -------------------------------------------------------- | -------------------------------------------------------- |
| **DWA** | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |

## Intelligent Algorithm

| Planner | C++                                                      | Python                                                   | Matlab                                                   |
| ------- | -------------------------------------------------------- | -------------------------------------------------------- | -------------------------------------------------------- |
| **ACO** | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |
| **GA**  | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |


# Papers
## Search-base Planning
* [A*: ](https://ieeexplore.ieee.org/document/4082128) A Formal Basis for the heuristic Determination of Minimum Cost Paths
* [JPS:](https://ojs.aaai.org/index.php/AAAI/article/view/7994) Online Graph Pruning for Pathfinding On Grid Maps
* [Lifelong Planning A*: ](https://www.cs.cmu.edu/~maxim/files/aij04.pdf) Lifelong Planning A*
* [D*: ](http://web.mit.edu/16.412j/www/html/papers/original_dstar_icra94.pdf) Optimal and Efficient Path Planning for Partially-Known Environments
* [D* Lite: ](http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf) D* Lite

# Update
| Date      | Update                                                                                                                                                                        |
| --------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 2023.1.13 | cost of motion nodes is set to `NEUTRAL_COST`, which is unequal to that of heuristics, so there is no difference between A* and Dijkstra. This bug has been solved in A* v1.1 |

# Acknowledgment
* Our robot and world models are from [
Dataset-of-Gazebo-Worlds-Models-and-Maps](https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps) and [
aws-robomaker-small-warehouse-world](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world). Thanks for these open source models sincerely.