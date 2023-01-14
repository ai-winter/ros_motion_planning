
# Introduction

# Install

# Version
## Global Planner

Planner      |    C++    | Python    | Matlab
------------ | --------- | --------- | -----------------
**GBFS**                 | ![](https://img.shields.io/badge/done-v1.0-brightgreen)   | ![Status](https://img.shields.io/badge/done-v1.0-brightgreen)   | ![Status](https://img.shields.io/badge/develop-v1.0-red)   |
**Dijkstra**                 | ![Status](https://img.shields.io/badge/done-v1.0-brightgreen)  | ![Status](https://img.shields.io/badge/done-v1.0-brightgreen) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |
**A***                 | ![Status](https://img.shields.io/badge/done-v1.1-brightgreen) | ![Status](https://img.shields.io/badge/done-v1.0-brightgreen) | ![Status](https://img.shields.io/badge/develop-v1.0-red) | 
**JPS**                 | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |
**D***                 | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |
**LPA***                 | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |
**D\* Lite**                 | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |
**RRT**                 | ![Status](https://img.shields.io/badge/done-v1.0-brightgreen) | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |
**RRT***                 | ![Status](https://img.shields.io/badge/done-v1.0-brightgreen) | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |
**Informed RRT**                 | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |
**RRT-Connected**                 | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/develop-v1.0-red) |

# Papers
## Search-base Planning
* [A*: ](https://ieeexplore.ieee.org/document/4082128) A Formal Basis for the heuristic Determination of Minimum Cost Paths

# Update
Date         |    Update  
------------ | ---------
2023.1.13    | cost of motion nodes is set to `NEUTRAL_COST`, which is unequal to that of heuristics, so there is no difference between A* and Dijkstra. This bug has been solved in A* v1.1 