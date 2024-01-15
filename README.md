<p align="center">
  <img src="assets/cover.png">
</p>

<p align="center">
    <img width="100px" height="20px" src="https://img.shields.io/badge/Ubuntu-20.04-orange?logo=Ubuntu&Ubuntu-20.04"
        alt="ubuntu" />
    <img width="100px" height="20px" src="https://img.shields.io/badge/ROS-noetic-blue?logo=ROS&ROS=noetic" alt="ROS" />
</p>

# ROS Motion Planning

**Robot Motion planning** is a computational problem that involves finding a sequence of valid configurations to move the robot from the source to the destination. Generally, it includes **Path Searching** and **Trajectory Optimization**.

* **Path Searching**: Based on path constraints, such as obstacles, to find the optimal sequence for the robot to travel from the source to the destination without any collisions.

* **Trajectory Planning**: Based on kinematics, dynamics and obstacles, it optimizes the trajectory of the motion state  from the source to the destination according to the path.

This repository provides the implementation of common **Motion Planning** algorithms. The theory analysis can be found at [motion-planning](https://blog.csdn.net/frigidwinter/category_11410243.html). Furthermore, we provide [Python](https://github.com/ai-winter/python_motion_planning) and [MATLAB](https://github.com/ai-winter/matlab_motion_planning) version.

**Your stars, forks and PRs are welcome!**

![demo.gif](./assets/demo.gif)

## Contents
- [Quick Start within 3 Minutes](#0)
- [File Tree](#1)
- [Tool Chains](#2)
- [Version](#3)
- [Papers](#4)
- [Acknowledgments](#5)
- [License](#6)
- [Maintenance](#7)

## <span id="0">0. Quick Start within 3 Minutes

*Tested on ubuntu 20.04 LTS with ROS Noetic.*

1. Install [ROS](http://wiki.ros.org/ROS/Installation) (Desktop-Full *suggested*).

2. Install git.
    ```bash
    sudo apt install git
    ```

3. Other dependence.
    ```bash
    sudo apt install python-is-python3 \
    ros-noetic-amcl \
    ros-noetic-base-local-planner \
    ros-noetic-map-server \
    ros-noetic-move-base \
    ros-noetic-navfn
    ```

4. Clone the reposity.
    ```bash
    git clone https://github.com/ai-winter/ros_motion_planning.git
    ```

5. Compile the code. 
    ```bash
    cd ros_motion_planning/
    catkin_make
    # or catkin build
    # you may need to install it by: sudo apt install python-catkin-tools
    ```

6. Execute the code.
    ```bash
    cd scripts/
    ./main.sh
    ```

    **NOTE: Modifying certain launch files does not have any effect, as they are regenerated based on the `src/user_config/user_config.yaml` by a Python script when you execute `main.sh`. Therefore, you should modify configurations in `user_config.yaml` instead of launch files.**

7. Use **2D Nav Goal** in RViz to select the goal.

8. Moving!

9.  You can use the other script to shutdown them rapidly.
    ```bash
    ./killpro.sh
    ```
10. For more details on system configuration, please refer to [Configuration](https://github.com/ai-winter/ros_motion_planning/blob/master/docs/configuration.md).


## 1. <span id="1">Documentation
Generating a doxygen documentation helps you understand the code better.

First install doxygen and graphviz.
```sh
sudo apt-get install doxygen
sudo apt-get install graphviz
```
Then enter the root directory of project and run the following command in your terminal to generate doxygen documentation.
```sh
doxygen
```
Then you can find the documentation in `./docs/html/index.html`. You can configure the doxygen settings in `./Doxyfile`.

## 02. <span id="2">Tool Chains

For the efficient operation of the motion planning system, we provide a series of user-friendly simulation tools that allow for on-demand selection of these lightweight repositories.

| Tool Version | Introduction |
|:-------:|:---------:|
|[![Status](https://img.shields.io/badge/Pedestrian-v2.0-8A2BE2?logo=iledefrancemobilites)](https://github.com/ai-winter/ros_pedestrians_simulation) | This is a Gazebo plugin for pedestians with collision property. You can construct a dynamic environment in ROS easily using plugin.
|[![Status](https://img.shields.io/badge/Path_Visual-v1.1-8A2BE2?logo=v)](https://github.com/ai-winter/path_visualization_plugins)|This repository provides a ROS-based visualization Rviz plugins for path planning and curve generation algorithms.

## <span id="3">03. Version

### Global Planner

| Planner | Version | Animation |
|:-------:|:-------:|:---------:|
|     **GBFS**     |      [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/global_planner/graph_planner/src/a_star.cpp)       |            ![gbfs_ros.gif](assets/gbfs_ros.gif)            |
|   **Dijkstra**   |      [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/global_planner/graph_planner/src/a_star.cpp)       |        ![dijkstra_ros.gif](assets/dijkstra_ros.gif)        |
|     **A\***      |      [![Status](https://img.shields.io/badge/done-v1.1-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/global_planner/graph_planner/src/a_star.cpp)       |          ![a_star_ros.gif](assets/a_star_ros.gif)          |
|     **JPS**      | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/global_planner/graph_planner/src/jump_point_search.cpp) |             ![jps_ros.gif](assets/jps_ros.gif)             |
|     **D\***      |     [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)]((https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/global_planner/graph_planner/src/d_star.cpp))      |          ![d_star_ros.gif](assets/d_star_ros.gif)          |
|    **LPA\***     |    [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)]((https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/global_planner/graph_planner/src/lpa_star.cpp))     |        ![lpa_star_ros.gif](assets/lpa_star_ros.gif)        |
|   **D\* Lite**   |   [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)]((https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/global_planner/graph_planner/src/d_star_lite.cpp))   |     ![d_star_lite_ros.gif](assets/d_star_lite_ros.gif)     |
|   **Voronoi**    |     [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)]((https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/global_planner/graph_planner/src/voronoi.cpp))     |         ![voronoi_ros.gif](assets/voronoi_ros.gif)         |
|   **Theta\***    |   [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)]((https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/global_planner/graph_planner/src/theta_star.cpp))    |      ![theta_star_ros.gif](assets/theta_star_ros.gif)      |
| **Lazy Theta\*** | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)]((https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/global_planner/graph_planner/src/lazy_theta_star.cpp)) | ![lazy_theta_star_ros.gif](assets/lazy_theta_star_ros.gif) |
| **Hybrid A\*** | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)]((https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/global_planner/graph_planner/src/hybrid_astar.cpp)) | ![hybrid_astar_ros.gif](assets/hybrid_astar_ros.gif) |
|     **RRT**      |       [![Status](https://img.shields.io/badge/done-v1.1-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/global_planner/sample_planner/src/rrt.cpp)        |             ![rrt_ros.gif](assets/rrt_ros.gif)             |
|    **RRT\***     |     [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/global_planner/sample_planner/src/rrt_star.cpp)     |        ![rrt_star_ros.gif](assets/rrt_star_ros.gif)        |
| **Informed RRT** |   [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/global_planner/sample_planner/src/informed_rrt.cpp)   |    ![informed_rrt_ros.gif](assets/informed_rrt_ros.gif)    |
| **RRT-Connect**  |   [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/global_planner/sample_planner/src/rrt_connect.cpp)    |     ![rrt_connect_ros.gif](assets/rrt_connect_ros.gif)     |

### Local Planner

| Planner | Version | Animation |
|:-------:|:-------:|:---------:|
|   **PID**   | [![Status](https://img.shields.io/badge/done-v1.1-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/local_planner/pid_planner/src/pid_planner.cpp) |           ![pid_ros.gif](assets/pid_ros.gif)            |
|   **LQR**   | [![Status](https://img.shields.io/badge/done-v1.1-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/local_planner/lqr_planner/src/lqr_planner.cpp) |           ![lqr_ros.gif](assets/lqr_ros.gif)            |
|   **DWA**   |     [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/local_planner/dwa_planner/src/dwa.cpp)     |           ![dwa_ros.gif](assets/dwa_ros.gif)            |
|   **APF**   |     [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/local_planner/apf_planner/src/apf_planner.cpp)     | ![apf_ros.gif](assets/apf_ros.gif)|
|   **RPP**   |     [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/local_planner/rpp_planner/src/rpp_planner.cpp)     | ![rpp_ros.gif](assets/rpp_ros.gif)|
|   **TEB**   |                                                                ![Status](https://img.shields.io/badge/develop-v1.0-red)                                                                 | ![Status](https://img.shields.io/badge/gif-none-yellow) |
|   **MPC**   |                                                                ![Status](https://img.shields.io/badge/develop-v1.0-red)                                                                 | ![Status](https://img.shields.io/badge/gif-none-yellow) |
| **Lattice** |                                                                ![Status](https://img.shields.io/badge/develop-v1.0-red)                                                                 | ![Status](https://img.shields.io/badge/gif-none-yellow) |

### Intelligent Algorithm

| Planner | Version | Animation |
|:-------:|:-------:|:---------:|
| **ACO** | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/global_planner/evolutionary_planner/src/aco.cpp) | ![aco_ros.gif](assets/aco_ros.gif)  |
| **GA**  | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/global_planner/evolutionary_planner/src/ga.cpp) | ![ga_ros.gif](assets/ga_ros.gif)  |
| **PSO** | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/global_planner/evolutionary_planner/src/pso.cpp) | ![pso_ros.gif](assets/pso_ros.gif) |
| **ABC** | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/gif-none-yellow) |


### Curve Generation

| Planner | Version | Animation |
|:-------:|:-------:|:---------:|
| **Polynomia** | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/curve_generation/src/polynomial_curve.cpp) | ![polynomial_curve_python.gif](assets/polynomial_curve_python.gif)
| **Bezier** | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/curve_generation/src/bezier_curve.cpp) | ![bezier_curve_python.png](assets/bezier_curve_python.png)
| **Cubic Spline** | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/curve_generation/src/cubic_spline.cpp) | ![cubic_spline_python.png](assets/cubic_spline_python.png)
| **BSpline** | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/curve_generation/src/bspline_curve.cpp) | ![bspline_curve_python.png](assets/bspline_curve_python.png) 
| **Dubins** | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/curve_generation/src/dubins_curve.cpp) | ![dubins_curve_python.png](assets/dubins_curve_python.png)
| **Reeds-Shepp** | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/core/curve_generation/src/reeds_shepp.cpp) | ![reeds_shepp_python.png](assets/reeds_shepp_python.gif)
## <span id="4">04. Papers

### Search-based Planning

* A*: [A Formal Basis for the heuristic Determination of Minimum Cost Paths](https://ieeexplore.ieee.org/document/4082128).
* JPS: [Online Graph Pruning for Pathfinding On Grid Maps](https://ojs.aaai.org/index.php/AAAI/article/view/7994).
* Lifelong Planning A*: [Lifelong Planning A*](https://www.cs.cmu.edu/~maxim/files/aij04.pdf).
* D*: [Optimal and Efficient Path Planning for Partially-Known Environments](http://web.mit.edu/16.412j/www/html/papers/original_dstar_icra94.pdf).
* D* Lite: [D* Lite](http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf).
* Theta*: [Theta*: Any-Angle Path Planning on Grids](https://www.jair.org/index.php/jair/article/view/10676), [Any-angle path planning on non-uniform costmaps](https://ieeexplore.ieee.org/abstract/document/5979769).
* Lazy Theta*: [Lazy Theta*: Any-Angle Path Planning and Path Length Analysis in 3D](https://ojs.aaai.org/index.php/AAAI/article/view/7566).
* Hybrid A*: [Practical search techniques in path planning for autonomous driving](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf)

### Sample-based Planning
* RRT: [Rapidly-Exploring Random Trees: A New Tool for Path Planning](http://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf).
* RRT-Connect: [RRT-Connect: An Efficient Approach to Single-Query Path Planning](http://www-cgi.cs.cmu.edu/afs/cs/academic/class/15494-s12/readings/kuffner_icra2000.pdf).
* RRT*: [Sampling-based algorithms for optimal motion planning](https://journals.sagepub.com/doi/abs/10.1177/0278364911406761).
* Informed RRT*: [Optimal Sampling-based Path Planning Focused via Direct Sampling of an Admissible Ellipsoidal heuristic](https://arxiv.org/abs/1404.2334).

### Evolutionary-based Planning
* ACO: [Ant Colony Optimization: A New Meta-Heuristic](http://www.cs.yale.edu/homes/lans/readings/routing/dorigo-ants-1999.pdf).
* GA: [Adaptation in Natural and Artificial Systems](https://ieeexplore.ieee.org/book/6267401)
* PSO: [Particle Swarm Optimization](https://ieeexplore.ieee.org/document/488968)


### Local Planning

* DWA: [The Dynamic Window Approach to Collision Avoidance](https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf).
* APF: [Real-time obstacle avoidance for manipulators and mobile robots](https://ieeexplore.ieee.org/document/1087247).
* RPP: [Regulated Pure Pursuit for Robot Path Tracking](https://arxiv.org/pdf/2305.20026.pdf)


### Curve Generation
* Reeds Shepp: [Optimal paths for a car that goes both forwards and backwards](https://projecteuclid.org/journals/pacific-journal-of-mathematics/volume-145/issue-2/Optimal-paths-for-a-car-that-goes-both-forwards-and/pjm/1102645450.full)
* Dubins: [On curves of minimal length with a constraint on average curvature, and with prescribed initial and terminal positions and tangents]()


## <span id="5">05. Acknowledgments
* Our robot and world models are from [
Dataset-of-Gazebo-Worlds-Models-and-Maps](https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps) and [
aws-robomaker-small-warehouse-world](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world). Thanks for these open source models sincerely.

* Pedestrians in this environment are using social force model(sfm), which comes from [https://github.com/robotics-upo/lightsfm](https://github.com/robotics-upo/lightsfm).

* A ROS costmap plugin for [dynamicvoronoi](http://www2.informatik.uni-freiburg.de/~lau/dynamicvoronoi/) presented by Boris Lau.

## <span id="6">06. License

The source code is released under [GPLv3](https://www.gnu.org/licenses/) license.

## <span id="7">07. Maintenance

Feel free to contact us if you have any question.
