![cover](assets/cover.png)

<p align="center">
    <img width="100px" height="20px" src="https://img.shields.io/badge/Ubuntu-20.04-orange?logo=Ubuntu&Ubuntu-20.04"
        alt="ubuntu" />
    <img width="100px" height="20px" src="https://img.shields.io/badge/ROS-noetic-blue?logo=ROS&ROS=noetic" alt="ROS" />
</p>

# ROS Motion Planning

**Motion planning** is a computational problem to find a sequence of valid configurations that moves the object from the source to destination. Generally, it includes **Path Searching** and **Trajectory Optimization**.

* **Path Searching**: Based on path constraints, e.g. obstacles, it searches an optimal path sequence for the robot to travel without conflicts between source and destination.

* **Trajectory Planning**: Based on the kinematics, dynamics and obstacles, it optimizes a motion state trajectory from the source to destination according to the path sequence.

This repository provides the implementation of common **Motion Planning** algorithms. The theory analysis can be found at [motion-planning](https://blog.csdn.net/frigidwinter/category_11410243.html).
Furthermore, we provide [Python](https://github.com/ai-winter/python_motion_planning) and [MATLAB](https://github.com/ai-winter/matlab_motion_planning) version.


**Your stars, forks and PRs are welcome!**

![demo.gif](./assets/demo.gif)

## Table of Contents
- [Quick Start within 3 Minutes](#0)
- [File Tree](#1)
- [Dynamic Configuration](#2)
- [Version](#3)
- [Papers](#4)
- [Application on a Real Robot](#5)
- [Important Updates](#6)
- [Acknowledgments](#7)
- [License](#8)
- [Maintenance](#9)

## <span id="0">0. Quick Start within 3 Minutes

*Tested on ubuntu 20.04 LTS with ROS Noetic.*

1. Install [ROS](http://wiki.ros.org/ROS/Installation) (Desktop-Full *suggested*).

2. Install git.

    ```bash
    sudo apt install git
    ```

3. Clone this reposity.

    ```bash
    cd <your_workspace>/
    git clone https://github.com/ai-winter/ros_motion_planning.git
    ```

4. Other dependence.

    ```bash
    sudo apt install python-is-python3
    ros-noetic-amcl \
    ros-noetic-base-local-planner \
    ros-noetic-map-server \
    ros-noetic-move-base \
    ros-noetic-navfn
    ```

5. Compile the code.

   ```bash
   cd ros_motion_planning/
   catkin_make
   ```

6. Run the scripts in `./src/sim_env/scripts/`, i.e.

    ```bash
    cd ./src/sim_env/scripts/
    ./main.sh
    ```

    **NOTE: Changing some launch files DOES NOT work, because some of them are re-generated according to the `./src/user_config/user_config.yaml` by a python script when you run `main.sh`. Therefore, you should change configurations in `user_config.yaml` instead of launch files.**

7. Use **2D Nav Goal** to select the goal.

8. Moving!

## 1. <span id="1">File Tree

The file structure is shown below.

```
ros_motion_planner
├── assets
└── src
    ├── planner
    │   ├── global_planner
    │   ├── local_planner
    │   └── utils
    ├── sim_env             # simulation environment
    │   ├── config
    │   ├── launch
    │   ├── maps
    │   ├── meshes
    │   ├── models
    │   ├── rviz
    │   ├── scripts
    │   ├── urdf
    │   └── worlds
    ├── third_party
    │   ├── dynamic_rviz_config
    │   ├── dynamic_xml_config
    │   ├── gazebo_plugins
    │   └── rviz_plugins
    └── user_config         # user configure file
```

## 02. <span id="2">Dynamic Configuration

In this reposity, you can simply change configs through modifing the `./src/user_config/user_config.yaml`. When you run `./main.sh`, our python script will re-generated `.launch`, `.world` and so on, according to your configs in that file.

Below is an example of `user_config.yaml`

```yaml
map: "warehouse"
world: "warehouse"
rviz_file: "sim_env.rviz"

robots_config:
  - robot1_type: "turtlebot3_burger"
    robot1_global_planner: "astar"
    robot1_local_planner: "dwa"
    robot1_x_pos: "0.0"
    robot1_y_pos: "0.0"
    robot1_z_pos: "0.0"
    robot1_yaw: "-1.57"
  - robot2_type: "turtlebot3_burger"
    robot2_global_planner: "jps"
    robot2_local_planner: "pid"
    robot2_x_pos: "-5.0"
    robot2_y_pos: "-7.5"
    robot2_z_pos: "0.0"
    robot2_yaw: "0.0"

plugins:
  pedestrians: "pedestrian_config.yaml"
  obstacles: "obstacles_config.yaml"
```

Explanation:

- `map`: static map，located in `src/sim_env/map/`, if `map: ""`, map_server will not publish map message which often used in SLAM.

- `world`: gazebo world，located in `src/sim_env/worlds/`, if `world: ""`, Gazebo will be disabled which often used in real world.

- `rviz_file`: RVIZ configure, automatically generated if `rviz_file` is not set.

- `robots_config`: robotic configuration.

  - `type`: robotic type，such as *`turtlebot3_burger`, `turtlebot3_waffle` and `turtlebot3_waffle_pi`*.

  - `global_planner`: global algorithm, details in Section `Version`.

  - `local_planner`: local algorithm, details in Section `Version`.

  - `xyz_pos and yaw`: initial pose.

- `plugins`: other applications using in simulation
  - `pedestrians`: configure file to add dynamic obstacles(e.g. pedestrians).
  - `obstacles`: configure file to add static obstacles.

For *pedestrians* and *obstacles* configuration files, the examples are shown below

```yaml
## pedestrians_config.yaml

# sfm algorithm configure
social_force:
  animation_factor: 5.1
  # only handle pedestrians within `people_distance`
  people_distance: 6.0
  # weights of social force model
  goal_weight: 2.0
  obstacle_weight: 80.0
  social_weight: 15
  group_gaze_weight: 3.0
  group_coh_weight: 2.0
  group_rep_weight: 1.0

# pedestrians setting
pedestrians:
  update_rate: 5
  ped_property:
    - name: human_1
      pose: 5 -2 1 0 0 1.57
      velocity: 0.9
      radius: 0.4
      cycle: true
      time_delay: 5
      ignore:
        model_1: ground_plane
        model_2: turtlebot3_waffle
      trajectory:
        goal_point_1: 5 -2 1 0 0 0
        goal_point_2: 5 2 1 0 0 0
    - name: human_2
      pose: 6 -3 1 0 0 0
      velocity: 1.2
      radius: 0.4
      cycle: true
      time_delay: 3
      ignore:
        model_1: ground_plane
        model_2: turtlebot3_waffle
      trajectory:
        goal_point_1: 6 -3 1 0 0 0
        goal_point_2: 6 4 1 0 0 0
```
Explanation:


- `social_force`: The weight factors that modify the navigation behavior. See the [Social Force Model](https://github.com/robotics-upo/lightsfm) for further information.
- `pedestrians/update_rate`: Update rate of pedestrains presentation. The higher `update_rate`, the more sluggish the environment becomes.
- `pedestrians/ped_property`: Pedestrians property configuration.
  - `name`: The id for each human.
  - `pose`: The initial pose for each human.
  - `velocity`: Maximum velocity (*m/s*) for each human.
  - `radius`: Approximate radius of the human's body (m).
  - `cycle`: If *true*, the actor will start the goal point sequence when the last goal point is reached.
  - `time_delay`: This is time in seconds to wait before starting the human motion.
  - `ignore_obstacles`: All the models that must be ignored as obstacles, must be indicated here. The other actors in the world are included automatically.
  - `trajectory`. The list of goal points that the actor must reach must be indicated here. The goals will be post into social force model.

```yaml
## obstacles_config.yaml 

# static obstacles
obstacles:
  - type: BOX
    pose: 5 2 0 0 0 0
    color: Grey
    props:
      m: 1.00
      w: 0.25
      d: 0.50
      h: 0.80
```
Explanation:
- `type`: model type of specific obstacle, *Optional: `BOX`, `CYLINDER` or `SPHERE`*
- `pose`: fixed pose of the obstacle
- `color`: color of the obstacle
- `props`: property of the obstacle
  - `m`: mass
  - `w`: width
  - `d`: depth
  - `h`: height
  - `r`: radius



## <span id="3">03. Version

### Global Planner

|     Planner      |                                                                                             Version                                                                                              |                       Animation                       |
|:----------------:|:------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|:-----------------------------------------------------:|
|     **GBFS**     |      [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/planner/global_planner/graph_planner/src/a_star.cpp)       |         ![gbfs_ros.gif](assets/gbfs_ros.gif)          |
|   **Dijkstra**   |      [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/planner/global_planner/graph_planner/src/a_star.cpp)       |     ![dijkstra_ros.gif](assets/dijkstra_ros.gif)      |
|     **A\***      |      [![Status](https://img.shields.io/badge/done-v1.1-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/planner/global_planner/graph_planner/src/a_star.cpp)       |       ![a_star_ros.gif](assets/a_star_ros.gif)        |
|     **JPS**      | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/planner/global_planner/graph_planner/src/jump_point_search.cpp) |          ![jps_ros.gif](assets/jps_ros.gif)           |
|     **D\***      |     [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)]((https://github.com/ai-winter/ros_motion_planning/blob/master/src/planner/global_planner/graph_planner/src/d_star.cpp))      |       ![d_star_ros.gif](assets/d_star_ros.gif)        |
|    **LPA\***     |    [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)]((https://github.com/ai-winter/ros_motion_planning/blob/master/src/planner/global_planner/graph_planner/src/lpa_star.cpp))     |     ![lpa_star_ros.gif](assets/lpa_star_ros.gif)      |
|   **D\* Lite**   |   [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)]((https://github.com/ai-winter/ros_motion_planning/blob/master/src/planner/global_planner/graph_planner/src/d_star_lite.cpp))   |  ![d_star_lite_ros.gif](assets/d_star_lite_ros.gif)   |
|   **Voronoi**    |     [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)]((https://github.com/ai-winter/ros_motion_planning/blob/master/src/planner/global_planner/graph_planner/src/voronoi.cpp))     |      ![voronoi_ros.gif](assets/voronoi_ros.gif)       |
|   **Theta\***    |   [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)]((https://github.com/ai-winter/ros_motion_planning/blob/master/src/planner/global_planner/graph_planner/src/theta_star.cpp))    |   ![Status](https://img.shields.io/badge/gif-none-yellow)    |
| **Lazy Theta\*** |   [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)]((https://github.com/ai-winter/ros_motion_planning/blob/master/src/planner/global_planner/graph_planner/src/lazy_theta_star.cpp))    | ![Status](https://img.shields.io/badge/gif-none-yellow) |
|     **RRT**      |       [![Status](https://img.shields.io/badge/done-v1.1-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/planner/global_planner/sample_planner/src/rrt.cpp)        |          ![rrt_ros.gif](assets/rrt_ros.gif)           |
|    **RRT\***     |     [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/planner/global_planner/sample_planner/src/rrt_star.cpp)     |     ![rrt_star_ros.gif](assets/rrt_star_ros.gif)      |
| **Informed RRT** |   [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/planner/global_planner/sample_planner/src/informed_rrt.cpp)   | ![informed_rrt_ros.gif](assets/informed_rrt_ros.gif)  |
| **RRT-Connect**  |   [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/planner/global_planner/sample_planner/src/rrt_connect.cpp)    |  ![rrt_connect_ros.gif](assets/rrt_connect_ros.gif)   |

### Local Planner

|   Planner   |                                                                                         Version                                                                                         |                        Animation                        |
| :---------: | :-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------: | :-----------------------------------------------------: |
|   **PID**   | [![Status](https://img.shields.io/badge/done-v1.1-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/planner/local_planner/pid_planner/src/pid_planner.cpp) |           ![pid_ros.gif](assets/pid_ros.gif)            |
|   **APF**   |                                                                ![Status](https://img.shields.io/badge/develop-v1.0-red)                                                                 | ![Status](https://img.shields.io/badge/gif-none-yellow) |
|   **DWA**   |     [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/planner/local_planner/dwa_planner/src/dwa.cpp)     |           ![dwa_ros.gif](assets/dwa_ros.gif)            |
|   **TEB**   |                                                                ![Status](https://img.shields.io/badge/develop-v1.0-red)                                                                 | ![Status](https://img.shields.io/badge/gif-none-yellow) |
|   **MPC**   |                                                                ![Status](https://img.shields.io/badge/develop-v1.0-red)                                                                 | ![Status](https://img.shields.io/badge/gif-none-yellow) |
| **Lattice** |                                                                ![Status](https://img.shields.io/badge/develop-v1.0-red)                                                                 | ![Status](https://img.shields.io/badge/gif-none-yellow) |

### Intelligent Algorithm

| Planner |                         Version                          |                        Animation                        |
| :-----: | :------------------------------------------------------: | :-----------------------------------------------------: |
| **ACO** | [![Status](https://img.shields.io/badge/done-v1.0-brightgreen)](https://github.com/ai-winter/ros_motion_planning/blob/master/src/planner/global_planner/evolutionary_planner/src/aco.cpp) | ![aco_ros.gif](assets/aco_ros.gif)  |
| **GA**  | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/gif-none-yellow) |
| **PSO** | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/gif-none-yellow) |
| **ABC** | ![Status](https://img.shields.io/badge/develop-v1.0-red) | ![Status](https://img.shields.io/badge/gif-none-yellow) |

## <span id="4">04. Papers

### Search-based Planning

* A*: [A Formal Basis for the heuristic Determination of Minimum Cost Paths](https://ieeexplore.ieee.org/document/4082128).
* JPS: [Online Graph Pruning for Pathfinding On Grid Maps](https://ojs.aaai.org/index.php/AAAI/article/view/7994).
* Lifelong Planning A*: [Lifelong Planning A*](https://www.cs.cmu.edu/~maxim/files/aij04.pdf).
* D*: [Optimal and Efficient Path Planning for Partially-Known Environments](http://web.mit.edu/16.412j/www/html/papers/original_dstar_icra94.pdf).
* D* Lite: [D* Lite](http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf).
* Theta*: [Theta*: Any-Angle Path Planning on Grids](https://www.jair.org/index.php/jair/article/view/10676), [Any-angle path planning on non-uniform costmaps](https://ieeexplore.ieee.org/abstract/document/5979769).
* Lazy Theta*: [Lazy Theta*: Any-Angle Path Planning and Path Length Analysis in 3D](https://ojs.aaai.org/index.php/AAAI/article/view/7566).

### Sample-based Planning
* RRT: [Rapidly-Exploring Random Trees: A New Tool for Path Planning](http://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf).
* RRT-Connect: [RRT-Connect: An Efficient Approach to Single-Query Path Planning](http://www-cgi.cs.cmu.edu/afs/cs/academic/class/15494-s12/readings/kuffner_icra2000.pdf).
* RRT*: [Sampling-based algorithms for optimal motion planning](https://journals.sagepub.com/doi/abs/10.1177/0278364911406761).
* Informed RRT*: [Optimal Sampling-based Path Planning Focused via Direct Sampling of an Admissible Ellipsoidal heuristic](https://arxiv.org/abs/1404.2334).

### Evolutionary-based Planning
* ACO: [Ant Colony Optimization: A New Meta-Heuristic](http://www.cs.yale.edu/homes/lans/readings/routing/dorigo-ants-1999.pdf)

### Local Planning

* DWA: [The Dynamic Window Approach to Collision Avoidance](https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf).

## <span id="5">05. Application on a Real Robot

> **In a word, compile our repository and make it visible to the robot, and then replace it just like other planner in `ROS navigation`.**

### Example

We use another [gazebo simulation](https://github.com/ZhanyuGuo/ackermann_ws) as an example, like we have a robot which has the capacity of localization, mapping and navigation (*using move_base*).

1. Download and compile this repository.

    ```bash
    cd <your_workspace>/
    git clone https://github.com/ai-winter/ros_motion_planning.git

    cd ros_motion_planning/
    catkin_make
    ```

2. Download and compile the 'real robot' software.

    ```bash
    cd <your_workspace>/
    git clone https://github.com/ZhanyuGuo/ackermann_ws.git

    cd ackermann_ws/
    # ---- IMPORTANT HERE, reasons in NOTE ----
    source <your_workspace>/ros_motion_planning/devel/setup.bash

    catkin_make
    ```

    **NOTE: Sourcing other workspaces before `catkin_make` will make the current `setup.bash` contain former sourced workspaces, i.e. they are also included when you only source this current workspace later.**
    
    **REMEMBER: Remove the old `build/` and `devel/` of current workspace before doing this, otherwise it will not work.**

3. Change the **base_global_planner** and **base_local_planner** in real robot's `move_base` as you need.
   
    ```xml
    <?xml version="1.0"?>
    <launch>
        <!-- something else ... -->
        <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
            <!-- something else ... -->

            <!-- Params -->
            <!-- for graph_planner -->
            <rosparam file="$(find sim_env)/config/planner/graph_planner_params.yaml" command="load" />
            <!-- for sample_planner -->
            <rosparam file="$(find sim_env)/config/planner/sample_planner_params.yaml" command="load" />
            <!-- for dwa_planner -->
            <rosparam file="$(find sim_env)/config/planner/dwa_planner_params.yaml" command="load" />
            <!-- for pid_planner -->
            <rosparam file="$(find sim_env)/config/planner/pid_planner_params.yaml" command="load" />

            <!-- Default Global Planner -->
            <!-- <param name="base_global_planner" value="global_planner/GlobalPlanner" /> -->
            <!-- GraphPlanner -->
            <param name="base_global_planner" value="graph_planner/GraphPlanner" />
            <!-- options: a_star, jps, gbfs, dijkstra, d_star, lpa_star, d_star_lite -->
            <param name="GraphPlanner/planner_name" value="a_star" />
            <!-- SamplePlanner -->
            <!-- <param name="base_global_planner" value="sample_planner/SamplePlanner" /> -->
            <!-- options: rrt, rrt_star, informed_rrt, rrt_connect -->
            <!-- <param name="SamplePlanner/planner_name" value="rrt_star" /> -->

            <!-- Default Local Planner -->
            <!-- <param name="base_local_planner" value="teb_local_planner/TebLocalPlannerROS" /> -->
            <param name="base_local_planner" value="pid_planner/PIDPlanner" />
            <!-- <param name="base_local_planner" value="dwa_planner/DWAPlanner" /> -->

            <!-- something else ... -->
        </node>
        <!-- something else ... -->
    </launch>
    ```

4. Run! But maybe there are still some details that you have to deal with...

## <span id="6">06. Important Updates
|   Date    | Update                                                                                                                                                                            |
| :-------: | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 2023.1.13 | cost of motion nodes is set to `NEUTRAL_COST`, which is unequal to that of heuristics, so there is no difference between A* and Dijkstra. This bug has been solved in A* C++ v1.1 |
| 2023.1.18 | update RRT C++ v1.1, adding heuristic judgement when generating random nodes                                                                                                      |
| 2023.2.25 | update PID C++ v1.1, making desired theta the weighted combination of theta error and theta on the trajectory                                                                     |
| 2023.3.16 | support dynamic simulation environment, user can add pedestrians by modifing `pedestrian_config.yaml`                                                                             |

## <span id="7">07. Acknowledgments
* Our robot and world models are from [
Dataset-of-Gazebo-Worlds-Models-and-Maps](https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps) and [
aws-robomaker-small-warehouse-world](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world). Thanks for these open source models sincerely.
* Pedestrians in this environment are using social force model(sfm), which comes from [https://github.com/robotics-upo/lightsfm](https://github.com/robotics-upo/lightsfm).
* A ROS costmap plugin for [dynamicvoronoi](http://www2.informatik.uni-freiburg.de/~lau/dynamicvoronoi/) presented by Boris Lau.

## <span id="8">08. License

The source code is released under [GPLv3](https://www.gnu.org/licenses/) license.

## <span id="9">09. Maintenance

Feel free to contact us if you have any question.
