# Application on a Real Robot

> **In a word, compile our repository and make it visible to the robot, and then replace it just like other planner in `ROS navigation`.**

### Example

We use another [gazebo simulation](https://github.com/ZhanyuGuo/ackermann_ws) as an example, like we have a robot which has the capacity of localization, mapping and navigation (*using move_base*).

1. Download and compile this repository.
    ```bash
    git clone https://github.com/ai-winter/ros_motion_planning.git

    cd ros_motion_planning/
    catkin_make
    ```

2. Download and compile the 'real robot' software.
    ```bash
    git clone https://github.com/ZhanyuGuo/ackermann_ws.git

    cd ackermann_ws/

    # ---- IMPORTANT HERE ----
    source ../ros_motion_planning/devel/setup.bash

    catkin_make
    ```

    **NOTE1: Sourcing other workspaces before `catkin_make` will make the current `setup.bash` contain former sourced workspaces, i.e., they are also included when you only source this current workspace later.**
    
    **NOTE2: Remove the old `build/` and `devel/` of the current workspace before doing this, otherwise it will not work.**

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
