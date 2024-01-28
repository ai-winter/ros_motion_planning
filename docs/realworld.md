# Application on a Real Robot

> **In a word, compile our repository and make it visible to the robot, and then replace it just like other planner in `ROS navigation`.**

### Example

1. Download and compile this repository.
    ```bash
    git clone https://github.com/ai-winter/ros_motion_planning.git

    cd ros_motion_planning/
    catkin_make
    ```

2. Download and compile the 'real robot' software. The software is usually obtained from your robot supplier. Here we use another [gazebo simulation](https://github.com/ZhanyuGuo/ackermann_ws) as an example, like we have a robot which has the capacity of localization, mapping and navigation (*using move_base*).
    ```bash
    # ---- EXAMPLE SOFTWARE ----
    git clone https://github.com/ZhanyuGuo/ackermann_ws.git

    cd ackermann_ws/

    # ---- IMPORTANT HERE ----
    source ../ros_motion_planning/devel/setup.bash

    catkin_make
    ```

    **NOTE1: Sourcing other workspaces before `catkin_make` will make the current `setup.bash` contain former sourced workspaces, i.e., they are also included when you only source this current workspace later.**
    
    **NOTE2: Remove the old `build/` and `devel/` of the current workspace before doing this, otherwise it will not work.**

3. Before navigation, you need to do a mapping and save the map. Your real robot software should have provided a mapping program.

4. Create a `navigation.launch` file in your workspace. Edit it as follows. You should refill the annotated lines according to your real robot software.
    ```xml
    <?xml version="1.0"?>
    <launch>
        <!--include file="your_control_drive" /-->
        <!--include file="your_lidar_drive" /-->
        <!--node name="map_server" pkg="map_server" type="map_server" args="your_map" /-->

        <!--Use tf package to perform coordinate system transformation, if neccessary-->
        <!--node pkg="tf" type="" name="" args="" /-->

        <!-- AMCL -->
        <node pkg="amcl" type="amcl" name="amcl">
            <rosparam file="$(find sim_env)/config/amcl_params.yaml" command="load" />
            <param name="initial_pose_x" value="0.0" />
            <param name="initial_pose_y" value="0.0" />
            <param name="initial_pose_z" value="0.0" />
            <remap from="static_map" to="/static_map" />
        </node>
    
        <!-- MoveBase-->
        <include file="$(find sim_env)/launch/include/navigation/move_base.launch.xml">
            <arg name="robot" value="turtlebot3_waffle" />
            <arg name="global_planner" value="a_star" />
            <arg name="local_planner" value="dwa" />
        </include>
    
        <!-- Rviz-->
        <node name="rviz" pkg="rviz" type="rviz" args="-d $(find sim_env)/rviz/sim_env.rviz" />
    </launch>
    ```

5. Now you can run `navigation.launch` in your workspace to do motion planning. But maybe there are still some details that you have to deal with...
