/**
 * *********************************************************
 *
 * @file: pid_planner.h
 * @brief: Contains the Proportional–Integral–Derivative (PID) controller local planner class
 * @author: Yang Haodong, Guo Zhanyu, Wu Maojia
 * @date: 2024-01-20
 * @version: 1.2
 *
 * Copyright (c) 2024, Yang Haodong, Guo Zhanyu, Wu Maojia.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */

#ifndef PID_PLANNER_H_
#define PID_PLANNER_H_

#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Twist.h>

#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>

#include "local_planner.h"

namespace pid_planner
{
/**
 * @brief A class implementing a local planner using the PID
 */
class PIDPlanner : public nav_core::BaseLocalPlanner, local_planner::LocalPlanner
{
public:
  /**
   * @brief Construct a new PIDPlanner object
   */
  PIDPlanner();

  /**
   * @brief Construct a new PIDPlanner object
   */
  PIDPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Destroy the PIDPlanner object
   */
  ~PIDPlanner();

  /**
   * @brief Initialization of the local planner
   * @param name        the name to give this instance of the trajectory planner
   * @param tf          a pointer to a transform listener
   * @param costmap_ros the cost map to use for assigning costs to trajectories
   */
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Set the plan that the controller is following
   * @param orig_global_plan the plan to pass to the controller
   * @return true if the plan was updated successfully, else false
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  /**
   * @brief Check if the goal pose has been achieved
   * @return true if achieved, false otherwise
   */
  bool isGoalReached();

  /**
   * @brief Given the current position, orientation, and velocity of the robot, compute the velocity commands
   * @param cmd_vel will be filled with the velocity command to be passed to the robot base
   * @return true if a valid trajectory was found, else false
   */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

private:
  bool initialized_;     // initialized flag
  bool goal_reached_;    // goal reached flag
  tf2_ros::Buffer* tf_;  // transform buffer
  // costmap_2d::Costmap2DROS* costmap_ros_;  // costmap(ROS wrapper)

  // std::vector<geometry_msgs::PoseStamped> global_plan_;

  // pid controller params
  double k_v_p_, k_v_i_, k_v_d_;
  double k_w_p_, k_w_i_, k_w_d_;
  double k_theta_;
  double k_, l_;

  double e_v_, e_w_;
  double i_v_, i_w_;

  double d_t_;  // control time step

  ros::Publisher target_pose_pub_, current_pose_pub_;

  // goal parameters
  double goal_x_, goal_y_;
  Eigen::Vector3d goal_rpy_;

  /**
   * @brief Execute PID control process (no model pid)
   * @param s   current state
   * @param s_d desired state
   * @param u_r refered input
   * @return u  control vector
   */
  Eigen::Vector2d _pidControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector2d u_r);
};
};  // namespace pid_planner

#endif