/***********************************************************
 *
 * @file: apf_planner.h
 * @breif: Contains the Artificial Potential Field (APF) local planner class
 * @author: Wu Maojia, Yang Haodong
 * @update: 2023-10-17
 * @version: 1.2
 *
 * Copyright (c) 2023，Wu Maojia, Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/

#ifndef APF_PLANNER_H_
#define APF_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <tf2/utils.h>

#include <Eigen/Dense>

#include "local_planner.h"

namespace apf_planner
{
/**
 * @brief A class implementing a local planner using the APF
 */
class APFPlanner : public nav_core::BaseLocalPlanner, local_planner::LocalPlanner
{
public:
  /**
   * @brief Construct a new APFPlanner object
   */
  APFPlanner();

  /**
   * @brief Construct a new APFPlanner object
   */
  APFPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Destroy the APFPlanner object
   */
  ~APFPlanner();

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
   * @return  true if the plan was updated successfully, else false
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  /**
   * @brief  Check if the goal pose has been achieved
   * @return True if achieved, false otherwise
   */
  bool isGoalReached();

  /**
   * @brief Given the current position, orientation, and velocity of the robot, compute the velocity commands
   * @param cmd_vel will be filled with the velocity command to be passed to the robot base
   * @return  true if a valid trajectory was found, else false
   */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  /**
   * @brief APF controller in linear
   * @param base_odometry odometry of the robot, to get velocity
   * @param v_d           desired velocity magnitude
   * @return  linear velocity
   */
  double LinearAPFController(nav_msgs::Odometry& base_odometry, double v_d);

  /**
   * @brief APF controller in angular
   * @param base_odometry odometry of the robot, to get velocity
   * @param e_theta       the error between the current and desired theta
   * @return  angular velocity
   */
  double AngularAPFController(nav_msgs::Odometry& base_odometry, double e_theta);

  /**
   * @brief Get the attractive force of APF
   * @param ps      global target PoseStamped
   * @return the attractive force
   */
  Eigen::Vector2d getAttractiveForce(const geometry_msgs::PoseStamped& ps);

  /**
   * @brief Get the repulsive force of APF
   * @return the repulsive force
   */
  Eigen::Vector2d getRepulsiveForce();

  /**
   * @brief Callback function of costmap_sub_ to publish /potential_map topic
   * @param msg the message received from topic /move_base/local_costmap/costmap
   */
  void publishPotentialMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);

private:
  bool initialized_, goal_reached_;
  tf2_ros::Buffer* tf_;
  costmap_2d::Costmap2DROS* costmap_ros_;  // costmap(ROS wrapper)
  unsigned char* local_costmap_;           // costmap char map
  nav_msgs::OccupancyGrid potential_map_; // local potential field map

  int plan_index_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  geometry_msgs::PoseStamped target_ps_, current_ps_;

  double p_window_;                   // next point distance
  double p_precision_, o_precision_;  // goal reached tolerance
  double d_t_;                        // control time step

  double max_v_, min_v_, max_v_inc_;  // linear velocity
  double max_w_, min_w_, max_w_inc_;  // angular velocity

  int s_window_;  // trajectory smoothing window time

  double zeta_, eta_;  // scale factor of attractive and repulsive force

  int cost_ub_, cost_lb_;  // the upper and lower bound of costmap used to calculate potential field

  double inflation_radius_; // the costmap inflation radius of obstacles

  std::deque<Eigen::Vector2d> hist_nf_;  // historical net forces

  base_local_planner::OdometryHelperRos* odom_helper_;
  ros::Publisher target_pose_pub_, current_pose_pub_, potential_map_pub_;
  ros::Subscriber costmap_sub_;   // subscribe local map topic to generate potential field

  double goal_x_, goal_y_;
  Eigen::Vector3d goal_rpy_;
};
};  // namespace apf_planner

#endif