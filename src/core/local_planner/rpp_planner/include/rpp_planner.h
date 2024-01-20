/**
 * *********************************************************
 *
 * @file: rpp_planner.h
 * @brief: Contains the regulated_pure_pursuit (RPP) local planner class
 * @author: Yang Haodong
 * @date: 2024-01-08
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RPP_PLANNER_H
#define RPP_PLANNER_H

#include <geometry_msgs/PointStamped.h>
#include <tf2/utils.h>

#include "local_planner.h"

namespace rpp_planner
{
/**
 * @brief A class implementing a local planner using the RPP
 */
class RPPPlanner : public nav_core::BaseLocalPlanner, local_planner::LocalPlanner
{
public:
  /**
   * @brief Construct a new RPP planner object
   */
  RPPPlanner();

  /**
   * @brief Construct a new RPP planner object
   */
  RPPPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Destroy the RPP planner object
   */
  ~RPPPlanner();

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

protected:
  /**
   * @brief calculate the relative angle between robot' yaw and relative lookahead vector
   * @param lookahead_pt      the lookahead pose [global]
   * @param robot_pose_global the robot's pose  [global]
   * @return dphi             the lookahead angle - robot's yaw
   */
  double _dphi(geometry_msgs::PointStamped lookahead_pt, geometry_msgs::PoseStamped robot_pose_global);

private:
  /**
   * @brief Applying curvature constraints to regularize the speed of robot turning
   * @param raw_linear_vel    the raw linear velocity of robot
   * @param curvature         the tracking curvature
   * @return reg_vel          the regulated velocity
   */
  double _applyCurvatureConstraint(const double raw_linear_vel, const double curvature);

  /**
   * @brief Applying obstacle constraints to regularize the speed of robot approaching obstacles
   * @param raw_linear_vel    the raw linear velocity of robot
   * @return reg_vel          the regulated velocity
   */
  double _applyObstacleConstraint(const double raw_linear_vel);

  /**
   * @brief Applying approach constraints to regularize the speed of robot approaching final goal
   * @param raw_linear_vel    the raw linear velocity of robot
   * @param robot_pose_global the robot's pose  [global]
   * @param prune_plan        the pruned plan
   * @return reg_vel          the regulated velocity
   */
  double _applyApproachConstraint(const double raw_linear_vel, geometry_msgs::PoseStamped robot_pose_global,
                                  const std::vector<geometry_msgs::PoseStamped>& prune_plan);

private:
  bool initialized_;     // initialized flag
  bool goal_reached_;    // goal reached flag
  tf2_ros::Buffer* tf_;  // transform buffer
  // costmap_2d::Costmap2DROS* costmap_ros_;  // costmap(ROS wrapper)

  double d_t_;                          // control time interval
  double regulated_min_radius_;         // the threshold to apply curvature constraint
  double inflation_cost_factor_;        // the heuristical factor to calculate minimum distance to obstacles
  double scaling_dist_, scaling_gain_;  // the threshold to apply obstacle constraint
  double approach_dist_;                // the threshold to apply approaching goal constraint
  double approach_min_v_;               // minimum approaching velocity

  // std::vector<geometry_msgs::PoseStamped> global_plan_;
  ros::Publisher target_pt_pub_, current_pose_pub_;

  // goal parameters
  double goal_x_, goal_y_;
  Eigen::Vector3d goal_rpy_;
};
}  // namespace rpp_planner
#endif