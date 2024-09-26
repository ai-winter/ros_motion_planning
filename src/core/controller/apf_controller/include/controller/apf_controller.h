/**
 * *********************************************************
 *
 * @file: apf_controller.h
 * @brief: Contains the Artificial Potential Field (APF) local controller class
 * @author: Wu Maojia, Yang Haodong
 * @date: 2023-10-17
 * @version: 1.2
 *
 * Copyright (c) 2024, Wu Maojia, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_CONTROLLER_APF_CONTROLLER_H_
#define RMP_CONTROLLER_APF_CONTROLLER_H_

#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2/utils.h>

#include "common/geometry/vec2d.h"
#include "controller/controller.h"

namespace rmp
{
namespace controller
{
/**
 * @brief A class implementing a local controller using the APF
 */
class APFController : public nav_core::BaseLocalPlanner, Controller
{
public:
  /**
   * @brief Construct a new APFController object
   */
  APFController();

  /**
   * @brief Construct a new APFController object
   */
  APFController(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Destroy the APFController object
   */
  ~APFController();

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
   * @brief Get the attractive force of APF
   * @param ps      global target PoseStamped
   * @return the attractive force
   */
  rmp::common::geometry::Vec2d getAttractiveForce(const geometry_msgs::PoseStamped& ps);

  /**
   * @brief Get the repulsive force of APF
   * @return the repulsive force
   */
  rmp::common::geometry::Vec2d getRepulsiveForce();

  /**
   * @brief Callback function of costmap_sub_ to publish /potential_map topic
   * @param msg the message received from topic /move_base/local_costmap/costmap
   */
  void publishPotentialMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);

private:
  bool initialized_;                       // initialized flag
  bool goal_reached_;                      // goal reached flag
  tf2_ros::Buffer* tf_;                    // transform buffer
  nav_msgs::OccupancyGrid potential_map_;  // local potential field map

  int plan_index_;
  // std::vector<geometry_msgs::PoseStamped> global_plan_;
  geometry_msgs::PoseStamped target_ps_, current_ps_;

  double p_window_;     // next point distance
  double o_precision_;  // goal reached tolerance
  double d_t_;          // control time step

  int s_window_;  // trajectory smoothing window time

  double zeta_, eta_;  // scale factor of attractive and repulsive force

  int cost_ub_, cost_lb_;  // the upper and lower bound of costmap used to calculate potential field

  double inflation_radius_;  // the costmap inflation radius of obstacles

  std::deque<rmp::common::geometry::Vec2d> hist_nf_;  // historical net forces

  ros::Publisher target_pose_pub_, current_pose_pub_, potential_map_pub_;
  ros::Subscriber costmap_sub_;  // subscribe local map topic to generate potential field

  // goal parameters
  double goal_x_, goal_y_, goal_theta_;
};
};  // namespace controller
}  // namespace rmp
#endif