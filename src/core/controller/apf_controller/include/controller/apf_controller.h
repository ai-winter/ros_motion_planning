/**
 * *********************************************************
 *
 * @file: apf_controller.h
 * @brief: Contains the Artificial Potential Field (APF) local controller class
 * @author: Yang Haodong, Wu Maojia
 * @date: 2023-10-17
 * @version: 1.2
 *
 * Copyright (c) 2024, Yang Haodong.
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
#include "system_config/controller_protos/apf_controller.pb.h"

namespace rmp {
namespace controller {
/**
 * @brief A class implementing a local controller using the APF
 */
class APFController : public nav_core::BaseLocalPlanner, Controller {
public:
  /**
   * @brief Construct a new APFController object
   */
  APFController();

  /**
   * @brief Construct a new APFController object
   */
  APFController(std::string name, tf2_ros::Buffer* tf,
                costmap_2d::Costmap2DROS* costmap_ros);

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
  void initialize(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap_ros);

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
   * @brief Given the current position, orientation, and velocity of the robot, compute
   * the velocity commands
   * @param cmd_vel will be filled with the velocity command to be passed to the robot
   * base
   * @return  true if a valid trajectory was found, else false
   */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  /**
   * @brief Get the attractive force of APF
   * @param ps      global target PointStamped
   * @return the attractive force
   */
  common::geometry::Vec2d getAttractiveForce(const common::geometry::Vec2d& current_pt,
                                             const common::geometry::Vec2d& target_pt);

  /**
   * @brief Get the repulsive force of APF
   * @return the repulsive force
   */
  common::geometry::Vec2d getRepulsiveForce(const common::geometry::Vec2d& current_pt);

  /**
   * @brief Callback function of costmap_sub_ to publish /potential_map topic
   * @param msg the message received from topic /move_base/local_costmap/costmap
   */
  void publishPotentialMap(const common::geometry::Vec2d& current_pt,
                           const common::geometry::Vec2d& target_pt);

private:
  pb::controller::APFController apf_config_;
  bool initialized_;     // initialized flag
  bool goal_reached_;    // goal reached flag
  tf2_ros::Buffer* tf_;  // transform buffer

  std::deque<common::geometry::Vec2d> hist_nf_;  // historical net forces

  ros::Publisher target_pt_pub_, current_pose_pub_, potential_map_pub_;

  // goal parameters
  double goal_x_, goal_y_, goal_theta_;
};
};  // namespace controller
}  // namespace rmp
#endif