/**
 * *********************************************************
 *
 * @file: rpp_controller.h
 * @brief: Contains the regulated_pure_pursuit (RPP) local controller class
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
#ifndef RMP_CONTROLLER_RPP_CONTROLLER_H_
#define RMP_CONTROLLER_RPP_CONTROLLER_H_

#include <geometry_msgs/PointStamped.h>
#include <tf2/utils.h>

#include "controller/controller.h"
#include "system_config/controller_protos/rpp_controller.pb.h"

namespace rmp {
namespace controller {
/**
 * @brief A class implementing a local planner using the RPP
 */
class RPPController : public nav_core::BaseLocalPlanner, Controller {
public:
  /**
   * @brief Construct a new RPP planner object
   */
  RPPController();

  /**
   * @brief Construct a new RPP planner object
   */
  RPPController(std::string name, tf2_ros::Buffer* tf,
                costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Destroy the RPP planner object
   */
  ~RPPController();

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

private:
  /**
   * @brief Applying curvature constraints to regularize the speed of robot turning
   * @param raw_linear_vel    the raw linear velocity of robot
   * @param curvature         the tracking curvature
   * @return reg_vel          the regulated velocity
   */
  double _applyCurvatureConstraint(const double raw_linear_vel, const double curvature);

  /**
   * @brief Applying obstacle constraints to regularize the speed of robot approaching
   * obstacles
   * @param raw_linear_vel    the raw linear velocity of robot
   * @return reg_vel          the regulated velocity
   */
  double _applyObstacleConstraint(const double raw_linear_vel);

  /**
   * @brief Applying approach constraints to regularize the speed of robot approaching
   * final goal
   * @param raw_linear_vel    the raw linear velocity of robot
   * @param robot_pose_global the robot's pose  [global]
   * @param prune_plan        the pruned plan
   * @return reg_vel          the regulated velocity
   */
  double _applyApproachConstraint(
      const double raw_linear_vel, geometry_msgs::PoseStamped robot_pose_global,
      const std::vector<geometry_msgs::PoseStamped>& prune_plan);

private:
  pb::controller::RPPController rpp_config_;

  bool initialized_;     // initialized flag
  bool goal_reached_;    // goal reached flag
  tf2_ros::Buffer* tf_;  // transform buffer

  ros::Publisher target_pt_pub_, current_pose_pub_;

  // goal parameters
  double goal_x_, goal_y_, goal_theta_;
};
}  // namespace controller
}  // namespace rmp
#endif