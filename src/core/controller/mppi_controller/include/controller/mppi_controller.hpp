/**
 * *********************************************************
 *
 * @file: mppi_controller.hpp
 * @brief: Contains the Model Predictive Path Integral (MPPI) controller local controller
 * class
 * @author: Zhanyu Guo
 * @date: 2026-02-19
 * @version: 1.0
 *
 * Copyright (c) 2026, Zhanyu Guo.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */

#ifndef RMP_CONTROLLER_MPPI_CONTROLLER_HPP_
#define RMP_CONTROLLER_MPPI_CONTROLLER_HPP_

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Dense>

#include "controller/controller.h"
#include "system_config/controller_protos/mppi_controller.pb.h"

namespace rmp::controller {

class MPPIController : public nav_core::BaseLocalPlanner, Controller {
public:
  MPPIController();

  MPPIController(std::string name, tf2_ros::Buffer* tf,
                 costmap_2d::Costmap2DROS* costmap_ros);

  ~MPPIController();

  void initialize(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap_ros);

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  bool isGoalReached();

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

private:
  pb::controller::MPPIController mppi_config_;

  bool initialized_;     // initialized flag
  bool goal_reached_;    // goal reached flag
  tf2_ros::Buffer* tf_;  // transform buffer

  // goal parameters
  double goal_x_, goal_y_, goal_theta_;
};

}  // namespace rmp::controller

#endif // RMP_CONTROLLER_MPPI_CONTROLLER_HPP_