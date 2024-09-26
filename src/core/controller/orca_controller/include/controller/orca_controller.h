/**
 * *********************************************************
 *
 * @file: orca_controller.h
 * @brief: Contains the ORCA local controller class
 * @author: Yang Haodong, Guo Zhanyu
 * @date: 2024-01-20
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong, Guo Zhanyu.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_CONTROLLER_ORCA_CONTROLLER_H_
#define RMP_CONTROLLER_ORCA_CONTROLLER_H_

#include <nav_core/base_local_planner.h>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <tf2/utils.h>

#include "RVO/RVO.h"
#include "controller/controller.h"

namespace rmp
{
namespace controller
{
class ORCAController : public nav_core::BaseLocalPlanner, Controller
{
public:
  ORCAController();
  ORCAController(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  ~ORCAController();

  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  bool isGoalReached();

private:
  costmap_2d::Costmap2DROS* costmap_ros_;
  tf2_ros::Buffer* tf_;
  bool initialized_, odom_flag_, goal_reached_, first_plan_;

  int agent_number_, agent_id_;                                       // id begin from 1
  double d_t_;                                                        // control time step
  double neighbor_dist_, time_horizon_, time_horizon_obst_, radius_;  // orca parameters
  int max_neighbors_;

  RVO::RVOSimulator* sim_;
  RVO::Vector2 goal_;
  std::vector<ros::Subscriber> odom_subs_;
  std::vector<nav_msgs::Odometry> other_odoms_;

  void odometryCallback(const nav_msgs::OdometryConstPtr& msg, int agent_id);

  void initState();

  void updateState();
};
};  // namespace controller
}  // namespace rmp
#endif