/**
 * *********************************************************
 *
 * @file: trajectory_planner_factory.h
 * @brief: Create the trajectory planner with specifical parameters
 * @author: Yang Haodong
 * @date: 2025-02-16
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_PATH_PLANNER_UTILS_TRAJECTORY_PLANNER_FACTORY_H_
#define RMP_PATH_PLANNER_UTILS_TRAJECTORY_PLANNER_FACTORY_H_

#include <ros/ros.h>

#include "trajectory_planner/trajectory_optimization/optimizer_core.h"

namespace rmp
{
namespace path_planner
{
class TrajectoryPlannerFactory
{
public:
  struct PlannerProps
  {
    std::string optimizer_name;
    std::shared_ptr<rmp::trajectory_optimization::Optimizer> traj_optimizer_ptr;  // trajectory optimizer
  };

public:
  /**
   * @brief Create and configure planner
   * @param nh ROS node handler
   * @param costmap_ros costmap ROS wrapper
   * @param planner_props planner property
   * @return bool true if create successful, else false
   */
  static bool createPlanner(ros::NodeHandle& nh, costmap_2d::Costmap2DROS* costmap_ros, PlannerProps& planner_props);
};
}  // namespace path_planner
}  // namespace rmp

#endif