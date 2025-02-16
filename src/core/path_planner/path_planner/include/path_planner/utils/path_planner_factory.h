/**
 * *********************************************************
 *
 * @file: path_planner_factory.h
 * @brief: Create the planner with specifical parameters
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
#ifndef RMP_PATH_PLANNER_UTILS_PATH_PLANNER_FACTORY_H_
#define RMP_PATH_PLANNER_UTILS_PATH_PLANNER_FACTORY_H_

#include <ros/ros.h>

#include "path_planner/path_planner.h"

namespace rmp
{
namespace path_planner
{
enum PLANNER_TYPE
{
  GRAPH_PLANNER = 0,
  SAMPLE_PLANNER = 1,
  EVOLUTION_PLANNER = 2,
};

class PathPlannerFactory
{
public:
  struct PlannerProps
  {
    PLANNER_TYPE planner_type;
    std::shared_ptr<PathPlanner> planner_ptr;  // global path planner
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