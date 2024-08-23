/**
 * *********************************************************
 *
 * @file: lazy_planner.h
 * @brief: Contains the lazy planner ROS wrapper class
 * @author: Zhanyu guo
 * @date: 2024-01-01
 * @version: 1.0
 *
 * Copyright (c) 2024, Zhanyu guo.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <pluginlib/class_list_macros.h>

#include "lazy_planner.h"

PLUGINLIB_EXPORT_CLASS(lazy_planner::LazyPlanner, nav_core::BaseGlobalPlanner)

namespace lazy_planner
{
LazyPlanner::LazyPlanner()
{
}

LazyPlanner::LazyPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(name, costmap_ros);
}

void LazyPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
}

bool LazyPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           std::vector<geometry_msgs::PoseStamped>& plan)
{
  plan.push_back(start);
  plan.push_back(goal);
  return true;
}
};  // namespace lazy_planner