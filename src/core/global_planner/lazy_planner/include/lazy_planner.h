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
#ifndef LAZY_PLANNER_H
#define LAZY_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

namespace lazy_planner
{
class LazyPlanner : public nav_core::BaseGlobalPlanner
{
public:
  LazyPlanner();
  LazyPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);
};
};  // namespace lazy_planner
#endif