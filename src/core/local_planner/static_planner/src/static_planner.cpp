/**
 * *********************************************************
 *
 * @file: static_planner.cpp
 * @brief: Contains the static planner class used in path debugging.
 * @author: Yang Haodong
 * @date: 2023-08-31
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong. 
 * All rights reserved.
 * 
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "static_planner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(static_planner::StaticPlanner, nav_core::BaseLocalPlanner)

namespace static_planner
{
/**
 * @brief Construct a new StaticPlanner object
 */
StaticPlanner::StaticPlanner() : initialized_(false), goal_reached_(false)
{
}

/**
 * @brief Construct a new StaticPlanner object
 */
StaticPlanner::StaticPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
  : StaticPlanner()
{
  initialize(name, tf, costmap_ros);
}

/**
 * @brief Destroy the StaticPlanner object
 */
StaticPlanner::~StaticPlanner()
{
}

/**
 * @brief Initialization of the local planner
 * @param name        the name to give this instance of the trajectory planner
 * @param tf          a pointer to a transform listener
 * @param costmap_ros the cost map to use for assigning costs to trajectories
 */
void StaticPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_)
  {
    initialized_ = true;
    ros::NodeHandle nh = ros::NodeHandle("~/" + name);

    ROS_INFO("Static planner initialized!");
  }
  else
    ROS_WARN("Static planner has already been initialized.");
}

/**
 * @brief  Set the plan that the controller is following
 * @param orig_global_plan the plan to pass to the controller
 * @return  true if the plan was updated successfully, else false
 */
bool StaticPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }
  return true;
}

/**
 * @brief  Check if the goal pose has been achieved
 * @return True if achieved, false otherwise
 */
bool StaticPlanner::isGoalReached()
{
  if (!initialized_)
  {
    ROS_ERROR("Static planner has not been initialized");
    return false;
  }

  if (goal_reached_)
  {
    ROS_INFO("GOAL Reached!");
    return true;
  }
  return false;
}

/**
 * @brief Static commands
 * @param cmd_vel will be filled with the velocity command to be passed to the robot base
 * @return  true if a valid trajectory was found, else false
 */
bool StaticPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_)
  {
    ROS_ERROR("Static planner has not been initialized");
    return false;
  }

  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;

  return true;
}

}  // namespace static_planner