#ifndef ORCA_PLANNER_H
#define ORCA_PLANNER_H

// abstract class from which our plugin inherits
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
#include "local_planner.h"

using namespace std;

namespace orca_planner
{

class OrcaPlanner : public nav_core::BaseLocalPlanner, local_planner::LocalPlanner
{
public:
  OrcaPlanner();
  OrcaPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  ~OrcaPlanner();

  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  bool isGoalReached();

private:
  costmap_2d::Costmap2DROS* costmap_ros_;
  tf2_ros::Buffer* tf_;
  bool initialized_, odom_flag_;

  int agent_number_;
  int agent_id_;  // NOTE: begin from 1
  RVO::RVOSimulator* sim_;
  double d_t_;         // control time step
  bool goal_reached_;  // goal reached flag
  RVO::Vector2 goal_;
  std::vector<ros::Subscriber> odom_subs_;
  std::vector<nav_msgs::Odometry> other_odoms_;

  void odometryCallback(const nav_msgs::OdometryConstPtr& msg, int agent_id);
  void initializeStates();
  void updateStates();
};
};  // namespace orca_planner

#endif