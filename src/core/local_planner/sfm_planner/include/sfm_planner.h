#ifndef SFM_PLANNER_H
#define SFM_PLANNER_H

#include <nav_core/base_local_planner.h>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <tf2/utils.h>

#include "lightsfm/sfm.hpp"
#include "local_planner.h"

using namespace std;

namespace sfm_planner
{
class SfmPlanner : public nav_core::BaseLocalPlanner, local_planner::LocalPlanner
{
public:
  SfmPlanner();
  SfmPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  ~SfmPlanner();

  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  bool isGoalReached();

private:
  costmap_2d::Costmap2DROS* costmap_ros_;
  tf2_ros::Buffer* tf_;
  bool initialized_, odom_flag_, goal_reached_, first_plan_;

  int agent_number_, agent_id_;  // id begin from 1
  double d_t_;                   // control time step

  sfm::Goal goal_;
  sfm::Agent agent_;
  std::vector<sfm::Agent> others_;
  std::vector<ros::Subscriber> odom_subs_;
  std::vector<nav_msgs::Odometry> other_odoms_;

  void initState();
  void handleAgents();

  void odometryCallback(const nav_msgs::OdometryConstPtr& msg, int agent_id);
};

};  // namespace sfm_planner
#endif