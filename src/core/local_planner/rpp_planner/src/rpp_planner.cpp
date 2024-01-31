/**
 * *********************************************************
 *
 * @file: rpp_planner.cpp
 * @brief: Contains the regulated_pure_pursuit (RPP) local planner class
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
#include <pluginlib/class_list_macros.h>

#include "rpp_planner.h"
#include "math_helper.h"

PLUGINLIB_EXPORT_CLASS(rpp_planner::RPPPlanner, nav_core::BaseLocalPlanner)

namespace rpp_planner
{
/**
 * @brief Construct a new RPP planner object
 */
RPPPlanner::RPPPlanner() : initialized_(false), tf_(nullptr), goal_reached_(false)  //, costmap_ros_(nullptr)
{
}

/**
 * @brief Construct a new RPP planner object
 */
RPPPlanner::RPPPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) : RPPPlanner()
{
  initialize(name, tf, costmap_ros);
}

/**
 * @brief Destroy the RPP planner object
 */
RPPPlanner::~RPPPlanner()
{
}

/**
 * @brief Initialization of the local planner
 * @param name        the name to give this instance of the trajectory planner
 * @param tf          a pointer to a transform listener
 * @param costmap_ros the cost map to use for assigning costs to trajectories
 */
void RPPPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_)
  {
    initialized_ = true;
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

    // set costmap properties
    setSize(costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
    setOrigin(costmap->getOriginX(), costmap->getOriginY());
    setResolution(costmap->getResolution());

    ros::NodeHandle nh = ros::NodeHandle("~/" + name);

    // base
    nh.param("goal_dist_tolerance", goal_dist_tol_, 0.2);
    nh.param("rotate_tolerance", rotate_tol_, 0.5);
    nh.param("convert_offset", convert_offset_, 0.0);
    nh.param("base_frame", base_frame_, base_frame_);
    nh.param("map_frame", map_frame_, map_frame_);

    // lookahead
    nh.param("lookahead_time", lookahead_time_, 1.5);
    nh.param("min_lookahead_dist", min_lookahead_dist_, 0.3);
    nh.param("max_lookahead_dist", max_lookahead_dist_, 0.9);

    // linear velocity
    nh.param("max_v", max_v_, 0.5);
    nh.param("min_v", min_v_, 0.0);
    nh.param("max_v_inc", max_v_inc_, 0.5);

    // angular velocity
    nh.param("max_w", max_w_, 1.57);
    nh.param("min_w", min_w_, 0.0);
    nh.param("max_w_inc", max_w_inc_, 1.57);

    // constriants
    nh.param("regulated_min_radius", regulated_min_radius_, 0.9);
    nh.param("inflation_cost_factor", inflation_cost_factor_, 3.0);
    nh.param("scaling_dist", scaling_dist_, 0.6);
    nh.param("scaling_gain", scaling_gain_, 1.0);
    nh.param("approach_dist", approach_dist_, 0.8);
    nh.param("approach_min_v", approach_min_v_, 0.1);

    double controller_freqency;
    nh.param("/move_base/controller_frequency", controller_freqency, 10.0);
    d_t_ = 1 / controller_freqency;

    odom_helper_ = new base_local_planner::OdometryHelperRos("/odom");
    target_pt_pub_ = nh.advertise<geometry_msgs::PointStamped>("/target_point", 10);
    current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);

    ROS_INFO("RPP planner initialized!");
  }
  else
    ROS_WARN("RPP planner has already been initialized.");
}

/**
 * @brief  Set the plan that the controller is following
 * @param orig_global_plan the plan to pass to the controller
 * @return  true if the plan was updated successfully, else false
 */
bool RPPPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  ROS_INFO("Got new plan");

  // set new plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // reset plan parameters
  if (goal_x_ != global_plan_.back().pose.position.x || goal_y_ != global_plan_.back().pose.position.y)
  {
    goal_x_ = global_plan_.back().pose.position.x;
    goal_y_ = global_plan_.back().pose.position.y;
    goal_rpy_ = getEulerAngles(global_plan_.back());
    goal_reached_ = false;
  }

  return true;
}

/**
 * @brief  Check if the goal pose has been achieved
 * @return True if achieved, false otherwise
 */
bool RPPPlanner::isGoalReached()
{
  if (!initialized_)
  {
    ROS_ERROR("RPP planner has not been initialized");
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
 * @brief Given the current position, orientation, and velocity of the robot, compute the velocity commands
 * @param cmd_vel will be filled with the velocity command to be passed to the robot base
 * @return  true if a valid trajectory was found, else false
 */
bool RPPPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_)
  {
    ROS_ERROR("RPP planner has not been initialized");
    return false;
  }

  // odometry observation - getting robot velocities in robot frame
  nav_msgs::Odometry base_odom;
  odom_helper_->getOdom(base_odom);

  // get robot position in global frame
  geometry_msgs::PoseStamped robot_pose_odom, robot_pose_map;
  costmap_ros_->getRobotPose(robot_pose_odom);
  transformPose(tf_, map_frame_, robot_pose_odom, robot_pose_map);

  // transform global plan to robot frame
  std::vector<geometry_msgs::PoseStamped> prune_plan = prune(robot_pose_map);

  // calculate look-ahead distance
  double vt = std::hypot(base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y);
  double L = getLookAheadDistance(vt);

  // get the particular point on the path at the lookahead distance
  geometry_msgs::PointStamped lookahead_pt;
  double theta, kappa;
  getLookAheadPoint(L, robot_pose_map, prune_plan, lookahead_pt, theta, kappa);

  // get the tracking curvature with goalahead point
  double lookahead_k = 2 * sin(_dphi(lookahead_pt, robot_pose_map)) / L;

  // calculate commands
  if (shouldRotateToGoal(robot_pose_map, global_plan_.back()))
  {
    double e_theta = regularizeAngle(goal_rpy_.z() - tf2::getYaw(robot_pose_map.pose.orientation));

    // orientation reached
    if (!shouldRotateToPath(std::fabs(e_theta)))
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      goal_reached_ = true;
    }
    // orientation not reached
    else
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = angularRegularization(base_odom, e_theta / d_t_);
    }
  }
  else
  {
    double e_theta = regularizeAngle(_dphi(lookahead_pt, robot_pose_map));

    // large angle, turn first
    if (shouldRotateToPath(std::fabs(e_theta), M_PI_2))
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = angularRegularization(base_odom, e_theta / d_t_);
    }

    // apply constraints
    else
    {
      double curv_vel = _applyCurvatureConstraint(max_v_, lookahead_k);
      double cost_vel = _applyObstacleConstraint(max_v_);
      double v_d = std::min(curv_vel, cost_vel);
      v_d = _applyApproachConstraint(v_d, robot_pose_map, prune_plan);

      cmd_vel.linear.x = linearRegularization(base_odom, v_d);
      cmd_vel.angular.z = angularRegularization(base_odom, v_d * lookahead_k);
    }
  }

  // publish lookahead pose
  target_pt_pub_.publish(lookahead_pt);

  // publish robot pose
  current_pose_pub_.publish(robot_pose_map);

  return true;
}

/**
 * @brief calculate the relative angle between robot' yaw and relative lookahead vector
 * @param lookahead_pt      the lookahead pose [global]
 * @param robot_pose_global the robot's pose  [global]
 * @return dphi             the lookahead angle - robot's yaw
 */
double RPPPlanner::_dphi(geometry_msgs::PointStamped lookahead_pt, geometry_msgs::PoseStamped robot_pose_global)
{
  return atan2(lookahead_pt.point.y - robot_pose_global.pose.position.y,
               lookahead_pt.point.x - robot_pose_global.pose.position.x) -
         tf2::getYaw(robot_pose_global.pose.orientation);
}

/**
 * @brief Applying curvature constraints to regularize the speed of robot turning
 * @param raw_linear_vel    the raw linear velocity of robot
 * @param curvature         the tracking curvature
 * @return reg_vel          the regulated velocity
 */
double RPPPlanner::_applyCurvatureConstraint(const double raw_linear_vel, const double curvature)
{
  const double radius = std::fabs(1.0 / curvature);
  if (radius < regulated_min_radius_)
    return raw_linear_vel * (radius / regulated_min_radius_);
  else
    return raw_linear_vel;
}

/**
 * @brief Applying obstacle constraints to regularize the speed of robot approaching obstacles
 * @param raw_linear_vel    the raw linear velocity of robot
 * @return reg_vel          the regulated velocity
 */
double RPPPlanner::_applyObstacleConstraint(const double raw_linear_vel)
{
  int size_x = costmap_ros_->getCostmap()->getSizeInCellsX() / 2;
  int size_y = costmap_ros_->getCostmap()->getSizeInCellsY() / 2;
  double robot_cost = static_cast<double>(costmap_ros_->getCostmap()->getCost(size_x, size_y));

  if (robot_cost != static_cast<double>(costmap_2d::FREE_SPACE) &&
      robot_cost != static_cast<double>(costmap_2d::NO_INFORMATION))
  {
    const double& inscribed_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();

    // calculate the minimum distance to obstacles heuristically
    const double obs_dist =
        inscribed_radius -
        (log(robot_cost) - log(static_cast<double>(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))) / inflation_cost_factor_;

    if (obs_dist < scaling_dist_)
      return raw_linear_vel * scaling_gain_ * obs_dist / scaling_dist_;
  }
  return raw_linear_vel;
}

/**
 * @brief Applying approach constraints to regularize the speed of robot approaching final goal
 * @param raw_linear_vel    the raw linear velocity of robot
 * @param robot_pose_global the robot's pose  [global]
 * @param prune_plan        the pruned plan
 * @return reg_vel          the regulated velocity
 */
double RPPPlanner::_applyApproachConstraint(const double raw_linear_vel, geometry_msgs::PoseStamped robot_pose_global,
                                            const std::vector<geometry_msgs::PoseStamped>& prune_plan)
{
  double remain_dist = 0.0;
  for (size_t i = 0; i < prune_plan.size() - 1; i++)
    remain_dist += helper::dist(prune_plan[i], prune_plan[i + 1]);
  double s = remain_dist < approach_dist_ ? helper::dist(prune_plan.back(), robot_pose_global) / approach_dist_ : 1.0;

  return std::min(raw_linear_vel, std::max(approach_min_v_, raw_linear_vel * s));
}

}  // namespace rpp_planner