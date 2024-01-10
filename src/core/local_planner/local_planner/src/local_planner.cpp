/***********************************************************
 *
 * @file: local_planner.cpp
 * @breif: Contains some implement of local planner class
 * @author: Yang Haodong
 * @update: 2023-10-2
 * @version: 1.0
 *
 * Copyright (c) 2023， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include <tf2/utils.h>

#include "local_planner.h"

namespace local_planner
{
/**
 * @brief Construct a new Local Planner object
 */
LocalPlanner::LocalPlanner()
  : lethal_cost_(LETHAL_COST)
  , neutral_cost_(NEUTRAL_COST)
  , factor_(OBSTACLE_FACTOR)
  , base_frame_("base_link")
  , map_frame_("map")
  , odom_frame_("odom")
  , convert_offset_(0.0)
{
  odom_helper_ = new base_local_planner::OdometryHelperRos(odom_frame_);
}

/**
 * @brief Destroy the Local Planner object
 */
LocalPlanner::~LocalPlanner()
{
  delete odom_helper_;
}

/**
 * @brief Set or reset costmap size
 * @param nx  pixel number in costmap x direction
 * @param ny  pixel number in costmap y direction
 */
void LocalPlanner::setSize(int nx, int ny)
{
  nx_ = nx;
  ny_ = ny;
  ns_ = nx * ny;
}

/**
 * @brief Set or reset costmap resolution
 * @param resolution  costmap resolution
 */
void LocalPlanner::setResolution(double resolution)
{
  resolution_ = resolution;
}

/**
 * @brief Set or reset costmap origin
 * @param origin_x  origin in costmap x direction
 * @param origin_y  origin in costmap y direction
 */
void LocalPlanner::setOrigin(double origin_x, double origin_y)
{
  origin_x_ = origin_x;
  origin_y_ = origin_y;
}

/**
 * @brief Set or reset lethal cost
 * @param neutral_cost  neutral cost
 */
void LocalPlanner::setLethalCost(unsigned char lethal_cost)
{
  lethal_cost_ = lethal_cost;
}

/**
 * @brief Set or reset neutral cost
 * @param neutral_cost  neutral cost
 */
void LocalPlanner::setNeutralCost(unsigned char neutral_cost)
{
  neutral_cost_ = neutral_cost;
}

/**
 * @brief Set or reset obstacle factor
 * @param factor  obstacle factor
 */
void LocalPlanner::setFactor(double factor)
{
  factor_ = factor;
}

/**
 * @brief Set or reset frame name
 * @param frame_name
 */
void LocalPlanner::setBaseFrame(std::string base_frame)
{
  base_frame_ = base_frame;
}
void LocalPlanner::setMapFrame(std::string map_frame)
{
  map_frame_ = map_frame;
}

/**
 * @brief Regularize angle to [-pi, pi]
 * @param angle the angle (rad) to regularize
 */
void LocalPlanner::regularizeAngle(double& angle)
{
  angle = angle - 2.0 * M_PI * std::floor((angle + M_PI) / (2.0 * M_PI));
}

/**
 * @brief Get the Euler Angles from PoseStamped
 * @param ps  PoseStamped to calculate
 * @return  roll, pitch and yaw in XYZ order
 */
Eigen::Vector3d LocalPlanner::getEulerAngles(geometry_msgs::PoseStamped& ps)
{
  tf2::Quaternion q(ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w);
  tf2::Matrix3x3 m(q);

  double roll(0.0), pitch(0.0), yaw(0.0);
  m.getRPY(roll, pitch, yaw);

  return Eigen::Vector3d(roll, pitch, yaw);
}

/**
 * @brief Whether to reach the target pose through rotation operation
 * @param cur   current pose of robot
 * @param goal  goal pose of robot
 * @return true if robot should perform rotation
 */
bool LocalPlanner::shouldRotateToGoal(const geometry_msgs::PoseStamped& cur, const geometry_msgs::PoseStamped& goal)
{
  std::pair<double, double> p1(cur.pose.position.x, cur.pose.position.y);
  std::pair<double, double> p2(goal.pose.position.x, goal.pose.position.y);
  
  return helper::dist(p1, p2) < goal_dist_tol_;
}

/**
 * @brief Whether to correct the tracking path with rotation operation
 * @param angle_to_path  the angle deviation
 * @return true if robot should perform rotation
 */
bool LocalPlanner::shouldRotateToPath(double angle_to_path, double tolerance)
{
  return (tolerance && (angle_to_path > tolerance)) || (angle_to_path > rotate_tol_);
}

/**
 * @brief linear velocity regularization
 * @param base_odometry odometry of the robot, to get velocity
 * @param v_d           desired velocity magnitude
 * @return v            regulated linear velocity
 */
double LocalPlanner::linearRegularization(nav_msgs::Odometry& base_odometry, double v_d)
{
  double v = std::hypot(base_odometry.twist.twist.linear.x, base_odometry.twist.twist.linear.y);
  double v_inc = v_d - v;

  if (std::fabs(v_inc) > max_v_inc_)
    v_inc = std::copysign(max_v_inc_, v_inc);

  double v_cmd = v + v_inc;
  if (std::fabs(v_cmd) > max_v_)
    v_cmd = std::copysign(max_v_, v_cmd);
  else if (std::fabs(v_cmd) < min_v_)
    v_cmd = std::copysign(min_v_, v_cmd);

  return v_cmd;
}

/**
 * @brief angular velocity regularization
 * @param base_odometry odometry of the robot, to get velocity
 * @param w_d           desired angular velocity
 * @return  w           regulated angular velocity
 */
double LocalPlanner::angularRegularization(nav_msgs::Odometry& base_odometry, double w_d)
{
  if (std::fabs(w_d) > max_w_)
    w_d = std::copysign(max_w_, w_d);

  double w = base_odometry.twist.twist.angular.z;
  double w_inc = w_d - w;

  if (std::fabs(w_inc) > max_w_inc_)
    w_inc = std::copysign(max_w_inc_, w_inc);

  double w_cmd = w + w_inc;
  if (std::fabs(w_cmd) > max_w_)
    w_cmd = std::copysign(max_w_, w_cmd);
  else if (std::fabs(w_cmd) < min_w_)
    w_cmd = std::copysign(min_w_, w_cmd);

  return w_cmd;
}

/**
 * @brief Tranform from in_pose to out_pose with out frame using tf
 */
void LocalPlanner::transformPose(tf2_ros::Buffer* tf, const std::string out_frame,
                                 const geometry_msgs::PoseStamped& in_pose, geometry_msgs::PoseStamped& out_pose) const
{
  if (in_pose.header.frame_id == out_frame)
    out_pose = in_pose;

  tf->transform(in_pose, out_pose, out_frame);
  out_pose.header.frame_id = out_frame;
}

/**
 * @brief Tranform from world map(x, y) to costmap(x, y)
 * @param mx  costmap x
 * @param my  costmap y
 * @param wx  world map x
 * @param wy  world map y
 * @return true if successfull, else false
 */
bool LocalPlanner::worldToMap(double wx, double wy, int& mx, int& my)
{
  if (wx < origin_x_ || wy < origin_y_)
    return false;

  mx = (int)((wx - origin_x_) / resolution_ - convert_offset_);
  my = (int)((wy - origin_y_) / resolution_ - convert_offset_);
  if (mx < nx_ && my < ny_)
    return true;

  return false;
}
}  // namespace local_planner