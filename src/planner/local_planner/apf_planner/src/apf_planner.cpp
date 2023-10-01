/***********************************************************
*
* @file: apf_planner.cpp
* @breif: Contains the Artificial Potential Field (APF) local planner class
* @author: Wu Maojia, Yang Haodong
* @update: 2023-10-1
* @version: 1.0
*
* Copyright (c) 2023，Wu Maojia, Yang Haodong
* All rights reserved.
* --------------------------------------------------------
*
**********************************************************/

#include "apf_planner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(apf_planner::APFPlanner, nav_core::BaseLocalPlanner)

namespace apf_planner
{
/**
 * @brief Construct a new APFPlanner object
 */
APFPlanner::APFPlanner()
  : initialized_(false)
  , tf_(nullptr)
  , costmap_ros_(nullptr)
  , goal_reached_(false)
  , plan_index_(0)
  , base_frame_("base_link")
  , map_frame_("map")
{
}

/**
 * @brief Construct a new APFPlanner object
 */
APFPlanner::APFPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) : APFPlanner()
{
  initialize(name, tf, costmap_ros);
}

/**
 * @brief Destroy the APFPlanner object
 */
APFPlanner::~APFPlanner()
{
}

/**
 * @brief Initialization of the local planner
 * @param name        the name to give this instance of the trajectory planner
 * @param tf          a pointer to a transform listener
 * @param costmap_ros the cost map to use for assigning costs to trajectories
 */
void APFPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_)
  {
    initialized_ = true;
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    costmap_char_ = costmap_->getCharMap();

    // get costmap properties
    nx_ = costmap_->getSizeInCellsX(), ny_ = costmap_->getSizeInCellsY();
    origin_x_ = costmap_->getOriginX(), origin_y_ = costmap_->getOriginY();
    resolution_ = costmap_->getResolution();

    ros::NodeHandle nh = ros::NodeHandle("~/" + name);

    nh.param("convert_offset", convert_offset_, 0.0);

    nh.param("p_window", p_window_, 0.2);
    nh.param("o_window", o_window_, 1.0);

    nh.param("p_precision", p_precision_, 0.2);
    nh.param("o_precision", o_precision_, 0.5);

    nh.param("max_v", max_v_, 0.5);
    nh.param("min_v", min_v_, 0.0);
    nh.param("max_v_inc", max_v_inc_, 0.5);

    nh.param("max_w", max_w_, 1.57);
    nh.param("min_w", min_w_, 0.0);
    nh.param("max_w_inc", max_w_inc_, 1.57);

    nh.param("s_window", s_window_, 5);

    nh.param("zeta", zeta_, 1.0);
    nh.param("eta", eta_, 3000.0);

    nh.param("/move_base/controller_frequency", controller_freqency_, 10.0);
    d_t_ = 1 / controller_freqency_;

    hist_nf_.clear();

    odom_helper_ = new base_local_planner::OdometryHelperRos("/odom");
    target_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/target_pose", 10);
    current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);

    ROS_INFO("APF planner initialized!");
  }
  else
  {
    ROS_WARN("APF planner has already been initialized.");
  }
}

/**
 * @brief  Set the plan that the controller is following
 * @param orig_global_plan the plan to pass to the controller
 * @return  true if the plan was updated successfully, else false
 */
bool APFPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
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
  plan_index_ = std::min(4, (int)global_plan_.size() - 1);

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
bool APFPlanner::isGoalReached()
{
  if (!initialized_)
  {
    ROS_ERROR("APF planner has not been initialized");
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
bool APFPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_)
  {
    ROS_ERROR("APF planner has not been initialized");
    return false;
  }

  // current pose
  geometry_msgs::PoseStamped current_ps_odom;
  costmap_ros_->getRobotPose(current_ps_odom);

  // transform into map
  tf_->transform(current_ps_odom, current_ps_, map_frame_);

  x_ = current_ps_.pose.position.x;
  y_ = current_ps_.pose.position.y;
  theta_ = tf2::getYaw(current_ps_.pose.orientation);  // [-pi, pi]

  double theta_d, theta_dir, theta_trj, v_inc;
  double b_x_d, b_y_d;  // desired x, y in base frame
  double e_theta;
  Eigen::Vector2d attr_force, rep_force, net_force;

  rep_force = getRepulsiveForce();

  while (plan_index_ < global_plan_.size())
  {
    target_ps_ = global_plan_[plan_index_];
    double x_d = target_ps_.pose.position.x;
    double y_d = target_ps_.pose.position.y;

    attr_force = getAttractiveForce(target_ps_, x_, y_);
    net_force = zeta_ * attr_force + eta_ * rep_force;

    // from robot to plan point
    theta_d = atan2(net_force[1], net_force[0]);  // [-pi, pi]
    regularizeAngle(theta_d);

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_d);
    tf2::convert(q, target_ps_.pose.orientation);

    e_theta = theta_d - theta_;
    regularizeAngle(e_theta);

    v_inc = net_force.norm() * cos(e_theta);

    // transform from map into base_frame
    getTransformedPosition(target_ps_, b_x_d, b_y_d);

    if (std::hypot(b_x_d, b_y_d) > p_window_ || std::fabs(e_theta) > o_window_)
      break;

    ++plan_index_;
  }

  // odometry observation - getting robot velocities in robot frame
  nav_msgs::Odometry base_odom;
  odom_helper_->getOdom(base_odom);

  // position reached
  if (getGoalPositionDistance(global_plan_.back(), x_, y_) < p_precision_)
  {
    e_theta = goal_rpy_[2] - theta_;
    regularizeAngle(e_theta);

    // orientation reached
    if (std::fabs(e_theta) < o_precision_)
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;

      goal_reached_ = true;
    }
    // orientation not reached
    else
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = AngularAPFController(base_odom, goal_rpy_[2], theta_);
    }
  }
  // large angle, turn first
  else if (std::fabs(e_theta) > M_PI_2)
  {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = AngularAPFController(base_odom, theta_d, theta_);
  }
  // posistion not reached
  else
  {
    // smoothing the net force with historical net forces in the smoothing window
    if (!hist_nf_.empty() && hist_nf_.size() >= s_window_)
    {
      hist_nf_.pop_front();
    }
    hist_nf_.push_back(net_force);
    net_force = Eigen::Vector2d(0.0, 0.0);
    for (int i = 0; i < hist_nf_.size(); ++i)
    {
      net_force += hist_nf_[i];
    }
    net_force /= hist_nf_.size();

    // reset the smoothed e_theta and v_inc
    // from robot to plan point
    theta_d = atan2(net_force[1], net_force[0]);  // [-pi, pi]
    regularizeAngle(theta_d);

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_d);
    tf2::convert(q, target_ps_.pose.orientation);

    e_theta = theta_d - theta_;
    regularizeAngle(e_theta);

    v_inc = net_force.norm() * cos(e_theta);

    cmd_vel.linear.x = LinearAPFController(base_odom, v_inc);
    cmd_vel.angular.z = AngularAPFController(base_odom, theta_d, theta_);
  }

  target_pose_pub_.publish(target_ps_);
  current_pose_pub_.publish(current_ps_);

  return true;
}

/**
 * @brief APF controller in linear
 * @param base_odometry odometry of the robot, to get velocity
 * @param b_x_d         desired x in body frame
 * @param b_y_d         desired y in body frame
 * @return  linear velocity
 */
double APFPlanner::LinearAPFController(nav_msgs::Odometry& base_odometry, double v_inc)
{
  double v = std::hypot(base_odometry.twist.twist.linear.x, base_odometry.twist.twist.linear.y);

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
 * @brief APF controller in angular
 * @param base_odometry odometry of the robot, to get velocity
 * @param theta_d       desired theta
 * @param theta         current theta
 * @return  angular velocity
 */
double APFPlanner::AngularAPFController(nav_msgs::Odometry& base_odometry, double theta_d, double theta)
{
  double e_theta = theta_d - theta;
  regularizeAngle(e_theta);

  double w_d = e_theta / d_t_;
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
 * @brief Get the distance to the goal
 * @param goal_ps global goal PoseStamped
 * @param x       global current x
 * @param y       global current y
 * @return the distance to the goal
 */
double APFPlanner::getGoalPositionDistance(const geometry_msgs::PoseStamped& goal_ps, double x, double y)
{
  return std::hypot(x - goal_ps.pose.position.x, y - goal_ps.pose.position.y);
}

/**
   * @brief Get the attractive force of APF
   * @param ps      global target PoseStamped
   * @param x       global current x
   * @param y       global current y
   * @return the attractive force
 */
Eigen::Vector2d APFPlanner::getAttractiveForce(const geometry_msgs::PoseStamped& ps, double x, double y)
{
  Eigen::Vector2d attr_force = Eigen::Vector2d(ps.pose.position.x - x, ps.pose.position.y - y);
  return attr_force / attr_force.norm();
}

/**
   * @brief Get the repulsive force of APF
   * @return the repulsive force
 */
Eigen::Vector2d APFPlanner::getRepulsiveForce()
{
  Eigen::Vector2d rep_force(0.0, 0.0);
  int mx, my;
  if (!_worldToMap(0.0, 0.0, mx, my))
  {
    ROS_WARN("Failed to convert the robot's coordinates from world map to costmap.");
    return rep_force;
  }

  double current_cost = costmap_char_[mx + nx_ * my];

  if (current_cost >= LETHAL_COST)
  {
    ROS_WARN("The robot's position is abnormal! Are you sure the robot has been properly localized?");
    return rep_force;
  }

  // obtain the distance between the robot and obstacles directly through costmap
  double dist = LETHAL_COST - current_cost;
  double k = ( 1.0 / (LETHAL_COST * resolution_) - 1.0 / (dist * resolution_) ) / ((dist * resolution_) * (dist * resolution_));
  double next_x = costmap_char_[std::min(mx + 1, (int)nx_ - 1) + nx_ * my];
  double prev_x = costmap_char_[std::max(mx - 1, 0) + nx_ * my];
  double next_y = costmap_char_[mx + nx_ * std::min(my + 1, (int)ny_ - 1)];
  double prev_y = costmap_char_[mx + nx_ * std::max(my - 1, 0)];
  Eigen::Vector2d grad_dist(
      ( (next_x - prev_x) / 2 ) / dist,
      ( (next_y - prev_y) / 2 ) / dist
      );

  rep_force = k * grad_dist;

  return rep_force;
}

/**
 * @brief Get the Euler Angles from PoseStamped
 * @param ps  PoseStamped to calculate
 * @return  roll, pitch and yaw in XYZ order
 */
std::vector<double> APFPlanner::getEulerAngles(geometry_msgs::PoseStamped& ps)
{
  std::vector<double> EulerAngles;
  EulerAngles.resize(3, 0);

  tf2::Quaternion q(ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w);
  tf2::Matrix3x3 m(q);

  m.getRPY(EulerAngles[0], EulerAngles[1], EulerAngles[2]);
  return EulerAngles;
}

/**
 * @brief Transform pose to body frame
 * @param src   src PoseStamped, the object to transform
 * @param x     result x
 * @param y     result y
 */
void APFPlanner::getTransformedPosition(geometry_msgs::PoseStamped& src, double& x, double& y)
{
  geometry_msgs::PoseStamped dst;

  src.header.stamp = ros::Time(0);
  tf_->transform(src, dst, base_frame_);

  x = dst.pose.position.x;
  y = dst.pose.position.y;
}

/**
 * @brief Regularize angle to [-pi, pi]
 * @param angle the angle (rad) to regularize
 */
void APFPlanner::regularizeAngle(double& angle)
{
  angle = angle - 2.0 * M_PI * std::floor((angle + M_PI) / (2.0 * M_PI));
}

/**
 * @brief Tranform from world map(x, y) to costmap(x, y)
 * @param mx  costmap x
 * @param my  costmap y
 * @param wx  world map x
 * @param wy  world map y
 * @return true if successfull, else false
 */
bool APFPlanner::_worldToMap(double wx, double wy, int& mx, int& my)
{
  if (wx < origin_x_ || wy < origin_y_)
  return false;

  mx = (int) ( (wx - origin_x_) / resolution_ - convert_offset_ );
  my = (int) ( (wy - origin_y_) / resolution_ - convert_offset_ );
  if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
  return true;

  return false;
}
}  // namespace apf_planner