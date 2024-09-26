/**
 * *********************************************************
 *
 * @file: apf_controller.cpp
 * @brief: Contains the Artificial Potential Field (APF) local controller class
 * @author: Wu Maojia, Yang Haodong
 * @date: 2023-10-17
 * @version: 1.2
 *
 * Copyright (c) 2024, Wu Maojia, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <pluginlib/class_list_macros.h>

#include "controller/apf_controller.h"

PLUGINLIB_EXPORT_CLASS(rmp::controller::APFController, nav_core::BaseLocalPlanner)

namespace rmp
{
namespace controller
{
/**
 * @brief Construct a new APFController object
 */
APFController::APFController()
  : initialized_(false), tf_(nullptr), goal_reached_(false), plan_index_(0)  //, costmap_ros_(nullptr)
{
}

/**
 * @brief Construct a new APFController object
 */
APFController::APFController(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
  : APFController()
{
  initialize(name, tf, costmap_ros);
}

/**
 * @brief Destroy the APFController object
 */
APFController::~APFController()
{
}

/**
 * @brief Initialization of the local planner
 * @param name        the name to give this instance of the trajectory planner
 * @param tf          a pointer to a transform listener
 * @param costmap_ros the cost map to use for assigning costs to trajectories
 */
void APFController::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_)
  {
    initialized_ = true;
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    ros::NodeHandle nh = ros::NodeHandle("~/" + name);

    nh.param("p_window", p_window_, 0.5);

    nh.param("goal_dist_tolerance", goal_dist_tol_, 0.2);
    nh.param("rotate_tolerance", rotate_tol_, 0.5);

    nh.param("max_v", max_v_, 0.5);
    nh.param("min_v", min_v_, 0.0);
    nh.param("max_v_inc", max_v_inc_, 0.5);

    nh.param("max_w", max_w_, 1.57);
    nh.param("min_w", min_w_, 0.0);
    nh.param("max_w_inc", max_w_inc_, 1.57);

    nh.param("s_window", s_window_, 5);

    nh.param("zeta", zeta_, 1.0);
    nh.param("eta", eta_, 1.0);

    nh.param("cost_ub", cost_ub_, static_cast<int>(costmap_2d::LETHAL_OBSTACLE));
    nh.param("cost_lb", cost_lb_, 0);

    nh.param("inflation_radius", inflation_radius_, 1.0);

    nh.param("base_frame", base_frame_, base_frame_);
    nh.param("map_frame", map_frame_, map_frame_);

    double controller_freqency;
    nh.param("/move_base/controller_frequency", controller_freqency, 10.0);
    d_t_ = 1 / controller_freqency;

    hist_nf_.clear();

    target_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/target_pose", 10);
    current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);
    potential_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/potential_map", 10);
    costmap_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>("/move_base/local_costmap/costmap", 10,
                                                         &APFController::publishPotentialMap, this);

    ROS_INFO("APF controller initialized!");
  }
  else
    ROS_WARN("APF controller has already been initialized.");
}

/**
 * @brief  Set the plan that the controller is following
 * @param orig_global_plan the plan to pass to the controller
 * @return  true if the plan was updated successfully, else false
 */
bool APFController::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
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
  plan_index_ = std::min(1, (int)global_plan_.size() - 1);
  if (goal_x_ != global_plan_.back().pose.position.x || goal_y_ != global_plan_.back().pose.position.y)
  {
    goal_x_ = global_plan_.back().pose.position.x;
    goal_y_ = global_plan_.back().pose.position.y;
    goal_theta_ = getYawAngle(global_plan_.back());
    goal_reached_ = false;
  }

  return true;
}

/**
 * @brief  Check if the goal pose has been achieved
 * @return True if achieved, false otherwise
 */
bool APFController::isGoalReached()
{
  if (!initialized_)
  {
    ROS_ERROR("APF controller has not been initialized");
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
bool APFController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_)
  {
    ROS_ERROR("APF controller has not been initialized");
    return false;
  }

  // odometry observation - getting robot velocities in robot frame
  nav_msgs::Odometry base_odom;
  odom_helper_->getOdom(base_odom);

  // get robot position in global frame
  geometry_msgs::PoseStamped current_ps_odom;
  costmap_ros_->getRobotPose(current_ps_odom);
  transformPose(tf_, map_frame_, current_ps_odom, current_ps_);

  // current angle
  double theta = tf2::getYaw(current_ps_.pose.orientation);

  // compute the tatget pose and force at the current step
  rmp::common::geometry::Vec2d attr_force, rep_force, net_force;
  rep_force = getRepulsiveForce();
  while (plan_index_ < global_plan_.size())
  {
    target_ps_ = global_plan_[plan_index_];
    attr_force = getAttractiveForce(target_ps_);
    net_force = zeta_ * attr_force + eta_ * rep_force;

    // transform from map into base_frame
    geometry_msgs::PoseStamped dst;
    target_ps_.header.stamp = ros::Time(0);
    tf_->transform(target_ps_, dst, base_frame_);

    // desired x, y in base frame
    double b_x_d = dst.pose.position.x;
    double b_y_d = dst.pose.position.y;

    if (std::hypot(b_x_d, b_y_d) > p_window_)
      break;

    ++plan_index_;
  }

  // smoothing the net force with historical net forces in the smoothing window
  if (!hist_nf_.empty() && hist_nf_.size() >= s_window_)
    hist_nf_.pop_front();
  hist_nf_.push_back(net_force);
  net_force = rmp::common::geometry::Vec2d(0.0, 0.0);
  for (int i = 0; i < hist_nf_.size(); ++i)
    net_force += hist_nf_[i];
  net_force /= hist_nf_.size();

  // set the smoothed new_v
  rmp::common::geometry::Vec2d new_v(base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y);
  new_v += net_force;
  new_v /= new_v.length();
  new_v *= max_v_;

  // set the desired angle and the angle error
  double theta_d = new_v.angle();  // [-pi, pi]
  tf2::Quaternion q;
  q.setRPY(0, 0, theta_d);
  tf2::convert(q, target_ps_.pose.orientation);
  double e_theta = regularizeAngle(theta_d - theta);

  // position reached
  if (shouldRotateToGoal(current_ps_, global_plan_.back()))
  {
    e_theta = regularizeAngle(goal_theta_ - theta);

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
  // large angle, turn first
  else if (shouldRotateToPath(std::fabs(e_theta), M_PI_2))
  {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = angularRegularization(base_odom, e_theta / d_t_);
  }
  // posistion not reached
  else
  {
    cmd_vel.linear.x = linearRegularization(base_odom, new_v.length());
    cmd_vel.angular.z = angularRegularization(base_odom, e_theta / d_t_);
  }

  // publish next target_ps_ pose
  target_pose_pub_.publish(target_ps_);

  // publish robot pose
  current_pose_pub_.publish(current_ps_);

  return true;
}

/**
 * @brief Get the attractive force of APF
 * @param ps      global target PoseStamped
 * @return the attractive force
 */
rmp::common::geometry::Vec2d APFController::getAttractiveForce(const geometry_msgs::PoseStamped& ps)
{
  double x = current_ps_.pose.position.x;
  double y = current_ps_.pose.position.y;
  rmp::common::geometry::Vec2d attr_force(ps.pose.position.x - x, ps.pose.position.y - y);
  attr_force.normalize();
  return attr_force;
}

/**
 * @brief Get the repulsive force of APF
 * @return the repulsive force
 */
rmp::common::geometry::Vec2d APFController::getRepulsiveForce()
{
  double x = current_ps_.pose.position.x;
  double y = current_ps_.pose.position.y;
  rmp::common::geometry::Vec2d rep_force(x, y);
  int mx, my;
  if (!worldToMap(x, y, mx, my))
  {
    ROS_WARN("Failed to convert the robot's coordinates from world map to costmap.");
    return rep_force;
  }

  int nx = costmap_ros_->getCostmap()->getSizeInCellsX();
  int ny = costmap_ros_->getCostmap()->getSizeInCellsY();
  double current_cost = costmap_ros_->getCostmap()->getCharMap()[mx + nx * my];
  if (current_cost >= cost_ub_ || current_cost < cost_lb_)
  {
    ROS_WARN(
        "The cost %.0lf of robot's position is out of bound! Are you sure the robot has been"
        " properly localized and the cost bound is right?",
        current_cost);
    return rep_force;
  }

  // obtain the distance between the robot and obstacles directly through costmap
  // mapping from cost_ub_ to distance 0
  // mapping from cost_lb_ to distance 1  (normalized)
  double bound_diff = cost_ub_ - cost_lb_;
  double dist = (cost_ub_ - current_cost) / bound_diff;
  double k = (1.0 - 1.0 / dist) / (dist * dist);
  double next_x = costmap_ros_->getCostmap()->getCharMap()[std::min(mx + 1, nx - 1) + nx * my];
  double prev_x = costmap_ros_->getCostmap()->getCharMap()[std::max(mx - 1, 0) + nx * my];
  double next_y = costmap_ros_->getCostmap()->getCharMap()[mx + nx * std::min(my + 1, ny - 1)];
  double prev_y = costmap_ros_->getCostmap()->getCharMap()[mx + nx * std::max(my - 1, 0)];
  rmp::common::geometry::Vec2d grad_dist((next_x - prev_x) / (2.0 * bound_diff), (next_y - prev_y) / (2.0 * bound_diff));

  rep_force = k * grad_dist;

  return rep_force;
}

/**
 * @brief Callback function of costmap_sub_ to publish /potential_map topic
 * @param msg the message received from topic /move_base/local_costmap/costmap
 */
void APFController::publishPotentialMap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  double attr_scale = zeta_ / (zeta_ + eta_),
         rep_scale = eta_ / (zeta_ + eta_);  // the two potential scales sum up to 1
  double current_cost, dist_to_target, dist_to_obstacles, inflation_radius, scaled_attr, scaled_rep;
  double bound_diff = cost_ub_ - cost_lb_;
  int tx, ty;  // costmap coordinates of target point

  // calculate costmap coordinates from world coordinates (maybe out of bound)
  int nx = costmap_ros_->getCostmap()->getSizeInCellsX();
  int ny = costmap_ros_->getCostmap()->getSizeInCellsY();
  double origin_x = costmap_ros_->getCostmap()->getOriginX();
  double origin_y = costmap_ros_->getCostmap()->getOriginY();
  double resolution = costmap_ros_->getCostmap()->getResolution();
  tx = (int)((target_ps_.pose.position.x - current_ps_.pose.position.x - origin_x) / resolution);
  ty = (int)((target_ps_.pose.position.y - current_ps_.pose.position.y - origin_y) / resolution);

  // calculate distance from the robot to target (on the scale of costmap)
  dist_to_target = std::hypot(tx - (int)((-origin_x) / resolution), ty - (int)((-origin_y) / resolution));

  // the costmap inflation radius of obstacles (on the scale of costmap)
  inflation_radius = inflation_radius_ / resolution;

  potential_map_ = *msg;
  for (int y = 0; y < ny; ++y)
  {
    for (int x = 0; x < nx; ++x)
    {
      // temp variables
      current_cost = costmap_ros_->getCostmap()->getCharMap()[x + nx * y];  // cost of the cell
      dist_to_obstacles =
          (cost_ub_ - current_cost) / bound_diff;  // distance from cell to obstacles
                                                   // (normalized scale, i.e., within the range of [0,1])

      // to calculate the two scaled force potential fields
      scaled_attr = attr_scale * ((std::hypot(tx - x, ty - y) - dist_to_target) / inflation_radius / 2.0 + 0.5);
      scaled_rep = rep_scale * std::pow(1.0 / dist_to_obstacles - 1.0, 2);

      // sum two potential fields to calculate the net potential field
      potential_map_.data[x + nx * y] =
          std::max(0, std::min(100, (int)((scaled_attr + scaled_rep) * 100)));  // [0, 100] is the range of rviz costmap
    }
  }

  // publish potential map
  potential_map_pub_.publish(potential_map_);
}
}  // namespace controller
}  // namespace rmp