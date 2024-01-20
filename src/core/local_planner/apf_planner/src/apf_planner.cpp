/**
 * *********************************************************
 *
 * @file: apf_planner.cpp
 * @brief: Contains the Artificial Potential Field (APF) local planner class
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

#include "apf_planner.h"

PLUGINLIB_EXPORT_CLASS(apf_planner::APFPlanner, nav_core::BaseLocalPlanner)

namespace apf_planner
{
/**
 * @brief Construct a new APFPlanner object
 */
APFPlanner::APFPlanner()
  : initialized_(false), tf_(nullptr), goal_reached_(false), plan_index_(0)  //, costmap_ros_(nullptr)
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
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    local_costmap_ = costmap->getCharMap();

    // set costmap properties
    setSize(costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
    setOrigin(costmap->getOriginX(), costmap->getOriginY());
    setResolution(costmap->getResolution());

    ros::NodeHandle nh = ros::NodeHandle("~/" + name);

    nh.param("convert_offset", convert_offset_, 0.0);

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

    nh.param("cost_ub", cost_ub_, (int)lethal_cost_);
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
                                                         &APFPlanner::publishPotentialMap, this);

    ROS_INFO("APF planner initialized!");
  }
  else
    ROS_WARN("APF planner has already been initialized.");
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
  plan_index_ = std::min(1, (int)global_plan_.size() - 1);
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
  Eigen::Vector2d attr_force, rep_force, net_force;
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
  net_force = Eigen::Vector2d(0.0, 0.0);
  for (int i = 0; i < hist_nf_.size(); ++i)
    net_force += hist_nf_[i];
  net_force /= hist_nf_.size();

  // set the smoothed new_v
  Eigen::Vector2d new_v = Eigen::Vector2d(base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y);
  new_v += net_force;
  new_v /= new_v.norm();
  new_v *= max_v_;

  // set the desired angle and the angle error
  double theta_d = regularizeAngle(atan2(new_v[1], new_v[0]));  // [-pi, pi]
  tf2::Quaternion q;
  q.setRPY(0, 0, theta_d);
  tf2::convert(q, target_ps_.pose.orientation);
  double e_theta = regularizeAngle(theta_d - theta);

  // position reached
  if (shouldRotateToGoal(current_ps_, global_plan_.back()))
  {
    e_theta = regularizeAngle(goal_rpy_.z() - theta);

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
    cmd_vel.linear.x = linearRegularization(base_odom, new_v.norm());
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
Eigen::Vector2d APFPlanner::getAttractiveForce(const geometry_msgs::PoseStamped& ps)
{
  double x = current_ps_.pose.position.x;
  double y = current_ps_.pose.position.y;
  Eigen::Vector2d attr_force = Eigen::Vector2d(ps.pose.position.x - x, ps.pose.position.y - y);
  return attr_force / attr_force.norm();  // normalization
}

/**
 * @brief Get the repulsive force of APF
 * @return the repulsive force
 */
Eigen::Vector2d APFPlanner::getRepulsiveForce()
{
  Eigen::Vector2d rep_force(0.0, 0.0);
  int mx, my;
  if (!worldToMap(0.0, 0.0, mx, my))
  {
    ROS_WARN("Failed to convert the robot's coordinates from world map to costmap.");
    return rep_force;
  }

  double current_cost = local_costmap_[mx + nx_ * my];

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
  double next_x = local_costmap_[std::min(mx + 1, (int)nx_ - 1) + nx_ * my];
  double prev_x = local_costmap_[std::max(mx - 1, 0) + nx_ * my];
  double next_y = local_costmap_[mx + nx_ * std::min(my + 1, (int)ny_ - 1)];
  double prev_y = local_costmap_[mx + nx_ * std::max(my - 1, 0)];
  Eigen::Vector2d grad_dist((next_x - prev_x) / (2.0 * bound_diff), (next_y - prev_y) / (2.0 * bound_diff));

  rep_force = k * grad_dist;

  return rep_force;
}

/**
 * @brief Callback function of costmap_sub_ to publish /potential_map topic
 * @param msg the message received from topic /move_base/local_costmap/costmap
 */
void APFPlanner::publishPotentialMap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  double attr_scale = zeta_ / (zeta_ + eta_),
         rep_scale = eta_ / (zeta_ + eta_);  // the two potential scales sum up to 1
  double current_cost, dist_to_target, dist_to_obstacles, inflation_radius, scaled_attr, scaled_rep;
  double bound_diff = cost_ub_ - cost_lb_;
  int tx, ty;  // costmap coordinates of target point

  // calculate costmap coordinates from world coordinates (maybe out of bound)
  tx = (int)((target_ps_.pose.position.x - current_ps_.pose.position.x - origin_x_) / resolution_ - convert_offset_);
  ty = (int)((target_ps_.pose.position.y - current_ps_.pose.position.y - origin_y_) / resolution_ - convert_offset_);

  // calculate distance from the robot to target (on the scale of costmap)
  dist_to_target = std::hypot(tx - (int)((-origin_x_) / resolution_ - convert_offset_),
                              ty - (int)((-origin_y_) / resolution_ - convert_offset_));

  // the costmap inflation radius of obstacles (on the scale of costmap)
  inflation_radius = inflation_radius_ / resolution_;

  potential_map_ = *msg;
  for (int y = 0; y < ny_; ++y)
  {
    for (int x = 0; x < nx_; ++x)
    {
      // temp variables
      current_cost = local_costmap_[x + nx_ * y];  // cost of the cell
      dist_to_obstacles =
          (cost_ub_ - current_cost) / bound_diff;  // distance from cell to obstacles
                                                   // (normalized scale, i.e., within the range of [0,1])

      // to calculate the two scaled force potential fields
      scaled_attr = attr_scale * ((std::hypot(tx - x, ty - y) - dist_to_target) / inflation_radius / 2.0 + 0.5);
      scaled_rep = rep_scale * std::pow(1.0 / dist_to_obstacles - 1.0, 2);

      // sum two potential fields to calculate the net potential field
      potential_map_.data[x + nx_ * y] =
          std::max(0, std::min(100, (int)((scaled_attr + scaled_rep) * 100)));  // [0, 100] is the range of rviz costmap
    }
  }

  // publish potential map
  potential_map_pub_.publish(potential_map_);
}
}  // namespace apf_planner