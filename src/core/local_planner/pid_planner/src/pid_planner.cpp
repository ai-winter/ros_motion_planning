/**
 * *********************************************************
 *
 * @file: pid_planner.cpp
 * @brief: Contains the Proportional–Integral–Derivative (PID) controller local planner class
 * @author: Yang Haodong, Guo Zhanyu, Wu Maojia
 * @date: 2024-01-20
 * @version: 1.2
 *
 * Copyright (c) 2024, Yang Haodong, Guo Zhanyu, Wu Maojia.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <pluginlib/class_list_macros.h>

#include "pid_planner.h"

PLUGINLIB_EXPORT_CLASS(pid_planner::PIDPlanner, nav_core::BaseLocalPlanner)

namespace pid_planner
{
/**
 * @brief Construct a new PIDPlanner object
 */
PIDPlanner::PIDPlanner() : initialized_(false), goal_reached_(false), tf_(nullptr), costmap_ros_(nullptr)
{
}

/**
 * @brief Construct a new PIDPlanner object
 */
PIDPlanner::PIDPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) : PIDPlanner()
{
  initialize(name, tf, costmap_ros);
}

/**
 * @brief Destroy the PIDPlanner object
 */
PIDPlanner::~PIDPlanner()
{
}

/**
 * @brief Initialization of the local planner
 * @param name        the name to give this instance of the trajectory planner
 * @param tf          a pointer to a transform listener
 * @param costmap_ros the cost map to use for assigning costs to trajectories
 */
void PIDPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_)
  {
    initialized_ = true;
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    ros::NodeHandle nh = ros::NodeHandle("~/" + name);

    // base
    nh.param("goal_dist_tolerance", goal_dist_tol_, 0.2);
    nh.param("rotate_tolerance", rotate_tol_, 0.5);
    nh.param("base_frame", base_frame_, base_frame_);
    nh.param("map_frame", map_frame_, map_frame_);

    // lookahead
    nh.param("lookahead_time", lookahead_time_, 0.5);
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

    // PID parameters
    nh.param("k_v_p", k_v_p_, 1.00);
    nh.param("k_v_i", k_v_i_, 0.01);
    nh.param("k_v_d", k_v_d_, 0.10);
    nh.param("k_w_p", k_w_p_, 1.00);
    nh.param("k_w_i", k_w_i_, 0.01);
    nh.param("k_w_d", k_w_d_, 0.10);
    nh.param("k_theta", k_theta_, 0.5);
    e_v_ = i_v_ = 0.0;
    e_w_ = i_w_ = 0.0;

    double controller_freqency;
    nh.param("/move_base/controller_frequency", controller_freqency, 10.0);
    d_t_ = 1 / controller_freqency;

    target_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/target_pose", 10);
    current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);

    ROS_INFO("PID planner initialized!");
  }
  else
  {
    ROS_WARN("PID planner has already been initialized.");
  }
}

/**
 * @brief Set the plan that the controller is following
 * @param orig_global_plan the plan to pass to the controller
 * @return true if the plan was updated successfully, else false
 */
bool PIDPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
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

  // receive a plan for a new goal
  if (goal_x_ != global_plan_.back().pose.position.x || goal_y_ != global_plan_.back().pose.position.y)
  {
    goal_x_ = global_plan_.back().pose.position.x;
    goal_y_ = global_plan_.back().pose.position.y;
    goal_rpy_ = getEulerAngles(global_plan_.back());
    goal_reached_ = false;

    e_v_ = i_v_ = 0.0;
    e_w_ = i_w_ = 0.0;
  }

  return true;
}

/**
 * @brief Check if the goal pose has been achieved
 * @return true if achieved, false otherwise
 */
bool PIDPlanner::isGoalReached()
{
  if (!initialized_)
  {
    ROS_ERROR("PID planner has not been initialized");
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
 * @return true if a valid trajectory was found, else false
 */
bool PIDPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_)
  {
    ROS_ERROR("PID planner has not been initialized");
    return false;
  }

  // odometry observation - getting robot velocities in odom
  nav_msgs::Odometry base_odom;
  odom_helper_->getOdom(base_odom);

  // get robot position in map
  geometry_msgs::PoseStamped current_ps_odom, current_ps_map, target_ps_map;
  costmap_ros_->getRobotPose(current_ps_odom);
  transformPose(tf_, map_frame_, current_ps_odom, current_ps_map);

  // prune the global plan
  std::vector<geometry_msgs::PoseStamped> prune_plan = _prune(current_ps_map);

  // calculate look-ahead distance
  double vt = std::hypot(base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y);
  double wt = base_odom.twist.twist.angular.z;
  double L = _getLookAheadDistance(vt);

  // get the particular point on the path at the lookahead distance
  double theta_d, theta_dir, theta_trj;
  _getLookAheadPoint(L, current_ps_map, prune_plan, target_ps_map, theta_trj);
  theta_dir = atan2((target_ps_map.pose.position.y - current_ps_map.pose.position.y),
                    (target_ps_map.pose.position.x - current_ps_map.pose.position.x));
  theta_d = regularizeAngle((1 - k_theta_) * theta_trj + k_theta_ * theta_dir);
  tf2::Quaternion q;
  q.setRPY(0, 0, theta_d);
  tf2::convert(q, target_ps_map.pose.orientation);

  // current angle
  double theta = tf2::getYaw(current_ps_map.pose.orientation);  // [-pi, pi]

  // position reached
  if (shouldRotateToGoal(current_ps_map, global_plan_.back()))
  {
    double e_theta = regularizeAngle(goal_rpy_.z() - theta);

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
  // posistion not reached
  else
  {
    Eigen::Vector3d s(current_ps_map.pose.position.x, current_ps_map.pose.position.y, theta);    // current state
    Eigen::Vector3d s_d(target_ps_map.pose.position.x, target_ps_map.pose.position.y, theta_d);  // desired state
    Eigen::Vector2d u_r(vt, wt);                                                                 // refered input
    Eigen::Vector2d u = _pidControl(s, s_d, u_r);
    cmd_vel.linear.x = u[0];
    cmd_vel.angular.z = u[1];
  }

  // publish next target_ps_map pose
  target_ps_map.header.frame_id = "map";
  target_ps_map.header.stamp = ros::Time::now();
  target_pose_pub_.publish(target_ps_map);

  // publish robot pose
  current_ps_map.header.frame_id = "map";
  current_ps_map.header.stamp = ros::Time::now();
  current_pose_pub_.publish(current_ps_map);

  return true;
}

/**
 * @brief Execute PID control process (no model pid)
 * @param s   current state
 * @param s_d desired state
 * @param u_r refered input
 * @return u  control vector
 */
Eigen::Vector2d PIDPlanner::_pidControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector2d u_r)
{
  Eigen::Vector2d u;
  Eigen::Vector3d e = s_d - s;

  double e_x = e[0];
  double e_y = e[1];
  double e_theta = e[2];

  double v_d = std::hypot(e_x, e_y) / d_t_;
  double w_d = e_theta / d_t_;

  if (std::fabs(v_d) > max_v_)
    v_d = std::copysign(max_v_, v_d);
  if (std::fabs(w_d) > max_w_)
    w_d = std::copysign(max_w_, w_d);

  double e_v = v_d - u_r[0];
  double e_w = w_d - u_r[1];

  i_v_ += e_v * d_t_;
  i_w_ += e_w * d_t_;

  double d_v = (e_v - e_v_) / d_t_;
  double d_w = (e_w - e_w_) / d_t_;

  e_v_ = e_v;
  e_w_ = e_w;

  double v_inc = k_v_p_ * e_v + k_v_i_ * i_v_ + k_v_d_ * d_v;
  double w_inc = k_w_p_ * e_w + k_w_i_ * i_w_ + k_w_d_ * d_w;

  if (std::fabs(v_inc) > max_v_inc_)
    v_inc = std::copysign(max_v_inc_, v_inc);
  if (std::fabs(w_inc) > max_w_inc_)
    w_inc = std::copysign(max_w_inc_, w_inc);

  double v_cmd = u_r[0] + v_inc;
  if (std::fabs(v_cmd) > max_v_)
    v_cmd = std::copysign(max_v_, v_cmd);
  else if (std::fabs(v_cmd) < min_v_)
    v_cmd = std::copysign(min_v_, v_cmd);

  double w_cmd = u_r[1] + w_inc;
  if (std::fabs(w_cmd) > max_w_)
    w_cmd = std::copysign(max_w_, w_cmd);
  else if (std::fabs(w_cmd) < min_w_)
    w_cmd = std::copysign(min_w_, w_cmd);

  u[0] = v_cmd;
  u[1] = w_cmd;

  return u;
}

/**
 * @brief Prune the path, removing the waypoints that the robot has already passed and distant waypoints
 * @param robot_pose_global the robot's pose  [global]
 * @return pruned path
 */
std::vector<geometry_msgs::PoseStamped> PIDPlanner::_prune(const geometry_msgs::PoseStamped robot_pose_global)
{
  auto calPoseDistance = [](const geometry_msgs::PoseStamped& ps_1, const geometry_msgs::PoseStamped& ps_2) {
    return helper::dist(ps_1, ps_2);
  };

  auto closest_pose_upper_bound = helper::firstIntegratedDistance(
      global_plan_.begin(), global_plan_.end(), calPoseDistance, costmap_ros_->getCostmap()->getSizeInMetersX() / 2.0);

  // find the closest pose on the path to the robot
  auto transform_begin =
      helper::getMinFuncVal(global_plan_.begin(), closest_pose_upper_bound, [&](const geometry_msgs::PoseStamped& ps) {
        return calPoseDistance(robot_pose_global, ps);
      });

  // Transform the near part of the global plan into the robot's frame of reference.
  std::vector<geometry_msgs::PoseStamped> prune_path;
  for (auto it = transform_begin; it < global_plan_.end(); it++)
    prune_path.push_back(*it);

  // path pruning: remove the portion of the global plan that already passed so don't process it on the next iteration
  global_plan_.erase(std::begin(global_plan_), transform_begin);

  return prune_path;
}

/**
 * @brief Calculate the look-ahead distance with current speed dynamically
 * @param vt the current speed
 * @return L the look-ahead distance
 */
double PIDPlanner::_getLookAheadDistance(double vt)
{
  double lookahead_dist = fabs(vt) * lookahead_time_;
  return helper::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
}

/**
 * @brief find the point on the path that is exactly the lookahead distance away from the robot
 * @param lookahead_dist    the lookahead distance
 * @param robot_pose_global the robot's pose  [global]
 * @param prune_plan        the pruned plan
 */
void PIDPlanner::_getLookAheadPoint(double lookahead_dist, geometry_msgs::PoseStamped robot_pose_global,
                                    const std::vector<geometry_msgs::PoseStamped>& prune_plan,
                                    geometry_msgs::PoseStamped& target_ps_map, double& theta)
{
  double rx = robot_pose_global.pose.position.x;
  double ry = robot_pose_global.pose.position.y;

  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(prune_plan.begin(), prune_plan.end(), [&](const auto& ps) {
    return helper::dist(ps, robot_pose_global) >= lookahead_dist;
  });

  std::vector<geometry_msgs::PoseStamped>::const_iterator prev_pose_it;
  // If the no pose is not far enough, take the last pose
  if (goal_pose_it == prune_plan.end())
  {
    goal_pose_it = std::prev(prune_plan.end());
    prev_pose_it = std::prev(goal_pose_it);
    target_ps_map = *goal_pose_it;
  }
  else
  {
    // find the point on the line segment between the two poses
    // that is exactly the lookahead distance away from the robot pose (the origin)
    // This can be found with a closed form for the intersection of a segment and a circle
    // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside the circle,
    // and goal_pose is guaranteed to be outside the circle.
    prev_pose_it = std::prev(goal_pose_it);

    double px = prev_pose_it->pose.position.x;
    double py = prev_pose_it->pose.position.y;
    double gx = goal_pose_it->pose.position.x;
    double gy = goal_pose_it->pose.position.y;

    // transform to the robot frame so that the circle centers at (0,0)
    std::pair<double, double> prev_p(px - rx, py - ry);
    std::pair<double, double> goal_p(gx - rx, gy - ry);
    std::vector<std::pair<double, double>> i_points = helper::circleSegmentIntersection(prev_p, goal_p, lookahead_dist);

    target_ps_map.pose.position.x = i_points[0].first + rx;
    target_ps_map.pose.position.y = i_points[0].second + ry;
  }

  target_ps_map.header.frame_id = goal_pose_it->header.frame_id;
  target_ps_map.header.stamp = goal_pose_it->header.stamp;

  theta = atan2(goal_pose_it->pose.position.y - prev_pose_it->pose.position.y,
                goal_pose_it->pose.position.x - prev_pose_it->pose.position.x);
}

}  // namespace pid_planner