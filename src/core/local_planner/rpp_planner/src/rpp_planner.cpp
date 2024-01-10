/***********************************************************
 *
 * @file: rpp_planner.h
 * @breif: Contains the regulated_pure_pursuit (RPP) local planner class
 * @author: Yang Haodong
 * @update: 2024-1-8
 * @version: 1.0
 *
 * Copyright (c) 2024 Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include <pluginlib/class_list_macros.h>

#include "rpp_planner.h"
#include "math_helper.h"

PLUGINLIB_EXPORT_CLASS(rpp_planner::RPPPlanner, nav_core::BaseLocalPlanner)

namespace rpp_planner
{
/**
 * @brief Construct a new RPP planner object
 */
RPPPlanner::RPPPlanner() : initialized_(false), tf_(nullptr), costmap_ros_(nullptr), goal_reached_(false)
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
  std::vector<geometry_msgs::PoseStamped> prune_plan = _prune(robot_pose_map);

  // calculate look-ahead distance
  double vt = std::hypot(base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y);
  double L = _getLookAheadDistance(vt);

  // get the particular point on the path at the lookahead distance
  auto lookahead_pt = _getLookAheadPoint(L, robot_pose_map, prune_plan);

  // get the tracking curvature with goalahead point
  double lookahead_k = 2 * sin(_dphi(lookahead_pt, robot_pose_map)) / L;

  // calculate commands
  if (shouldRotateToGoal(robot_pose_map, global_plan_.back()))
  {
    double e_theta = goal_rpy_.z() - tf2::getYaw(robot_pose_map.pose.orientation);
    regularizeAngle(e_theta);

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
    double e_theta = _dphi(lookahead_pt, robot_pose_map);
    regularizeAngle(e_theta);

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
 * @brief calculate the look-ahead distance with current speed dynamically
 * @param vt  the current speed
 * @return L  the look-ahead distance
 */
double RPPPlanner::_getLookAheadDistance(double vt)
{
  double lookahead_dist = fabs(vt) * lookahead_time_;
  return helper::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
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
 * @brief find the point on the path that is exactly the lookahead distance away from the robot
 * @param lookahead_dist    the lookahead distance
 * @param robot_pose_global the robot's pose  [global]
 * @param prune_plan        the pruned plan
 * @return point            the lookahead point
 */
geometry_msgs::PointStamped RPPPlanner::_getLookAheadPoint(double lookahead_dist,
                                                           geometry_msgs::PoseStamped robot_pose_global,
                                                           const std::vector<geometry_msgs::PoseStamped>& prune_plan)
{
  geometry_msgs::PointStamped pt;

  double rx = robot_pose_global.pose.position.x;
  double ry = robot_pose_global.pose.position.y;

  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(prune_plan.begin(), prune_plan.end(), [&](const auto& ps) {
    return hypot(ps.pose.position.x - rx, ps.pose.position.y - ry) >= lookahead_dist;
  });

  // If the no pose is not far enough, take the last pose
  if (goal_pose_it == prune_plan.end())
  {
    goal_pose_it = std::prev(prune_plan.end());
    pt.point.x = goal_pose_it->pose.position.x;
    pt.point.y = goal_pose_it->pose.position.y;
  }
  else
  {
    // find the point on the line segment between the two poses
    // that is exactly the lookahead distance away from the robot pose (the origin)
    // This can be found with a closed form for the intersection of a segment and a circle
    // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside the circle,
    // and goal_pose is guaranteed to be outside the circle.
    auto prev_pose_it = std::prev(goal_pose_it);

    double px = prev_pose_it->pose.position.x;
    double py = prev_pose_it->pose.position.y;
    double gx = goal_pose_it->pose.position.x;
    double gy = goal_pose_it->pose.position.y;

    // transform to the robot frame so that the circle centers at (0,0)
    std::pair<double, double> prev_p(px - rx, py - ry);
    std::pair<double, double> goal_p(gx - rx, gy - ry);
    std::vector<std::pair<double, double>> i_points = helper::circleSegmentIntersection(prev_p, goal_p, lookahead_dist);

    pt.point.x = i_points[0].first + rx;
    pt.point.y = i_points[0].second + ry;
  }

  pt.header.frame_id = goal_pose_it->header.frame_id;
  pt.header.stamp = goal_pose_it->header.stamp;
  return pt;
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

/**
 * @brief Prune the path, removing the waypoints that the robot has already passed and distant waypoints
 * @param robot_pose_global the robot's pose  [global]
 * @return pruned path
 */
std::vector<geometry_msgs::PoseStamped> RPPPlanner::_prune(const geometry_msgs::PoseStamped robot_pose_global)
{
  auto calPoseDistance = [](const geometry_msgs::PoseStamped& ps_1, const geometry_msgs::PoseStamped& ps_2) {
    return std::hypot(ps_1.pose.position.x - ps_2.pose.position.x, ps_1.pose.position.y - ps_2.pose.position.y);
  };

  auto closest_pose_upper_bound = helper::firstIntegratedDistance(
      global_plan_.begin(), global_plan_.end(), calPoseDistance, costmap_ros_->getCostmap()->getSizeInMetersX() / 2.0);

  // find the closest pose on the path to the robot
  auto transform_begin =
      helper::getMinFuncVal(global_plan_.begin(), closest_pose_upper_bound, [&](const geometry_msgs::PoseStamped& ps) {
        return calPoseDistance(robot_pose_global, ps);
      });

  // // discard points on the plan that are outside the local costmap
  // const double max_costmap_extent =
  //     std::max(costmap_ros_->getCostmap()->getSizeInMetersX(), costmap_ros_->getCostmap()->getSizeInMetersY()) / 2.0;
  // auto transform_end = std::find_if(transform_begin, global_plan_.end(), [&](const auto& global_plan_pose) {
  //   return calPoseDistance(global_plan_pose, robot_pose_global) > max_costmap_extent;
  // });

  // Transform the near part of the global plan into the robot's frame of reference.
  std::vector<geometry_msgs::PoseStamped> prune_path;
  for (auto it = transform_begin; it < global_plan_.end(); it++)
    prune_path.push_back(*it);

  // path pruning: remove the portion of the global plan that already passed so don't process it on the next iteration
  global_plan_.erase(std::begin(global_plan_), transform_begin);

  return prune_path;
}

}  // namespace rpp_planner