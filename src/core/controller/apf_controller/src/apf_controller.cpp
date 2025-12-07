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

#include "common/util/log.h"
#include "common/util/visualizer.h"
#include "common/math/math_helper.h"
#include "common/geometry/angles.h"
#include "controller/apf_controller.h"

PLUGINLIB_EXPORT_CLASS(rmp::controller::APFController, nav_core::BaseLocalPlanner)

using namespace rmp::common::math;
using namespace rmp::common::geometry;

namespace rmp {
namespace controller {
static constexpr double kLargeAngleRad = M_PI_2;
static constexpr double kInflationRadiusM = 3.0;

/**
 * @brief Construct a new APFController object
 */
APFController::APFController() : initialized_(false), tf_(nullptr), goal_reached_(false) {
}

/**
 * @brief Construct a new APFController object
 */
APFController::APFController(std::string name, tf2_ros::Buffer* tf,
                             costmap_2d::Costmap2DROS* costmap_ros)
  : APFController() {
  initialize(name, tf, costmap_ros);
}

/**
 * @brief Destroy the APFController object
 */
APFController::~APFController() {
}

/**
 * @brief Initialization of the local planner
 * @param name        the name to give this instance of the trajectory planner
 * @param tf          a pointer to a transform listener
 * @param costmap_ros the cost map to use for assigning costs to trajectories
 */
void APFController::initialize(std::string name, tf2_ros::Buffer* tf,
                               costmap_2d::Costmap2DROS* costmap_ros) {
  if (!initialized_) {
    initialized_ = true;
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    ros::NodeHandle nh = ros::NodeHandle("~/" + name);
    apf_config_ = config_.apf_controller();

    hist_nf_.clear();

    target_pt_pub_ = nh.advertise<visualization_msgs::Marker>("/target_point", 1);
    current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);
    potential_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/potential_map", 10);

    R_INFO << "APF controller initialized!";
  } else {
    R_WARN << "APF controller has already been initialized.";
  }
}

/**
 * @brief  Set the plan that the controller is following
 * @param orig_global_plan the plan to pass to the controller
 * @return  true if the plan was updated successfully, else false
 */
bool APFController::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
  if (!initialized_) {
    ROS_ERROR(
        "This planner has not been initialized, please call initialize() before using "
        "this planner");
    return false;
  }

  R_INFO << "Got new plan";

  // set new plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // reset plan parameters
  if (goal_x_ != global_plan_.back().pose.position.x ||
      goal_y_ != global_plan_.back().pose.position.y) {
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
bool APFController::isGoalReached() {
  if (!initialized_) {
    ROS_ERROR("APF controller has not been initialized");
    return false;
  }

  if (goal_reached_) {
    R_INFO << "GOAL Reached!";
    return true;
  }
  return false;
}

/**
 * @brief Given the current position, orientation, and velocity of the robot, compute the
 * velocity commands
 * @param cmd_vel will be filled with the velocity command to be passed to the robot base
 * @return  true if a valid trajectory was found, else false
 */
bool APFController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  if (!initialized_) {
    ROS_ERROR("APF controller has not been initialized");
    return false;
  }

  // odometry observation - getting robot velocities in robot frame
  nav_msgs::Odometry base_odom;
  odom_helper_->getOdom(base_odom);

  // get robot position in global frame
  geometry_msgs::PoseStamped robot_pose_odom, robot_pose_map;
  costmap_ros_->getRobotPose(robot_pose_odom);
  transformPose(tf_, config_.map_frame(), robot_pose_odom, robot_pose_map);

  // transform global plan to robot frame
  std::vector<geometry_msgs::PoseStamped> prune_plan = prune(robot_pose_map);

  // calculate look-ahead distance
  double vt = std::hypot(base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y);
  double wt = base_odom.twist.twist.angular.z;
  double L = clamp(std::fabs(vt) * apf_config_.lookahead_time(),
                   apf_config_.min_lookahead_dist(), apf_config_.max_lookahead_dist());

  // get the particular point on the path at the lookahead distance
  double kappa;
  Point3d lookahead_pt;
  getLookAheadPoint(L, robot_pose_map, prune_plan, &lookahead_pt, &kappa);

  // current angle
  double theta = tf2::getYaw(robot_pose_map.pose.orientation);  // [-pi, pi]

  // calculate forces
  Vec2d curr_pt(robot_pose_odom.pose.position.x, robot_pose_odom.pose.position.y);
  Vec2d rep_force = getRepulsiveForce(curr_pt);
  Vec2d attr_force =
      getAttractiveForce(curr_pt, Vec2d(lookahead_pt.x(), lookahead_pt.y()));
  Vec2d net_force = apf_config_.weight_attractive_force() * attr_force +
                    apf_config_.weight_repulsive_force() * rep_force;

  // smoothing the net force with historical net forces in the smoothing window
  if (!hist_nf_.empty() && hist_nf_.size() >= apf_config_.smooth_window()) {
    hist_nf_.pop_front();
  }
  hist_nf_.push_back(net_force);
  net_force = Vec2d(0.0, 0.0);
  for (int i = 0; i < hist_nf_.size(); ++i) {
    net_force += hist_nf_[i];
  }
  net_force /= hist_nf_.size();

  // set the smoothed new_v
  Vec2d new_v(base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y);
  new_v += net_force;
  new_v /= new_v.length();
  new_v *= config_.max_linear_velocity();

  // set the desired angle and the angle error
  double theta_d = new_v.angle();  // [-pi, pi]
  double e_theta = normalizeAngle(theta_d - theta);

  // position reached
  if (shouldRotateToGoal(robot_pose_map, global_plan_.back())) {
    e_theta = normalizeAngle(goal_theta_ - theta);

    // orientation reached
    if (!shouldRotateToPath(std::fabs(e_theta))) {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      goal_reached_ = true;
    }
    // orientation not reached
    else {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = angularRegularization(wt, e_theta / control_dt_);
    }
  }
  // large angle, turn first
  else if (std::fabs(e_theta) > kLargeAngleRad) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = angularRegularization(wt, e_theta / control_dt_);
  }
  // posistion not reached
  else {
    cmd_vel.linear.x = linearRegularization(vt, new_v.length());
    cmd_vel.angular.z = angularRegularization(wt, e_theta / control_dt_);
  }

  // visualization
  const auto& visualizer = rmp::common::util::VisualizerPtr::Instance();

  // publish lookahead pose
  Points3d points;
  points.emplace_back(std::move(lookahead_pt));
  visualizer->publishPoints(points, target_pt_pub_, "map", "lookahead",
                            rmp::common::util::Visualizer::RED, 0.3);

  // publish robot pose
  current_pose_pub_.publish(robot_pose_map);
  // publishPotentialMap(curr_pt, Vec2d(lookahead_pt.x(), lookahead_pt.y()));

  return true;
}

/**
 * @brief Get the attractive force of APF
 * @param ps      global target PointStamped
 * @return the attractive force
 */
Vec2d APFController::getAttractiveForce(const Vec2d& current_pt, const Vec2d& target_pt) {
  Vec2d attr_force = target_pt - current_pt;
  attr_force.normalize();
  return attr_force;
}

/**
 * @brief Get the repulsive force of APF
 * @return the repulsive force
 */
Vec2d APFController::getRepulsiveForce(const Vec2d& current_pt) {
  Vec2d rep_force = current_pt;
  int mx, my;
  if (!worldToMap(rep_force.x(), rep_force.y(), mx, my)) {
    ROS_WARN("Failed to convert the robot's coordinates from world map to costmap.");
    return rep_force;
  }

  int nx = costmap_ros_->getCostmap()->getSizeInCellsX();
  int ny = costmap_ros_->getCostmap()->getSizeInCellsY();
  double current_cost = costmap_ros_->getCostmap()->getCharMap()[mx + nx * my];
  if (current_cost >= costmap_2d::LETHAL_OBSTACLE ||
      current_cost < costmap_2d::FREE_SPACE) {
    ROS_WARN(
        "The cost %.0lf of robot's position is out of bound! Are you sure the robot has "
        "been"
        " properly localized and the cost bound is right?",
        current_cost);
    return rep_force;
  }

  // obtain the distance between the robot and obstacles directly through costmap
  // mapping from cost_ub_ to distance 0
  // mapping from cost_lb_ to distance 1  (normalized)
  double bound_diff =
      static_cast<double>(costmap_2d::LETHAL_OBSTACLE - costmap_2d::FREE_SPACE);
  double dist =
      (static_cast<double>(costmap_2d::LETHAL_OBSTACLE) - current_cost) / bound_diff;
  double k = (1.0 - 1.0 / dist) / (dist * dist);
  double next_x =
      costmap_ros_->getCostmap()->getCharMap()[std::min(mx + 1, nx - 1) + nx * my];
  double prev_x = costmap_ros_->getCostmap()->getCharMap()[std::max(mx - 1, 0) + nx * my];
  double next_y =
      costmap_ros_->getCostmap()->getCharMap()[mx + nx * std::min(my + 1, ny - 1)];
  double prev_y = costmap_ros_->getCostmap()->getCharMap()[mx + nx * std::max(my - 1, 0)];
  Vec2d grad_dist((next_x - prev_x) / (2.0 * bound_diff),
                  (next_y - prev_y) / (2.0 * bound_diff));

  rep_force = k * grad_dist;

  return rep_force;
}

/**
 * @brief Callback function of costmap_sub_ to publish /potential_map topic
 * @param msg the message received from topic /move_base/local_costmap/costmap
 */
void APFController::publishPotentialMap(const Vec2d& current_pt, const Vec2d& target_pt) {
  const double attr_scale =
      apf_config_.weight_attractive_force() /
      (apf_config_.weight_attractive_force() + apf_config_.weight_repulsive_force());
  const double rep_scale = 1.0 - attr_scale;
  double current_cost, dist_to_target, dist_to_obstacles, inflation_radius, scaled_attr,
      scaled_rep;
  double bound_diff =
      static_cast<double>(costmap_2d::LETHAL_OBSTACLE - costmap_2d::FREE_SPACE);
  int tx, ty;  // costmap coordinates of target point

  // calculate costmap coordinates from world coordinates (maybe out of bound)
  int nx = costmap_ros_->getCostmap()->getSizeInCellsX();
  int ny = costmap_ros_->getCostmap()->getSizeInCellsY();
  double origin_x = costmap_ros_->getCostmap()->getOriginX();
  double origin_y = costmap_ros_->getCostmap()->getOriginY();
  double resolution = costmap_ros_->getCostmap()->getResolution();
  tx = static_cast<int>((target_pt.x() - current_pt.x() - origin_x) / resolution);
  ty = static_cast<int>((target_pt.y() - current_pt.y() - origin_y) / resolution);

  // calculate distance from the robot to target (on the scale of costmap)
  dist_to_target = std::hypot(tx - static_cast<int>((-origin_x) / resolution),
                              ty - static_cast<int>((-origin_y) / resolution));

  // the costmap inflation radius of obstacles (on the scale of costmap)
  inflation_radius = kInflationRadiusM / resolution;

  nav_msgs::OccupancyGrid potential_map;
  potential_map.data.resize(nx * ny);
  for (int y = 0; y < ny; ++y) {
    for (int x = 0; x < nx; ++x) {
      // temp variables
      current_cost =
          costmap_ros_->getCostmap()->getCharMap()[x + nx * y];  // cost of the cell
      dist_to_obstacles =
          (static_cast<double>(costmap_2d::LETHAL_OBSTACLE) - current_cost) /
          bound_diff;  // distance from cell to obstacles
                       // (normalized scale, i.e., within the range of [0,1])

      // to calculate the two scaled force potential fields
      scaled_attr =
          attr_scale *
          ((std::hypot(tx - x, ty - y) - dist_to_target) / inflation_radius / 2.0 + 0.5);
      scaled_rep = rep_scale * std::pow(1.0 / dist_to_obstacles - 1.0, 2);

      // sum two potential fields to calculate the net potential field
      potential_map.data[x + nx * y] =
          std::max(0, std::min(100, static_cast<int>(( scaled_rep) *
                                                     100)));  // [0, 100] is the range of
                                                              // rviz costmap
    }
  }

  // publish potential map
  potential_map.header.stamp = ros::Time::now();
  potential_map.header.frame_id = costmap_ros_->getGlobalFrameID();
  potential_map.info.height = ny;
  potential_map.info.width = nx;
  potential_map.info.resolution = resolution;
  potential_map.info.origin.position.x = origin_x;
  potential_map.info.origin.position.y = origin_y;
  potential_map_pub_.publish(potential_map);
}
}  // namespace controller
}  // namespace rmp