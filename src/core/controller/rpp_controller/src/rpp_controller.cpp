/**
 * *********************************************************
 *
 * @file: rpp_controller.cpp
 * @brief: Contains the regulated_pure_pursuit (RPP) local controller class
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

#include "common/util/log.h"
#include "common/util/visualizer.h"
#include "common/math/math_helper.h"
#include "common/geometry/angles.h"
#include "controller/rpp_controller.h"

PLUGINLIB_EXPORT_CLASS(rmp::controller::RPPController, nav_core::BaseLocalPlanner)

using namespace rmp::common::math;
using namespace rmp::common::geometry;

namespace rmp {
namespace controller {
static constexpr double kLargeAngleRad = M_PI_2;

/**
 * @brief Construct a new RPP Controller object
 */
RPPController::RPPController() : initialized_(false), tf_(nullptr), goal_reached_(false) {
}

/**
 * @brief Construct a new RPP Controller object
 */
RPPController::RPPController(std::string name, tf2_ros::Buffer* tf,
                             costmap_2d::Costmap2DROS* costmap_ros)
  : RPPController() {
  initialize(name, tf, costmap_ros);
}

/**
 * @brief Destroy the RPP Controller object
 */
RPPController::~RPPController() {
}

/**
 * @brief Initialization of the local planner
 * @param name        the name to give this instance of the trajectory planner
 * @param tf          a pointer to a transform listener
 * @param costmap_ros the cost map to use for assigning costs to trajectories
 */
void RPPController::initialize(std::string name, tf2_ros::Buffer* tf,
                               costmap_2d::Costmap2DROS* costmap_ros) {
  if (!initialized_) {
    initialized_ = true;
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    ros::NodeHandle nh = ros::NodeHandle("~/" + name);
    rpp_config_ = config_.rpp_controller();

    target_pt_pub_ = nh.advertise<visualization_msgs::Marker>("/target_point", 1);
    current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);

    R_INFO << "RPP Controller initialized!";
  } else
    R_WARN << "RPP Controller has already been initialized.";
}

/**
 * @brief  Set the plan that the controller is following
 * @param orig_global_plan the plan to pass to the controller
 * @return  true if the plan was updated successfully, else false
 */
bool RPPController::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
  if (!initialized_) {
    R_ERROR << "This planner has not been initialized, please call initialize() before "
               "using this planner";
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
bool RPPController::isGoalReached() {
  if (!initialized_) {
    R_ERROR << "RPP Controller has not been initialized";
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
bool RPPController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  if (!initialized_) {
    R_ERROR << "RPP Controller has not been initialized";
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
  double L = clamp(std::fabs(vt) * rpp_config_.lookahead_time(),
                   rpp_config_.min_lookahead_dist(), rpp_config_.max_lookahead_dist());

  // get the particular point on the path at the lookahead distance
  double kappa;
  Point3d lookahead_pt;
  getLookAheadPoint(L, robot_pose_map, prune_plan, &lookahead_pt, &kappa);

  // calculate the relative angle between robot' yaw and relative lookahead vector
  const double dphi = std::atan2(lookahead_pt.y() - robot_pose_map.pose.position.y,
                                 lookahead_pt.x() - robot_pose_map.pose.position.x) -
                      tf2::getYaw(robot_pose_map.pose.orientation);

  // get the tracking curvature with goalahead point
  double lookahead_k = 2 * std::sin(dphi) / L;

  // calculate commands
  if (shouldRotateToGoal(robot_pose_map, global_plan_.back())) {
    double e_theta =
        normalizeAngle(goal_theta_ - tf2::getYaw(robot_pose_map.pose.orientation));

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
  } else {
    double e_theta = normalizeAngle(dphi);

    // large angle, turn first
    if (std::fabs(e_theta) > kLargeAngleRad) {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = angularRegularization(wt, e_theta / control_dt_);
    }

    // apply constraints
    else {
      double curv_vel =
          _applyCurvatureConstraint(config_.max_linear_velocity(), lookahead_k);
      double cost_vel = _applyObstacleConstraint(config_.max_linear_velocity());
      double v_d = std::min(curv_vel, cost_vel);
      v_d = _applyApproachConstraint(v_d, robot_pose_map, prune_plan);

      cmd_vel.linear.x = linearRegularization(vt, v_d);
      cmd_vel.angular.z = angularRegularization(wt, v_d * lookahead_k);
    }
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

  return true;
}

/**
 * @brief Applying curvature constraints to regularize the speed of robot turning
 * @param raw_linear_vel    the raw linear velocity of robot
 * @param curvature         the tracking curvature
 * @return reg_vel          the regulated velocity
 */
double RPPController::_applyCurvatureConstraint(const double raw_linear_vel,
                                                const double curvature) {
  const double radius = std::fabs(1.0 / curvature);
  return radius < rpp_config_.regulated_min_radius() ?
             raw_linear_vel * (radius / rpp_config_.regulated_min_radius()) :
             raw_linear_vel;
}

/**
 * @brief Applying obstacle constraints to regularize the speed of robot approaching
 * obstacles
 * @param raw_linear_vel    the raw linear velocity of robot
 * @return reg_vel          the regulated velocity
 */
double RPPController::_applyObstacleConstraint(const double raw_linear_vel) {
  int size_x = costmap_ros_->getCostmap()->getSizeInCellsX() / 2;
  int size_y = costmap_ros_->getCostmap()->getSizeInCellsY() / 2;
  double robot_cost =
      static_cast<double>(costmap_ros_->getCostmap()->getCost(size_x, size_y));

  if (robot_cost != static_cast<double>(costmap_2d::FREE_SPACE) &&
      robot_cost != static_cast<double>(costmap_2d::NO_INFORMATION)) {
    const double& inscribed_radius =
        costmap_ros_->getLayeredCostmap()->getInscribedRadius();

    // calculate the minimum distance to obstacles heuristically
    const double obs_dist =
        inscribed_radius -
        (std::log(robot_cost) -
         std::log(static_cast<double>(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))) /
            rpp_config_.inflation_cost_factor();

    if (obs_dist < rpp_config_.scaling_dist())
      return raw_linear_vel * rpp_config_.scaling_gain() * obs_dist /
             rpp_config_.scaling_dist();
  }
  return raw_linear_vel;
}

/**
 * @brief Applying approach constraints to regularize the speed of robot approaching final
 * goal
 * @param raw_linear_vel    the raw linear velocity of robot
 * @param robot_pose_global the robot's pose  [global]
 * @param prune_plan        the pruned plan
 * @return reg_vel          the regulated velocity
 */
double RPPController::_applyApproachConstraint(
    const double raw_linear_vel, geometry_msgs::PoseStamped robot_pose_global,
    const std::vector<geometry_msgs::PoseStamped>& prune_plan) {
  auto dist = [](const geometry_msgs::PoseStamped& ps_1,
                 const geometry_msgs::PoseStamped& ps_2) {
    return std::hypot(ps_1.pose.position.x - ps_2.pose.position.x,
                      ps_1.pose.position.y - ps_2.pose.position.y);
  };
  double remain_dist = 0.0;
  for (size_t i = 0; i < prune_plan.size() - 1; i++)
    remain_dist += dist(prune_plan[i], prune_plan[i + 1]);
  double s =
      remain_dist < rpp_config_.approach_dist() ?
          dist(prune_plan.back(), robot_pose_global) / rpp_config_.approach_dist() :
          1.0;

  return std::min(raw_linear_vel,
                  std::max(rpp_config_.approach_min_v(), raw_linear_vel * s));
}

}  // namespace controller
}  // namespace rmp