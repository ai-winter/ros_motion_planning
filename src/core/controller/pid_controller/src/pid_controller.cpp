/**
 * *********************************************************
 *
 * @file: pid_controller.cpp
 * @brief: Contains the Proportional–Integral–Derivative (PID) controller local controller
 * class
 * @author: Yang Haodong, Guo Zhanyu
 * @date: 2024-01-20
 * @version: 1.2
 *
 * Copyright (c) 2024, Yang Haodong, Guo Zhanyu.
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
#include "common/geometry/point.h"
#include "controller/pid_controller.h"

PLUGINLIB_EXPORT_CLASS(rmp::controller::PIDController, nav_core::BaseLocalPlanner)

using namespace rmp::common::math;
using namespace rmp::common::geometry;

namespace rmp {
namespace controller {
/**
 * @brief Construct a new PIDController object
 */
PIDController::PIDController() : initialized_(false), goal_reached_(false), tf_(nullptr) {
}

/**
 * @brief Construct a new PIDController object
 */
PIDController::PIDController(std::string name, tf2_ros::Buffer* tf,
                             costmap_2d::Costmap2DROS* costmap_ros)
  : PIDController() {
  initialize(name, tf, costmap_ros);
}

/**
 * @brief Destroy the PIDController object
 */
PIDController::~PIDController() {
}

/**
 * @brief Initialization of the local planner
 * @param name        the name to give this instance of the trajectory planner
 * @param tf          a pointer to a transform listener
 * @param costmap_ros the cost map to use for assigning costs to trajectories
 */
void PIDController::initialize(std::string name, tf2_ros::Buffer* tf,
                               costmap_2d::Costmap2DROS* costmap_ros) {
  if (!initialized_) {
    initialized_ = true;
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    ros::NodeHandle nh = ros::NodeHandle("~/" + name);
    pid_config_ = config_.pid_controller();

    e_v_ = i_v_ = 0.0;
    e_w_ = i_w_ = 0.0;

    target_pt_pub_ = nh.advertise<visualization_msgs::Marker>("/target_point", 1);
    current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);

    std::string model_type =
        pid_config_.model_based_mode() ? "Model-based" : "Model-free";
    R_INFO << model_type + " PID Controller initialized!";
  } else {
    R_WARN << "PID Controller has already been initialized.";
  }
}

/**
 * @brief Set the plan that the controller is following
 * @param orig_global_plan the plan to pass to the controller
 * @return true if the plan was updated successfully, else false
 */
bool PIDController::setPlan(
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

  // receive a plan for a new goal
  if (goal_x_ != global_plan_.back().pose.position.x ||
      goal_y_ != global_plan_.back().pose.position.y) {
    goal_x_ = global_plan_.back().pose.position.x;
    goal_y_ = global_plan_.back().pose.position.y;
    goal_theta_ = getYawAngle(global_plan_.back());
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
bool PIDController::isGoalReached() {
  if (!initialized_) {
    R_ERROR << "PID Controller has not been initialized";
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
 * @return true if a valid trajectory was found, else false
 */
bool PIDController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  if (!initialized_) {
    R_ERROR << "PID Controller has not been initialized";
    return false;
  }

  // odometry observation - getting robot velocities in odom
  nav_msgs::Odometry base_odom;
  odom_helper_->getOdom(base_odom);

  // get robot position in map
  geometry_msgs::PoseStamped robot_pose_odom, robot_pose_map;
  costmap_ros_->getRobotPose(robot_pose_odom);
  transformPose(tf_, config_.map_frame(), robot_pose_odom, robot_pose_map);

  // prune the global plan
  std::vector<geometry_msgs::PoseStamped> prune_plan = prune(robot_pose_odom);

  // calculate look-ahead distance
  double vt = std::hypot(base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y);
  double wt = base_odom.twist.twist.angular.z;
  double L = clamp(std::fabs(vt) * pid_config_.lookahead_time(),
                   pid_config_.min_lookahead_dist(), pid_config_.max_lookahead_dist());

  // get the particular point on the path at the lookahead distance
  double kappa;
  Point3d lookahead_pt;
  getLookAheadPoint(L, robot_pose_map, prune_plan, &lookahead_pt, &kappa);
  // Vec2d curr_pt(robot_pose_odom.pose.position.x, robot_pose_odom.pose.position.y);
  // const double theta_dir =
  //     std::atan2((lookahead_pt.y() - curr_pt.y()), (lookahead_pt.x() - curr_pt.x()));
  // theta_d = normalizeAngle((1 - pid_config_.theta_weight()) * lookahead_pt.theta() +
  //                          pid_config_.theta_weight() * theta_dir);
  // tf2::Quaternion q;
  // q.setRPY(0, 0, theta_d);
  // tf2::convert(q, target_ps_map.pose.orientation);

  // current angle
  double theta = tf2::getYaw(robot_pose_map.pose.orientation);  // [-pi, pi]

  // position reached
  if (shouldRotateToGoal(robot_pose_map, global_plan_.back())) {
    double e_theta = normalizeAngle(goal_theta_ - theta);

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
  // posistion not reached
  else {
    Eigen::Vector3d s(robot_pose_map.pose.position.x, robot_pose_map.pose.position.y,
                      theta);  // current state
    Eigen::Vector3d s_d(lookahead_pt.x(), lookahead_pt.y(),
                        lookahead_pt.theta());  // desired state
    Eigen::Vector2d u_r(vt, wt);                // refered input
    Eigen::Vector2d u = pid_config_.model_based_mode() ?
                            _modelBasedPIDControl(s, s_d, u_r) :
                            _modelFreePIDControl(s, s_d, u_r);

    cmd_vel.linear.x = linearRegularization(vt, u[0]);
    cmd_vel.angular.z = angularRegularization(wt, u[1]);
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
 * @brief Execute model-free PID control process
 * @param s   current state
 * @param s_d desired state
 * @param u_r refered input
 * @return u  control vector
 */
Eigen::Vector2d PIDController::_modelFreePIDControl(Eigen::Vector3d s,
                                                    Eigen::Vector3d s_d,
                                                    Eigen::Vector2d u_r) {
  Eigen::Vector2d u;
  Eigen::Vector3d e = s_d - s;

  double e_x = e[0];
  double e_y = e[1];
  double e_theta = e[2];

  double v_d = std::hypot(e_x, e_y) / control_dt_;
  double w_d = e_theta / control_dt_;

  if (std::fabs(v_d) > config_.max_linear_velocity()) {
    v_d = std::copysign(config_.max_linear_velocity(), v_d);
  }
  if (std::fabs(w_d) > config_.max_angular_velocity()) {
    w_d = std::copysign(config_.max_angular_velocity(), w_d);
  }

  double e_v = v_d - u_r[0];
  double e_w = w_d - u_r[1];

  i_v_ += e_v * control_dt_;
  i_w_ += e_w * control_dt_;

  double d_v = (e_v - e_v_) / control_dt_;
  double d_w = (e_w - e_w_) / control_dt_;

  e_v_ = e_v;
  e_w_ = e_w;

  double v_inc = pid_config_.p_linear_velocity() * e_v +
                 pid_config_.i_linear_velocity() * i_v_ +
                 pid_config_.d_linear_velocity() * d_v;
  double w_inc = pid_config_.p_angular_velocity() * e_w +
                 pid_config_.i_angular_velocity() * i_w_ +
                 pid_config_.d_angular_velocity() * d_w;

  if (std::fabs(v_inc) > config_.max_linear_velocity_increment()) {
    v_inc = std::copysign(config_.max_linear_velocity_increment(), v_inc);
  }
  if (std::fabs(w_inc) > config_.max_angular_velocity_increment()) {
    w_inc = std::copysign(config_.max_angular_velocity_increment(), w_inc);
  }

  double v_cmd = u_r[0] + v_inc;
  if (std::fabs(v_cmd) > config_.max_linear_velocity()) {
    v_cmd = std::copysign(config_.max_linear_velocity(), v_cmd);
  } else if (std::fabs(v_cmd) < config_.min_linear_velocity()) {
    v_cmd = std::copysign(config_.min_linear_velocity(), v_cmd);
  }

  double w_cmd = u_r[1] + w_inc;
  if (std::fabs(w_cmd) > config_.max_angular_velocity()) {
    w_cmd = std::copysign(config_.max_angular_velocity(), w_cmd);
  } else if (std::fabs(w_cmd) < config_.min_angular_velocity()) {
    w_cmd = std::copysign(config_.min_angular_velocity(), w_cmd);
  }

  u[0] = v_cmd;
  u[1] = w_cmd;

  return u;
}

/**
 * @brief Execute model-based PID control process
 * @param s   current state
 * @param s_d desired state
 * @param u_r refered input
 * @return u  control vector
 */
Eigen::Vector2d PIDController::_modelBasedPIDControl(Eigen::Vector3d s,
                                                     Eigen::Vector3d s_d,
                                                     Eigen::Vector2d u_r) {
  Eigen::Vector2d u;
  Eigen::Vector3d e(s_d - s);
  Eigen::Vector2d sx_dot(pid_config_.k_feedback() * e[0],
                         pid_config_.k_feedback() * e[1]);
  Eigen::Matrix2d R_inv;
  R_inv << std::cos(s[2]), sin(s[2]),
      -std::sin(s[2]) / pid_config_.dist_from_center_to_front_edge(),
      std::cos(s[2]) / pid_config_.dist_from_center_to_front_edge();
  u = R_inv * sx_dot;

  return u;
}
}  // namespace controller
}  // namespace rmp