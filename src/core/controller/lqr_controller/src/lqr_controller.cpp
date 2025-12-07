/**
 * *********************************************************
 *
 * @file: lqr_controller.cpp
 * @brief: Contains the linear quadratic regulator (LQR) local controller class
 * @author: Yang Haodong
 * @date: 2024-01-12
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "common/util/log.h"
#include "common/util/visualizer.h"
#include "common/math/math_helper.h"
#include "common/geometry/angles.h"
#include "controller/lqr_controller.h"

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rmp::controller::LQRController, nav_core::BaseLocalPlanner)

using namespace rmp::common::math;
using namespace rmp::common::geometry;

namespace rmp {
namespace controller {
/**
 * @brief Construct a new LQR controller object
 */
LQRController::LQRController() : initialized_(false), goal_reached_(false), tf_(nullptr) {
}

/**
 * @brief Construct a new LQR controller object
 */
LQRController::LQRController(std::string name, tf2_ros::Buffer* tf,
                             costmap_2d::Costmap2DROS* costmap_ros)
  : LQRController() {
  initialize(name, tf, costmap_ros);
}

/**
 * @brief Destroy the LQR controller object
 */
LQRController::~LQRController() {
}

/**
 * @brief Initialization of the local planner
 * @param name        the name to give this instance of the trajectory planner
 * @param tf          a pointer to a transform listener
 * @param costmap_ros the cost map to use for assigning costs to trajectories
 */
void LQRController::initialize(std::string name, tf2_ros::Buffer* tf,
                               costmap_2d::Costmap2DROS* costmap_ros) {
  if (!initialized_) {
    initialized_ = true;
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    ros::NodeHandle nh = ros::NodeHandle("~/" + name);
    lqr_config_ = config_.lqr_controller();

    // weight matrix for penalizing state error while tracking [x,y,theta]
    Q_ = Eigen::Matrix3d::Zero();
    Q_(0, 0) = lqr_config_.q_matrix_x();
    Q_(1, 1) = lqr_config_.q_matrix_y();
    Q_(2, 2) = lqr_config_.q_matrix_theta();

    // weight matrix for penalizing input error while tracking[v, w]
    R_ = Eigen::Matrix2d::Zero();
    R_(0, 0) = lqr_config_.r_matrix_v();
    R_(1, 1) = lqr_config_.r_matrix_w();

    target_pt_pub_ = nh.advertise<visualization_msgs::Marker>("/target_point", 1);
    current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);

    R_INFO << "LQR controller initialized!";
  } else {
    R_WARN << "LQR controller has already been initialized.";
  }
}

/**
 * @brief Set the plan that the controller is following
 * @param orig_global_plan the plan to pass to the controller
 * @return true if the plan was updated successfully, else false
 */
bool LQRController::setPlan(
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
 * @brief Check if the goal pose has been achieved
 * @return true if achieved, false otherwise
 */
bool LQRController::isGoalReached() {
  if (!initialized_) {
    R_ERROR << "LQR controller has not been initialized";
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
bool LQRController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  if (!initialized_) {
    R_ERROR << "LQR controller has not been initialized";
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
  double L = clamp(std::fabs(vt) * lqr_config_.lookahead_time(),
                   lqr_config_.min_lookahead_dist(), lqr_config_.max_lookahead_dist());

  // get the particular point on the path at the lookahead distance
  double kappa;
  Point3d lookahead_pt;
  getLookAheadPoint(L, robot_pose_map, prune_plan, &lookahead_pt, &kappa);

  // current angle
  double theta = tf2::getYaw(robot_pose_map.pose.orientation);  // [-pi, pi]

  // calculate commands
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
  } else {
    Eigen::Vector3d s(robot_pose_map.pose.position.x, robot_pose_map.pose.position.y,
                      theta);  // current state
    Eigen::Vector3d s_d(lookahead_pt.x(), lookahead_pt.y(),
                        lookahead_pt.theta());  // desired state
    Eigen::Vector2d u_r(vt, vt * kappa);        // refered input
    Eigen::Vector2d u = _lqrControl(s, s_d, u_r);

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
 * @brief Execute LQR control process
 * @param s   current state
 * @param s_d desired state
 * @param u_r refered control
 * @return u  control vector
 */
Eigen::Vector2d LQRController::_lqrControl(Eigen::Vector3d s, Eigen::Vector3d s_d,
                                           Eigen::Vector2d u_r) {
  Eigen::Vector2d u;
  Eigen::Vector3d e(s - s_d);
  e[2] = normalizeAngle(e[2]);

  // state equation on error
  Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
  A(0, 2) = -u_r[0] * sin(s_d[2]) * control_dt_;
  A(1, 2) = u_r[0] * cos(s_d[2]) * control_dt_;

  Eigen::Matrix<double, 3, 2> B = Eigen::Matrix<double, 3, 2>::Zero();
  B(0, 0) = cos(s_d[2]) * control_dt_;
  B(1, 0) = sin(s_d[2]) * control_dt_;
  B(2, 1) = control_dt_;
  Eigen::Matrix<double, 2, 3> B_trans = B.transpose();

  // discrete iteration Ricatti equation
  Eigen::Matrix3d P, P_;
  P = Q_;
  for (int i = 0; i < lqr_config_.solver_max_iterations(); ++i) {
    Eigen::Matrix2d temp = R_ + B_trans * P * B;
    P_ = Q_ + A.transpose() * P * A -
         A.transpose() * P * B * temp.inverse() * B_trans * P * A;
    if ((P - P_).array().abs().maxCoeff() < lqr_config_.solver_eps()) {
      break;
    }
    P = P_;
  }

  // feedback
  Eigen::Matrix<double, 2, 3> K = -(R_ + B_trans * P_ * B).inverse() * B_trans * P_ * A;
  u = u_r + K * e;

  return u;
}

}  // namespace controller
}  // namespace rmp
