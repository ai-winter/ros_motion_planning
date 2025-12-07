/**
 * *********************************************************
 *
 * @file: mpc_planner.cpp
 * @brief: Contains the model predicted control (MPC) local planner class
 * @author: Yang Haodong
 * @date: 2024-01-31
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <osqp/osqp.h>
#include <unsupported/Eigen/KroneckerProduct>
#include <unsupported/Eigen/MatrixFunctions>
#include <pluginlib/class_list_macros.h>

#include "common/util/log.h"
#include "common/util/visualizer.h"
#include "common/math/math_helper.h"
#include "common/geometry/angles.h"
#include "controller/mpc_controller.h"

PLUGINLIB_EXPORT_CLASS(rmp::controller::MPCController, nav_core::BaseLocalPlanner)

using namespace rmp::common::math;
using namespace rmp::common::geometry;

namespace rmp {
namespace controller {
static constexpr int X_DIM = 3;
static constexpr int U_DIM = 2;

/**
 * @brief Construct a new MPC Controller object
 */
MPCController::MPCController()
  : initialized_(false)
  , goal_reached_(false)
  , tf_(nullptr)  //, costmap_ros_(nullptr)
{
}

/**
 * @brief Construct a new MPC Controller object
 */
MPCController::MPCController(std::string name, tf2_ros::Buffer* tf,
                             costmap_2d::Costmap2DROS* costmap_ros)
  : MPCController() {
  initialize(name, tf, costmap_ros);
}

/**
 * @brief Destroy the MPC Controller object
 */
MPCController::~MPCController() {
}

/**
 * @brief Initialization of the local planner
 * @param name        the name to give this instance of the trajectory planner
 * @param tf          a pointer to a transform listener
 * @param costmap_ros the cost map to use for assigning costs to trajectories
 */
void MPCController::initialize(std::string name, tf2_ros::Buffer* tf,
                               costmap_2d::Costmap2DROS* costmap_ros) {
  if (!initialized_) {
    initialized_ = true;
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    ros::NodeHandle nh = ros::NodeHandle("~/" + name);
    mpc_config_ = config_.mpc_controller();

    // weight matrix for penalizing state error while tracking [x,y,theta]
    Q_ = Eigen::Matrix3d::Zero();
    Q_(0, 0) = mpc_config_.q_matrix_x();
    Q_(1, 1) = mpc_config_.q_matrix_y();
    Q_(2, 2) = mpc_config_.q_matrix_theta();

    // weight matrix for penalizing input error while tracking[v, w]
    R_ = Eigen::Matrix2d::Zero();
    R_(0, 0) = mpc_config_.r_matrix_v();
    R_(1, 1) = mpc_config_.r_matrix_w();

    target_pt_pub_ = nh.advertise<visualization_msgs::Marker>("/target_point", 10);
    current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);

    R_INFO << "MPC Controller initialized!";
  } else {
    R_WARN << "MPC Controller has already been initialized.";
  }
}

/**
 * @brief Set the plan that the controller is following
 * @param orig_global_plan the plan to pass to the controller
 * @return true if the plan was updated successfully, else false
 */
bool MPCController::setPlan(
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
bool MPCController::isGoalReached() {
  if (!initialized_) {
    R_ERROR << "MPC Controller has not been initialized";
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
bool MPCController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  if (!initialized_) {
    R_ERROR << "MPC Controller has not been initialized";
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
  double L = clamp(std::fabs(vt) * mpc_config_.lookahead_time(),
                   mpc_config_.min_lookahead_dist(), mpc_config_.max_lookahead_dist());

  // get the particular point on the path at the lookahead distance
  double kappa;
  Point3d lookahead_pt;
  getLookAheadPoint(L, robot_pose_map, prune_plan, &lookahead_pt, &kappa);

  // current angle
  double theta = tf2::getYaw(robot_pose_map.pose.orientation);  // [-pi, pi]
  // calculate commands
  if (shouldRotateToGoal(robot_pose_map, global_plan_.back())) {
    du_p_ = Eigen::Vector2d(0, 0);
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
                        lookahead_pt.theta());            // desired state
    Eigen::Vector2d u_r(vt, normalizeAngle(vt * kappa));  // refered input
    Eigen::Vector2d u = _mpcControl(s, s_d, u_r, du_p_);
    double u_v = linearRegularization(vt, u[0]);
    double u_w = angularRegularization(wt, u[1]);
    du_p_ = Eigen::Vector2d(u_v - u_r[0], normalizeAngle(u_w - u_r[1]));
    cmd_vel.linear.x = u_v;
    cmd_vel.angular.z = u_w;
  }

  // publish lookahead pose
  const auto& visualizer = rmp::common::util::VisualizerPtr::Instance();
  Points3d points;
  points.emplace_back(std::move(lookahead_pt));
  visualizer->publishPoints(points, target_pt_pub_, "map", "lookahead",
                            rmp::common::util::Visualizer::RED, 0.3);

  // publish robot pose
  current_pose_pub_.publish(robot_pose_map);

  return true;
}

/**
 * @brief Execute MPC control process
 * @param s     current state
 * @param s_d   desired state
 * @param u_r   refered control
 * @param du_p  previous control error
 * @return u  control vector
 */
Eigen::Vector2d MPCController::_mpcControl(Eigen::Vector3d s, Eigen::Vector3d s_d,
                                           Eigen::Vector2d u_r, Eigen::Vector2d du_p) {
  // state vector (5 x 1)
  Eigen::VectorXd x = Eigen::VectorXd(X_DIM + U_DIM);
  x.topLeftCorner(X_DIM, 1) = s - s_d;
  x[2] = normalizeAngle(x[2]);
  x.bottomLeftCorner(U_DIM, 1) = du_p;

  // original state matrix
  Eigen::Matrix3d A_o = Eigen::Matrix3d::Identity();
  A_o(0, 2) = -u_r[0] * sin(s_d[2]) * control_dt_;
  A_o(1, 2) = u_r[0] * cos(s_d[2]) * control_dt_;

  // original control matrix
  Eigen::MatrixXd B_o = Eigen::MatrixXd::Zero(X_DIM, U_DIM);
  B_o(0, 0) = cos(s_d[2]) * control_dt_;
  B_o(1, 0) = sin(s_d[2]) * control_dt_;
  B_o(2, 1) = control_dt_;

  // state matrix (5 x 5)
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(X_DIM + U_DIM, X_DIM + U_DIM);
  A.topLeftCorner(X_DIM, X_DIM) = A_o;
  A.topRightCorner(X_DIM, U_DIM) = B_o;
  A.bottomLeftCorner(U_DIM, X_DIM) = Eigen::MatrixXd::Zero(U_DIM, X_DIM);
  A.bottomRightCorner(U_DIM, U_DIM) = Eigen::Matrix2d::Identity();

  // control matrix (5 x 2)
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(X_DIM + U_DIM, U_DIM);
  B.topLeftCorner(X_DIM, U_DIM) = B_o;
  B.bottomLeftCorner(U_DIM, U_DIM) = Eigen::Matrix2d::Identity();

  // output matrix(3 x 5)
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(X_DIM, X_DIM + U_DIM);
  C.topLeftCorner(X_DIM, X_DIM) = Eigen::Matrix3d::Identity();
  C.topRightCorner(X_DIM, U_DIM) = Eigen::MatrixXd::Zero(X_DIM, U_DIM);

  // mpc state matrix(3p x 5)
  Eigen::MatrixPower<Eigen::MatrixXd> A_pow(A);
  Eigen::MatrixXd S_x =
      Eigen::MatrixXd::Zero(X_DIM * mpc_config_.predict_time_domain(), X_DIM + U_DIM);
  for (int i = 0; i < mpc_config_.predict_time_domain(); i++) {
    S_x.middleRows(X_DIM * i, X_DIM) = C * A_pow(i + 1);
  }

  // mpc control matrix(3p x 2m)
  Eigen::MatrixXd S_u = Eigen::MatrixXd::Zero(X_DIM * mpc_config_.predict_time_domain(),
                                              U_DIM * mpc_config_.control_time_domain());
  for (int i = 0; i < mpc_config_.predict_time_domain(); i++) {
    for (int j = 0; j < mpc_config_.control_time_domain(); j++) {
      if (j <= i) {
        S_u.block(X_DIM * i, U_DIM * j, X_DIM, U_DIM) = C * A_pow(i - j) * B;
      } else {
        S_u.block(X_DIM * i, U_DIM * j, X_DIM, U_DIM) =
            Eigen::MatrixXd::Zero(X_DIM, U_DIM);
      }
    }
  }

  // optimization
  // min 1/2 * x.T * P * x + q.T * x
  // s.t. l <= Ax <= u
  Eigen::VectorXd Yr =
      Eigen::VectorXd::Zero(X_DIM * mpc_config_.predict_time_domain());  // (3p x 1)
  Eigen::MatrixXd Q = Eigen::kroneckerProduct(
      Eigen::MatrixXd::Identity(mpc_config_.predict_time_domain(),
                                mpc_config_.predict_time_domain()),
      Q_);  // (3p x 3p)
  Eigen::MatrixXd R = Eigen::kroneckerProduct(
      Eigen::MatrixXd::Identity(mpc_config_.control_time_domain(),
                                mpc_config_.control_time_domain()),
      R_);                                                   // (2m x 2m)
  Eigen::MatrixXd P = S_u.transpose() * Q * S_u + R;         // (2m x 2m)
  Eigen::VectorXd q = S_u.transpose() * Q * (S_x * x - Yr);  // (2m x 1)

  // boundary
  Eigen::Vector2d u_min(config_.min_linear_velocity(), -config_.max_angular_velocity());
  Eigen::Vector2d u_max(config_.max_linear_velocity(), config_.max_angular_velocity());
  Eigen::Vector2d u_k_1(du_p[0], du_p[1]);
  Eigen::Vector2d du_min(mpc_config_.delta_linear_velocity_min(),
                         mpc_config_.delta_angular_velocity_min());
  Eigen::Vector2d du_max(mpc_config_.delta_linear_velocity_max(),
                         mpc_config_.delta_angular_velocity_max());
  Eigen::VectorXd U_min = Eigen::kroneckerProduct(
      Eigen::VectorXd::Ones(mpc_config_.control_time_domain()), u_min);  // (2m x 1)
  Eigen::VectorXd U_max = Eigen::kroneckerProduct(
      Eigen::VectorXd::Ones(mpc_config_.control_time_domain()), u_max);  // (2m x 1)
  Eigen::VectorXd U_r = Eigen::kroneckerProduct(
      Eigen::VectorXd::Ones(mpc_config_.control_time_domain()), u_r);  // (2m x 1)
  Eigen::VectorXd U_k_1 = Eigen::kroneckerProduct(
      Eigen::VectorXd::Ones(mpc_config_.control_time_domain()), u_k_1);  // (2m x 1)
  Eigen::VectorXd dU_min = Eigen::kroneckerProduct(
      Eigen::VectorXd::Ones(mpc_config_.control_time_domain()), du_min);  // (2m x 1)
  Eigen::VectorXd dU_max = Eigen::kroneckerProduct(
      Eigen::VectorXd::Ones(mpc_config_.control_time_domain()), du_max);  // (2m x 1)

  // constriants
  Eigen::VectorXd lower =
      Eigen::VectorXd::Zero(2 * U_DIM * mpc_config_.control_time_domain());  // (4m x 1)
  Eigen::VectorXd upper =
      Eigen::VectorXd::Zero(2 * U_DIM * mpc_config_.control_time_domain());  // (4m x 1)
  lower.topRows(U_DIM * mpc_config_.control_time_domain()) = U_min - U_k_1 - U_r;
  lower.bottomRows(U_DIM * mpc_config_.control_time_domain()) = dU_min;
  upper.topRows(U_DIM * mpc_config_.control_time_domain()) = U_max - U_k_1 - U_r;
  upper.bottomRows(U_DIM * mpc_config_.control_time_domain()) = dU_max;

  // Calculate kernel
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  int ind_P = 0;
  for (int col = 0; col < U_DIM * mpc_config_.control_time_domain(); ++col) {
    P_indptr.push_back(ind_P);
    for (int row = 0; row <= col; ++row) {
      P_data.push_back(P(row, col));
      // P_data.push_back(P(row, col) * 2.0);
      P_indices.push_back(row);
      ind_P++;
    }
  }
  P_indptr.push_back(ind_P);

  // Calculate affine constraints (4m x 2m)
  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  int ind_A = 0;
  A_indptr.push_back(ind_A);
  for (int j = 0; j < mpc_config_.control_time_domain(); ++j) {
    for (int n = 0; n < U_DIM; ++n) {
      for (int row = U_DIM * j + n; row < U_DIM * mpc_config_.control_time_domain();
           row += U_DIM) {
        A_data.push_back(1.0);
        A_indices.push_back(row);
        ++ind_A;
      }
      A_data.push_back(1.0);
      A_indices.push_back(U_DIM * mpc_config_.control_time_domain() + U_DIM * j + n);
      ++ind_A;
      A_indptr.push_back(ind_A);
    }
  }

  // Calculate offset
  std::vector<c_float> q_data;
  for (int row = 0; row < U_DIM * mpc_config_.control_time_domain(); ++row) {
    q_data.push_back(q(row, 0));
  }

  // Calculate constraints
  std::vector<c_float> lower_bounds;
  std::vector<c_float> upper_bounds;
  for (int row = 0; row < 2 * U_DIM * mpc_config_.control_time_domain(); row++) {
    lower_bounds.push_back(lower(row, 0));
    upper_bounds.push_back(upper(row, 0));
  }

  // solve
  OSQPWorkspace* work = nullptr;
  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  osqp_set_default_settings(settings);
  settings->verbose = false;
  settings->warm_start = true;

  data->n = U_DIM * mpc_config_.control_time_domain();
  data->m = 2 * U_DIM * mpc_config_.control_time_domain();
  data->P = csc_matrix(data->n, data->n, P_data.size(), P_data.data(), P_indices.data(),
                       P_indptr.data());
  data->q = q.data();
  data->A = csc_matrix(data->m, data->n, A_data.size(), A_data.data(), A_indices.data(),
                       A_indptr.data());
  data->l = lower_bounds.data();
  data->u = upper_bounds.data();

  osqp_setup(&work, data, settings);
  osqp_solve(work);
  auto status = work->info->status_val;

  if ((status < 0) || (status != 1 && status != 2)) {
    R_WARN << "failed optimization status: " << work->info->status;
    return Eigen::Vector2d::Zero();
  }

  Eigen::Vector2d u(work->solution->x[0] + du_p[0] + u_r[0],
                    normalizeAngle(work->solution->x[1] + du_p[1] + u_r[1]));

  // Cleanup
  osqp_cleanup(work);
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);

  return u;
}

}  // namespace controller
}  // namespace rmp