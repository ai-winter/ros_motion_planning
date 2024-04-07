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
#include <unsupported/Eigen/KroneckerProduct>
#include <unsupported/Eigen/MatrixFunctions>
#include <OsqpEigen/OsqpEigen.h>
#include <pluginlib/class_list_macros.h>

#include "mpc_planner.h"

PLUGINLIB_EXPORT_CLASS(mpc_planner::MPCPlanner, nav_core::BaseLocalPlanner)

namespace mpc_planner
{
/**
 * @brief Construct a new MPC planner object
 */
MPCPlanner::MPCPlanner() : initialized_(false), goal_reached_(false), tf_(nullptr)  //, costmap_ros_(nullptr)
{
}

/**
 * @brief Construct a new MPC planner object
 */
MPCPlanner::MPCPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) : MPCPlanner()
{
  initialize(name, tf, costmap_ros);
}

/**
 * @brief Destroy the MPC planner object
 */
MPCPlanner::~MPCPlanner()
{
}

/**
 * @brief Initialization of the local planner
 * @param name        the name to give this instance of the trajectory planner
 * @param tf          a pointer to a transform listener
 * @param costmap_ros the cost map to use for assigning costs to trajectories
 */
void MPCPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
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

    // iteration for ricatti solution
    nh.param("predicting_time_domain", p_, 4);
    nh.param("control_time_domain", m_, 4);

    // weight matrix for penalizing state error while tracking [x,y,theta]
    std::vector<double> diag_vec;
    nh.getParam("Q_matrix_diag", diag_vec);
    for (size_t i = 0; i < diag_vec.size(); ++i)
      Q_(i, i) = diag_vec[i];

    // weight matrix for penalizing input error while tracking[v, w]
    nh.getParam("R_matrix_diag", diag_vec);
    for (size_t i = 0; i < diag_vec.size(); ++i)
      R_(i, i) = diag_vec[i];

    double controller_freqency;
    nh.param("/move_base/controller_frequency", controller_freqency, 10.0);
    d_t_ = 1 / controller_freqency;

    target_pt_pub_ = nh.advertise<geometry_msgs::PointStamped>("/target_point", 10);
    current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);

    ROS_INFO("MPC planner initialized!");
  }
  else
  {
    ROS_WARN("MPC planner has already been initialized.");
  }
}

/**
 * @brief Set the plan that the controller is following
 * @param orig_global_plan the plan to pass to the controller
 * @return true if the plan was updated successfully, else false
 */
bool MPCPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
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
 * @brief Check if the goal pose has been achieved
 * @return true if achieved, false otherwise
 */
bool MPCPlanner::isGoalReached()
{
  if (!initialized_)
  {
    ROS_ERROR("MPC planner has not been initialized");
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
bool MPCPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_)
  {
    ROS_ERROR("MPC planner has not been initialized");
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
  std::vector<geometry_msgs::PoseStamped> prune_plan = prune(robot_pose_map);

  // calculate look-ahead distance
  double vt = std::hypot(base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y);
  double wt = base_odom.twist.twist.angular.z;
  double L = getLookAheadDistance(vt);

  // get the particular point on the path at the lookahead distance
  geometry_msgs::PointStamped lookahead_pt;
  double theta_trj, kappa;
  getLookAheadPoint(L, robot_pose_map, prune_plan, lookahead_pt, theta_trj, kappa);

  // current angle
  double theta = tf2::getYaw(robot_pose_map.pose.orientation);  // [-pi, pi]
  // calculate commands
  if (shouldRotateToGoal(robot_pose_map, global_plan_.back()))
  {
    du_p_ = Eigen::Vector2d(0, 0);
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
  else
  {
    Eigen::Vector3d s(robot_pose_map.pose.position.x, robot_pose_map.pose.position.y, theta);  // current state
    Eigen::Vector3d s_d(lookahead_pt.point.x, lookahead_pt.point.y, theta_trj);                // desired state
    Eigen::Vector2d u_r(vt, regularizeAngle(vt * kappa));                                      // refered input
    Eigen::Vector2d u = _mpcControl(s, s_d, u_r, du_p_);
    double u_v = linearRegularization(base_odom, u[0]);
    double u_w = angularRegularization(base_odom, u[1]);
    du_p_ = Eigen::Vector2d(u_v - u_r[0], regularizeAngle(u_w - u_r[1]));
    cmd_vel.linear.x = u_v;
    cmd_vel.angular.z = u_w;
  }

  // publish lookahead pose
  target_pt_pub_.publish(lookahead_pt);

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
Eigen::Vector2d MPCPlanner::_mpcControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector2d u_r,
                                        Eigen::Vector2d du_p)
{
  int dim_u = 2;
  int dim_x = 3;

  // state vector (5 x 1)
  Eigen::VectorXd x = Eigen::VectorXd(dim_x + dim_u);
  x.topLeftCorner(dim_x, 1) = s - s_d;
  x[2] = regularizeAngle(x[2]);
  x.bottomLeftCorner(dim_u, 1) = du_p;

  // original state matrix
  Eigen::Matrix3d A_o = Eigen::Matrix3d::Identity();
  A_o(0, 2) = -u_r[0] * sin(s_d[2]) * d_t_;
  A_o(1, 2) = u_r[0] * cos(s_d[2]) * d_t_;

  // original control matrix
  Eigen::MatrixXd B_o = Eigen::MatrixXd::Zero(dim_x, dim_u);
  B_o(0, 0) = cos(s_d[2]) * d_t_;
  B_o(1, 0) = sin(s_d[2]) * d_t_;
  B_o(2, 1) = d_t_;

  // state matrix (5 x 5)
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dim_x + dim_u, dim_x + dim_u);
  A.topLeftCorner(dim_x, dim_x) = A_o;
  A.topRightCorner(dim_x, dim_u) = B_o;
  A.bottomLeftCorner(dim_u, dim_x) = Eigen::MatrixXd::Zero(dim_u, dim_x);
  A.bottomRightCorner(dim_u, dim_u) = Eigen::Matrix2d::Identity();

  // control matrix (5 x 2)
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(dim_x + dim_u, dim_u);
  B.topLeftCorner(dim_x, dim_u) = B_o;
  B.bottomLeftCorner(dim_u, dim_u) = Eigen::Matrix2d::Identity();

  // output matrix(3 x 5)
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_x, dim_x + dim_u);
  C.topLeftCorner(dim_x, dim_x) = Eigen::Matrix3d::Identity();
  C.topRightCorner(dim_x, dim_u) = Eigen::MatrixXd::Zero(dim_x, dim_u);

  // mpc state matrix(3p x 5)
  Eigen::MatrixPower<Eigen::MatrixXd> A_pow(A);
  Eigen::MatrixXd S_x = Eigen::MatrixXd::Zero(dim_x * p_, dim_x + dim_u);
  for (int i = 0; i < p_; i++)
    S_x.middleRows(dim_x * i, dim_x) = C * A_pow(i + 1);

  // mpc control matrix(3p x 2m)
  Eigen::MatrixXd S_u = Eigen::MatrixXd::Zero(dim_x * p_, dim_u * m_);
  for (int i = 0; i < p_; i++)
  {
    for (int j = 0; j < m_; j++)
    {
      if (j <= i)
        S_u.block(dim_x * i, dim_u * j, dim_x, dim_u) = C * A_pow(i - j) * B;
      else
        S_u.block(dim_x * i, dim_u * j, dim_x, dim_u) = Eigen::MatrixXd::Zero(dim_x, dim_u);
    }
  }

  // optimization
  // min 1/2 * x.T * H * x + g.T * x
  // s.t. l <= Px <= u
  Eigen::VectorXd Yr = Eigen::VectorXd::Zero(dim_x * p_);                              // (3p x 1)
  Eigen::MatrixXd Q = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(p_, p_), Q_);  // (3p x 3p)
  Eigen::MatrixXd R = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(m_, m_), R_);  // (2m x 2m)
  Eigen::MatrixXd H = S_u.transpose() * Q * S_u + R;                                   // (2m x 2m)
  Eigen::VectorXd g = S_u.transpose() * Q * (S_x * x - Yr);                            // (2m x 1)

  // boundary
  Eigen::Vector2d u_min(min_v_, -max_w_);
  Eigen::Vector2d u_max(max_v_, max_w_);
  Eigen::Vector2d u_k_1(du_p[0], du_p[1]);
  Eigen::Vector2d du_min(-0.2, -0.2);
  Eigen::Vector2d du_max(0.2, M_PI);
  Eigen::VectorXd U_min = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), u_min);    // (2m x 1)
  Eigen::VectorXd U_max = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), u_max);    // (2m x 1)
  Eigen::VectorXd U_r = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), u_r);        // (2m x 1)
  Eigen::VectorXd U_k_1 = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), u_k_1);    // (2m x 1)
  Eigen::VectorXd dU_min = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), du_min);  // (2m x 1)
  Eigen::VectorXd dU_max = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), du_max);  // (2m x 1)

  // constriants
  Eigen::MatrixXd temp = Eigen::MatrixXd::Ones(m_, m_).triangularView<Eigen::Lower>();
  Eigen::MatrixXd A_I = Eigen::kroneckerProduct(temp, Eigen::MatrixXd::Identity(dim_u, dim_u));  // (2m x 2m)

  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(2 * dim_u * m_, dim_u * m_);  // (4m x 2m)
  P.topRows(dim_u * m_) = A_I;
  P.bottomRows(dim_u * m_) = Eigen::MatrixXd::Identity(dim_u * m_, dim_u * m_);

  Eigen::VectorXd lower = Eigen::VectorXd::Zero(2 * dim_u * m_);  // (4m x 1)
  Eigen::VectorXd upper = Eigen::VectorXd::Zero(2 * dim_u * m_);  // (4m x 1)
  lower.topRows(dim_u * m_) = U_min - U_k_1 - U_r;
  lower.bottomRows(dim_u * m_) = dU_min;
  upper.topRows(dim_u * m_) = U_max - U_k_1 - U_r;
  upper.bottomRows(dim_u * m_) = dU_max;

  // solve
  OsqpEigen::Solver solver;
  solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(true);
  solver.data()->setNumberOfVariables(dim_u * m_);
  solver.data()->setNumberOfConstraints(2 * dim_u * m_);
  solver.data()->setHessianMatrix(_convertToSparseMatrix(H));
  solver.data()->setGradient(g);
  solver.data()->setLinearConstraintsMatrix(_convertToSparseMatrix(P));
  solver.data()->setLowerBound(lower);
  solver.data()->setUpperBound(upper);

  solver.initSolver();
  solver.solveProblem();

  Eigen::VectorXd solution = solver.getSolution();

  // real control
  Eigen::Vector2d u(solution[0] + du_p[0] + u_r[0], regularizeAngle(solution[1] + du_p[1] + u_r[1]));

  return u;
}

/**
 * @brief convert matrix A to its sparse view
 * @param A     dense matrix
 * @return A_s  sparse matrix
 */
Eigen::SparseMatrix<double> MPCPlanner::_convertToSparseMatrix(Eigen::MatrixXd A)
{
  int row = A.rows();
  int col = A.cols();
  Eigen::SparseMatrix<double> A_s(row, col);

  for (int i = 0; i < row; i++)
    for (int j = 0; j < col; j++)
      A_s.insert(i, j) = A(i, j);

  return A_s;
}

}  // namespace mpc_planner