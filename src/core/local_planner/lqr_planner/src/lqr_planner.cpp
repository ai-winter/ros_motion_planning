/**
 * *********************************************************
 *
 * @file: lqr_planner.cpp
 * @brief: Contains the linear quadratic regulator (LQR) local planner class
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
#include <pluginlib/class_list_macros.h>

#include "lqr_planner.h"

PLUGINLIB_EXPORT_CLASS(lqr_planner::LQRPlanner, nav_core::BaseLocalPlanner)

namespace lqr_planner
{
/**
 * @brief Construct a new LQR planner object
 */
LQRPlanner::LQRPlanner() : initialized_(false), goal_reached_(false), tf_(nullptr)  //, costmap_ros_(nullptr)
{
}

/**
 * @brief Construct a new LQR planner object
 */
LQRPlanner::LQRPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) : LQRPlanner()
{
  initialize(name, tf, costmap_ros);
}

/**
 * @brief Destroy the LQR planner object
 */
LQRPlanner::~LQRPlanner()
{
}

/**
 * @brief Initialization of the local planner
 * @param name        the name to give this instance of the trajectory planner
 * @param tf          a pointer to a transform listener
 * @param costmap_ros the cost map to use for assigning costs to trajectories
 */
void LQRPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
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
    nh.param("max_iter_", max_iter_, 100);
    nh.param("eps_iter_", eps_iter_, 1e-4);

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

    ROS_INFO("LQR planner initialized!");
  }
  else
  {
    ROS_WARN("LQR planner has already been initialized.");
  }
}

/**
 * @brief Set the plan that the controller is following
 * @param orig_global_plan the plan to pass to the controller
 * @return true if the plan was updated successfully, else false
 */
bool LQRPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
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
bool LQRPlanner::isGoalReached()
{
  if (!initialized_)
  {
    ROS_ERROR("LQR planner has not been initialized");
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
bool LQRPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_)
  {
    ROS_ERROR("LQR planner has not been initialized");
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
    Eigen::Vector2d u_r(vt, vt * kappa);                                                       // refered input
    Eigen::Vector2d u = _lqrControl(s, s_d, u_r);

    cmd_vel.linear.x = linearRegularization(base_odom, u[0]);
    cmd_vel.angular.z = angularRegularization(base_odom, u[1]);
  }

  // publish lookahead pose
  target_pt_pub_.publish(lookahead_pt);

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
Eigen::Vector2d LQRPlanner::_lqrControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector2d u_r)
{
  Eigen::Vector2d u;
  Eigen::Vector3d e(s - s_d);
  e[2] = regularizeAngle(e[2]);

  // state equation on error
  Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
  A(0, 2) = -u_r[0] * sin(s_d[2]) * d_t_;
  A(1, 2) = u_r[0] * cos(s_d[2]) * d_t_;

  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3, 2);
  B(0, 0) = cos(s_d[2]) * d_t_;
  B(1, 0) = sin(s_d[2]) * d_t_;
  B(2, 1) = d_t_;

  // discrete iteration Ricatti equation
  Eigen::Matrix3d P, P_;
  P = Q_;
  for (int i = 0; i < max_iter_; ++i)
  {
    Eigen::Matrix2d temp = R_ + B.transpose() * P * B;
    P_ = Q_ + A.transpose() * P * A - A.transpose() * P * B * temp.inverse() * B.transpose() * P * A;
    if ((P - P_).array().abs().maxCoeff() < eps_iter_)
      break;
    P = P_;
  }

  // feedback
  Eigen::MatrixXd K = -(R_ + B.transpose() * P_ * B).inverse() * B.transpose() * P_ * A;

  u = u_r + K * e;

  return u;
}

}  // namespace lqr_planner
