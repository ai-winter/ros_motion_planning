/**
 * *********************************************************
 *
 * @file: local_planner.h
 * @brief: Contains the abstract local planner class
 * @author: Yang Haodong
 * @date: 2024-01-20
 * @version: 1.3
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#define INFINITE_COST 1e10   // infinite cost
#define LETHAL_COST 253      // lethal cost
#define NEUTRAL_COST 50      // neutral cost
#define OBSTACLE_FACTOR 0.5  // obstacle factor

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/odometry_helper_ros.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <Eigen/Dense>

#include "math_helper.h"

namespace local_planner
{
class LocalPlanner
{
public:
  /**
   * @brief Construct a new Local Planner object
   */
  LocalPlanner();

  /**
   * @brief Destroy the Local Planner object
   */
  ~LocalPlanner();

  /**
   * @brief Set or reset costmap size
   * @param nx  pixel number in costmap x direction
   * @param ny  pixel number in costmap y direction
   */
  void setSize(int nx, int ny);

  /**
   * @brief Set or reset costmap resolution
   * @param resolution  costmap resolution
   */
  void setResolution(double resolution);

  /**
   * @brief Set or reset costmap origin
   * @param origin_x  origin in costmap x direction
   * @param origin_y  origin in costmap y direction
   */
  void setOrigin(double origin_x, double origin_y);

  /**
   * @brief Set or reset lethal cost
   * @param lethal_cost lethal cost
   */
  void setLethalCost(unsigned char lethal_cost);

  /**
   * @brief Set or reset neutral cost
   * @param neutral_cost  neutral cost
   */
  void setNeutralCost(unsigned char neutral_cost);

  /**
   * @brief Set or reset obstacle factor
   * @param factor  obstacle factor
   */
  void setFactor(double factor);

  /**
   * @brief Set or reset frame name
   * @param frame_name
   */
  void setBaseFrame(std::string base_frame);
  void setMapFrame(std::string map_frame);

  /**
   * @brief Regularize angle to [-pi, pi]
   * @param angle the angle (rad) to regularize
   * @return reg_angle the regulated angle
   */
  double regularizeAngle(double angle);

  /**
   * @brief Get the Euler Angles from PoseStamped
   * @param ps  PoseStamped to calculate
   * @return  roll, pitch and yaw in XYZ order
   */
  Eigen::Vector3d getEulerAngles(geometry_msgs::PoseStamped& ps);

  /**
   * @brief Whether to reach the target pose through rotation operation
   * @param cur   current pose of robot
   * @param goal  goal pose of robot
   * @return true if robot should perform rotation
   */
  bool shouldRotateToGoal(const geometry_msgs::PoseStamped& cur, const geometry_msgs::PoseStamped& goal);

  /**
   * @brief Whether to correct the tracking path with rotation operation
   * @param angle_to_path  the angle deviation
   * @return true if robot should perform rotation
   */
  bool shouldRotateToPath(double angle_to_path, double tolerance = 0.0);

  /**
   * @brief linear velocity regularization
   * @param base_odometry odometry of the robot, to get velocity
   * @param v_d           desired velocity magnitude
   * @return v            regulated linear velocity
   */
  double linearRegularization(nav_msgs::Odometry& base_odometry, double v_d);

  /**
   * @brief angular velocity regularization
   * @param base_odometry odometry of the robot, to get velocity
   * @param w_d           desired angular velocity
   * @return  w           regulated angular velocity
   */
  double angularRegularization(nav_msgs::Odometry& base_odometry, double w_d);

  /**
   * @brief Tranform from in_pose to out_pose with out frame using tf
   */
  void transformPose(tf2_ros::Buffer* tf, const std::string out_frame, const geometry_msgs::PoseStamped& in_pose,
                     geometry_msgs::PoseStamped& out_pose) const;

  /**
   * @brief Tranform from world map(x, y) to costmap(x, y)
   * @param mx  costmap x
   * @param my  costmap y
   * @param wx  world map x
   * @param wy  world map y
   * @return true if successfull, else false
   */
  bool worldToMap(double wx, double wy, int& mx, int& my);

  /**
   * @brief Prune the path, removing the waypoints that the robot has already passed and distant waypoints
   * @param robot_pose_global the robot's pose  [global]
   * @return pruned path
   */
  std::vector<geometry_msgs::PoseStamped> prune(const geometry_msgs::PoseStamped robot_pose_global);

  /**
   * @brief Calculate the look-ahead distance with current speed dynamically
   * @param vt the current speed
   * @return L the look-ahead distance
   */
  double getLookAheadDistance(double vt);

  /**
   * @brief find the point on the path that is exactly the lookahead distance away from the robot
   * @param lookahead_dist    the lookahead distance
   * @param robot_pose_global the robot's pose  [global]
   * @param prune_plan        the pruned plan
   * @param pt                the lookahead point
   * @param theta             the angle on traj
   * @param kappa             the curvature on traj
   */
  void getLookAheadPoint(double lookahead_dist, geometry_msgs::PoseStamped robot_pose_global,
                         const std::vector<geometry_msgs::PoseStamped>& prune_plan, geometry_msgs::PointStamped& pt,
                         double& theta, double& kappa);

protected:
  // lethal cost and neutral cost
  unsigned char lethal_cost_, neutral_cost_;

  int nx_, ny_, ns_;            // pixel number in local costmap x, y and total
  double origin_x_, origin_y_;  // local costmap origin
  double resolution_;           // local ostmap resolution
  double convert_offset_;       // offset of transform from world(x,y) to grid map(x,y)
  double factor_;               // obstacle factor(greater means obstacles)

  double max_v_, min_v_, max_v_inc_;  // linear velocity
  double max_w_, min_w_, max_w_inc_;  // angular velocity

  // if the distance is less than the tolerance value, it is considered to have reached the target position
  double goal_dist_tol_;

  // if the angle deviation is greater than this threshold, perform rotation first
  double rotate_tol_;

  // frame name of base link, map and odometry
  std::string base_frame_, map_frame_, odom_frame_;

  // odometry helper
  base_local_planner::OdometryHelperRos* odom_helper_;

  costmap_2d::Costmap2DROS* costmap_ros_;  // costmap(ROS wrapper)
  std::vector<geometry_msgs::PoseStamped> global_plan_;

  double lookahead_time_;      // lookahead time gain
  double min_lookahead_dist_;  // minimum lookahead distance
  double max_lookahead_dist_;  // maximum lookahead distance
};
}  // namespace local_planner

#endif