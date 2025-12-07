/**
 * *********************************************************
 *
 * @file: controller.h
 * @brief: Contains the abstract Controller class
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
#ifndef RMP_CONTROLLER_CONTROLLER_H_
#define RMP_CONTROLLER_CONTROLLER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/odometry_helper_ros.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include "common/geometry/point.h"
#include "system_config/controller_protos/controller.pb.h"

namespace rmp {
namespace controller {
class Controller {
public:
  /**
   * @brief Construct a new Controller object
   */
  Controller();

  /**
   * @brief Destroy the Local Planner object
   */
  ~Controller();

  /**
   * @brief Get the Yaw Angles from PoseStamped
   * @param ps  PoseStamped to calculate
   * @return  yaw
   */
  double getYawAngle(geometry_msgs::PoseStamped& ps);

  /**
   * @brief Whether to reach the target pose through rotation operation
   * @param cur   current pose of robot
   * @param goal  goal pose of robot
   * @return true if robot should perform rotation
   */
  bool shouldRotateToGoal(const geometry_msgs::PoseStamped& cur,
                          const geometry_msgs::PoseStamped& goal);

  /**
   * @brief Whether to correct the tracking path with rotation operation
   * @param angle_to_path  the angle deviation
   * @return true if robot should perform rotation
   */
  bool shouldRotateToPath(double angle_to_path);

  /**
   * @brief linear velocity regularization
   * @param v_in  raw linear velocity
   * @param v_d   desired linear velocity
   * @return v    regulated linear velocity
   */
  double linearRegularization(double v_in, double v_d);

  /**
   * @brief angular velocity regularization
   * @param w_in  raw angular velocity
   * @param w_d   desired angular velocity
   * @return  w   regulated angular velocity
   */
  double angularRegularization(double w_in, double w_d);

  /**
   * @brief Tranform from in_pose to out_pose with out frame using tf
   */
  void transformPose(tf2_ros::Buffer* tf, const std::string out_frame,
                     const geometry_msgs::PoseStamped& in_pose,
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
   * @brief Prune the path, removing the waypoints that the robot has already passed and
   * distant waypoints
   * @param robot_pose_global the robot's pose  [global]
   * @return pruned path
   */
  std::vector<geometry_msgs::PoseStamped>
  prune(const geometry_msgs::PoseStamped robot_pose_global);

  /**
   * @brief find the point on the path that is exactly the lookahead distance away from
   * the robot
   * @param lookahead_dist    the lookahead distance
   * @param robot_pose_global the robot's pose  [global]
   * @param prune_plan        the pruned plan
   * @param pt                the lookahead point
   * @param kappa             the curvature on traj
   */
  void getLookAheadPoint(double lookahead_dist,
                         geometry_msgs::PoseStamped robot_pose_global,
                         const std::vector<geometry_msgs::PoseStamped>& prune_plan,
                         common::geometry::Point3d* pt, double* kappa);

protected:
  pb::controller::Controller config_;
  double control_dt_;
  std::shared_ptr<base_local_planner::OdometryHelperRos> odom_helper_;  // odometry helper
  costmap_2d::Costmap2DROS* costmap_ros_;  // costmap(ROS wrapper)
  std::vector<geometry_msgs::PoseStamped> global_plan_;
};
}  // namespace controller
}  // namespace rmp
#endif