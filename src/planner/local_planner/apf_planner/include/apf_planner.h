/***********************************************************
*
* @file: apf_planner.h
* @breif: Contains the Artificial Potential Field (APF) local planner class
* @author: Wu Maojia, Yang Haodong
* @update: 2023-10-1
* @version: 1.0
*
* Copyright (c) 2023，Wu Maojia, Yang Haodong
* All rights reserved.
* --------------------------------------------------------
*
**********************************************************/

#ifndef APF_PLANNER_H_
#define APF_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <nav_msgs/Odometry.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <tf2/utils.h>

#include <Eigen/Dense>

namespace apf_planner
{
/**
 * @brief A class implementing a local planner using the APF
 */
class APFPlanner : public nav_core::BaseLocalPlanner
{
public:
  /**
   * @brief Construct a new APFPlanner object
   */
  APFPlanner();

  /**
   * @brief Construct a new APFPlanner object
   */
  APFPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Destroy the APFPlanner object
   */
  ~APFPlanner();

  /**
   * @brief Initialization of the local planner
   * @param name        the name to give this instance of the trajectory planner
   * @param tf          a pointer to a transform listener
   * @param costmap_ros the cost map to use for assigning costs to trajectories
   */
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Set the plan that the controller is following
   * @param orig_global_plan the plan to pass to the controller
   * @return  true if the plan was updated successfully, else false
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  /**
   * @brief  Check if the goal pose has been achieved
   * @return True if achieved, false otherwise
   */
  bool isGoalReached();

  /**
   * @brief Given the current position, orientation, and velocity of the robot, compute the velocity commands
   * @param cmd_vel will be filled with the velocity command to be passed to the robot base
   * @return  true if a valid trajectory was found, else false
   */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  /**
   * @brief APF controller in linear
   * @param base_odometry odometry of the robot, to get velocity
   * @param v_d           desired velocity magnitude
   * @return  linear velocity
   */
  double LinearAPFController(nav_msgs::Odometry& base_odometry, double v_d);

  /**
   * @brief APF controller in angular
   * @param base_odometry odometry of the robot, to get velocity
   * @param theta_d       desired theta
   * @param theta         current theta
   * @return  angular velocity
   */
  double AngularAPFController(nav_msgs::Odometry& base_odometry, double theta_d, double theta);

  /**
   * @brief Get the distance to the goal
   * @param goal_ps global goal PoseStamped
   * @param x       global current x
   * @param y       global current y
   * @return the distance to the goal
   */
  double getGoalPositionDistance(const geometry_msgs::PoseStamped& goal_ps, double x, double y);

  /**
   * @brief Get the attractive force of APF
   * @param ps      global target PoseStamped
   * @param x       global current x
   * @param y       global current y
   * @return the attractive force
   */
  Eigen::Vector2d getAttractiveForce(const geometry_msgs::PoseStamped& ps, double x, double y);

  /**
   * @brief Get the repulsive force of APF
   * @return the repulsive force
   */
  Eigen::Vector2d getRepulsiveForce();

  /**
   * @brief Get the Euler Angles from PoseStamped
   * @param ps  PoseStamped to calculate
   * @return  roll, pitch and yaw in XYZ order
   */
  std::vector<double> getEulerAngles(geometry_msgs::PoseStamped& ps);

  /**
   * @brief Transform pose to body frame
   * @param src   src PoseStamped, the object to transform
   * @param x     result x
   * @param y     result y
   */
  void getTransformedPosition(geometry_msgs::PoseStamped& src, double& x, double& y);

  /**
   * @brief Regularize angle to [-pi, pi]
   * @param angle the angle (rad) to regularize
   */
  void regularizeAngle(double& angle);

protected:
  /**
   * @brief Tranform from world map(x, y) to costmap(x, y)
   * @param mx  costmap x
   * @param my  costmap y
   * @param wx  world map x
   * @param wy  world map y
   * @return true if successfull, else false
   */
  bool _worldToMap(double wx, double wy, int& mx, int& my);

private:
  bool initialized_, goal_reached_;
  tf2_ros::Buffer* tf_;
  costmap_2d::Costmap2DROS* costmap_ros_;     // costmap(ROS wrapper)
  costmap_2d::Costmap2D* costmap_;            // costmap
  unsigned char* costmap_char_;               // costmap char map

  unsigned int nx_, ny_;                      // costmap size
  double origin_x_, origin_y_;                // costmap origin
  double resolution_;                         // costmap resolution

  int plan_index_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  geometry_msgs::PoseStamped target_ps_, current_ps_;

  double x_, y_, theta_;

  double convert_offset_;  // offset of transform from world(x,y) to grid map(x,y)

  double p_window_;                   // next point distance
  double p_precision_, o_precision_;  // goal reached tolerance
  double controller_freqency_, d_t_;

  double max_v_, min_v_, max_v_inc_;  // linear velocity
  double max_w_, min_w_, max_w_inc_;  // angular velocity

  int s_window_;     // trajectory smoothing window time

  double zeta_, eta_;   // scale factor of attractive and repulsive force

  int cost_ub_, cost_lb_;  // the upper and lower bound of costmap used to calculate repulsive force potential field

  std::deque<Eigen::Vector2d> hist_nf_; // historical net forces

  std::string base_frame_, map_frame_;
  base_local_planner::OdometryHelperRos* odom_helper_;
  ros::Publisher target_pose_pub_, current_pose_pub_;

  double goal_x_, goal_y_;
  std::vector<double> goal_rpy_;
};
};  // namespace apf_planner

#endif