#ifndef PID_PLANNER_H_
#define PID_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

namespace pid_planner
{
/**
 * @brief A class implementing a local planner using the PID
 */
class PIDPlanner : public nav_core::BaseLocalPlanner
{
public:
  /**
   * @brief Construct a new PIDPlanner object
   */
  PIDPlanner();

  /**
   * @brief Construct a new PIDPlanner object
   *
   * @param name        The name to give this instance of the trajectory planner
   * @param tf          A pointer to a transform listener
   * @param costmap_ros The cost map to use for assigning costs to trajectories
   */
  PIDPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Destroy the PIDPlanner object
   */
  ~PIDPlanner();

  /**
   * @brief Initialization of the local planner
   *
   * @param name        The name to give this instance of the trajectory planner
   * @param tf          A pointer to a transform listener
   * @param costmap_ros The cost map to use for assigning costs to trajectories
   */
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief  Set the plan that the controller is following
   *
   * @param orig_global_plan The plan to pass to the controller
   * @return True if the plan was updated successfully, false otherwise
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  /**
   * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to
   * the base
   *
   * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
   * @return True if a valid trajectory was found, false otherwise
   */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  /**
   * @brief PID controller in linear
   *
   * @param base_odometry odometry of the robot, to get velocity
   * @param b_x_d         desired x in body frame
   * @param b_y_d         desired y in body frame
   * @return linear velocity
   */
  double LinearPIDController(nav_msgs::Odometry& base_odometry, double b_x_d, double b_y_d);

  /**
   * @brief PID controller in angular
   *
   * @param base_odometry odometry of the robot, to get velocity
   * @param theta_d       desired theta
   * @param theta         current theta
   * @return angular velocity
   */
  double AngularPIDController(nav_msgs::Odometry& base_odometry, double theta_d, double theta);

  /**
   * @brief  Check if the goal pose has been achieved
   *
   * @return True if achieved, false otherwise
   */
  bool isGoalReached();

  /**
   * @brief Get the distance to the goal
   *
   * @param goal_ps global goal PoseStamped
   * @param x       global current x
   * @param y       global current y
   * @return the distance to the goal
   */
  double getGoalPositionDistance(const geometry_msgs::PoseStamped& goal_ps, double x, double y);

  /**
   * @brief Get the Euler Angles from PoseStamped
   *
   * @param ps PoseStamped to calculate
   * @return roll, pitch and yaw in XYZ order
   */
  std::vector<double> getEulerAngles(geometry_msgs::PoseStamped& ps);

private:
  /**
   * @brief Stop the robot
   */
  void robotStops()
  {
    goal_reached_ = true;
    ROS_INFO("Robot will stop.");
  }

  /**
   * @brief Transform pose to body frame
   *
   * @param src   src PoseStamped, the object to transform
   * @param x     result x
   * @param y     result y
   * @param theta result theta
   */
  void getTransformedPosition(geometry_msgs::PoseStamped& src, double* x, double* y, double* theta)
  {
    src.header.stamp = ros::Time(0);
    geometry_msgs::PoseStamped dst;
    tf_->transform(src, dst, base_frame_);
    *x = dst.pose.position.x;
    *y = dst.pose.position.y;
    *theta = tf2::getYaw(dst.pose.orientation);
  }

  costmap_2d::Costmap2DROS* costmap_ros_;
  tf2_ros::Buffer* tf_;
  bool initialized_, goal_reached_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  geometry_msgs::PoseStamped target_ps_, current_ps_;
  int plan_index_;

  double x_, y_, theta_;
  std::vector<double> final_rpy_;

  double p_window_, o_window_;
  double p_precision_, o_precision_;
  double d_t_;
  double e_v_, e_w_;
  double i_v_, i_w_;
  double max_v_, min_v_, max_v_inc_;
  double max_w_, min_w_, max_w_inc_;
  double k_v_p_, k_v_i_, k_v_d_;
  double k_w_p_, k_w_i_, k_w_d_;

  std::string base_frame_;
  base_local_planner::OdometryHelperRos* odom_helper_;
  ros::Publisher target_pose_pub_, current_pose_pub_;
};
};  // namespace pid_planner

#endif