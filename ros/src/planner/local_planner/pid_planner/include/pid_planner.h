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
class PIDPlanner : public nav_core::BaseLocalPlanner
{
public:
  PIDPlanner();
  PIDPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  ~PIDPlanner();

  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  double LinearPIDController(nav_msgs::Odometry& base_odometry, double b_x_d, double b_y_d);

  double AngularPIDController(nav_msgs::Odometry& base_odometry, double target_th_w, double robot_orien);

  bool isGoalReached();

  double getGoalPositionDistance(const geometry_msgs::PoseStamped& g_goal_ps, double g_x, double g_y);

  std::vector<double> getEulerAngles(geometry_msgs::PoseStamped& ps);

private:
  void robotStops()
  {
    goal_reached_ = true;
    ROS_INFO("Robot will stop.");
  }

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
  int plan_index_, last_plan_index_;

  double g_x_, g_y_, g_theta_;
  std::vector<double> g_final_rpy_;

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
  ros::Subscriber emergency_stop_sub_;
};
};  // namespace pid_planner

#endif