#include <pluginlib/class_list_macros.h>

#include "common/util/log.h"
#include "common/util/visualizer.h"
#include "common/math/math_helper.h"
#include "common/geometry/angles.h"
#include "common/geometry/point.h"
#include "controller/mppi_controller.hpp"

PLUGINLIB_EXPORT_CLASS(rmp::controller::MPPIController, nav_core::BaseLocalPlanner)

using namespace rmp::common::math;
using namespace rmp::common::geometry;

namespace rmp {
namespace controller {

MPPIController::MPPIController()
  : initialized_(false), goal_reached_(false), tf_(nullptr) {
}

MPPIController::MPPIController(std::string name, tf2_ros::Buffer* tf,
                               costmap_2d::Costmap2DROS* costmap_ros)
  : MPPIController() {
  initialize(name, tf, costmap_ros);
}

MPPIController::~MPPIController() {
}

void MPPIController::initialize(std::string name, tf2_ros::Buffer* tf,
                                costmap_2d::Costmap2DROS* costmap_ros) {
  if (initialized_) {
    R_WARN << "MPPI Controller has already been initialized.";
    return;
  }

  tf_ = tf;
  costmap_ros_ = costmap_ros;

  ros::NodeHandle nh = ros::NodeHandle("~/" + name);
  mppi_config_ = config_.mppi_controller();

  // TODO

  initialized_ = true;
  R_INFO << "MPPI Controller is initialized.";
}

bool MPPIController::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
  if (!initialized_) {
    R_ERROR << "MPPI Controller has not been initialized";
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
  }

  // TODO

  return true;
}

bool MPPIController::isGoalReached() {
  if (!initialized_) {
    R_ERROR << "MPPI Controller has not been initialized";
    return false;
  }

  if (goal_reached_) {
    R_INFO << "GOAL Reached!";
    return true;
  }
  return false;
}

bool MPPIController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  if (!initialized_) {
    R_ERROR << "MPPI Controller has not been initialized";
    return false;
  }

  // odometry observation - getting robot velocities in odom
  nav_msgs::Odometry base_odom;
  odom_helper_->getOdom(base_odom);
  double vt = std::hypot(base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y);
  double wt = base_odom.twist.twist.angular.z;

  // get robot position in map
  geometry_msgs::PoseStamped robot_pose_odom, robot_pose_map;
  costmap_ros_->getRobotPose(robot_pose_odom);
  transformPose(tf_, config_.map_frame(), robot_pose_odom, robot_pose_map);
  double theta = tf2::getYaw(robot_pose_map.pose.orientation);  // [-pi, pi]

  // prune the global plan
  std::vector<geometry_msgs::PoseStamped> prune_plan = prune(robot_pose_odom);

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
    return true;
  }

  // TODO
  return true;
}

}  // namespace controller
}  // namespace rmp