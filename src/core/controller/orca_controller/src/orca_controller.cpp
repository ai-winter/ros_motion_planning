/**
 * *********************************************************
 *
 * @file: orca_controller.h
 * @brief: Contains the ORCA local controller class
 * @author: Yang Haodong, Guo Zhanyu
 * @date: 2024-01-20
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong, Guo Zhanyu.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <pluginlib/class_list_macros.h>

#include "common/util/log.h"
#include "common/geometry/angles.h"
#include "controller/orca_controller.h"

PLUGINLIB_EXPORT_CLASS(rmp::controller::ORCAController, nav_core::BaseLocalPlanner)

using namespace rmp::common::geometry;

namespace rmp {
namespace controller {
ORCAController::ORCAController()
  : initialized_(false)
  , costmap_ros_(nullptr)
  , tf_(nullptr)
  , odom_flag_(false)
  , goal_reached_(false) {
}

ORCAController::ORCAController(std::string name, tf2_ros::Buffer* tf,
                               costmap_2d::Costmap2DROS* costmap_ros)
  : ORCAController() {
  initialize(name, tf, costmap_ros);
}

ORCAController::~ORCAController() {
}

// Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS
// Noetic
void ORCAController::initialize(std::string name, tf2_ros::Buffer* tf,
                                costmap_2d::Costmap2DROS* costmap_ros) {
  if (!initialized_) {
    initialized_ = true;
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    ros::NodeHandle nh = ros::NodeHandle("~/" + name);
    orca_config_ = config_.orca_controller();

    // multi-robot info
    nh.param("agent_number", agent_number_, -1);
    nh.param("agent_id", agent_id_, -1);

    other_odoms_.resize(agent_number_);
    for (int i = 0; i < agent_number_; ++i) {
      ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
          "/robot" + std::to_string(i + 1) + "/odom", 1,
          boost::bind(&ORCAController::odometryCallback, this, _1, i + 1));
      odom_subs_.push_back(odom_sub);
      R_INFO << "agent " << agent_id_ << ", subscribe to agent " << i + 1 << ".";
    }

    int spin_cnt = 5 * agent_number_;
    ros::Rate rate(10);
    R_WARN << "ORCA controller waiting for odoms...";
    while (spin_cnt-- > 0) {
      ros::spinOnce();
      rate.sleep();
    }

    sim_ = std::make_unique<RVO::RVOSimulator>();
    initState();
    R_INFO << "ORCA Controller initialized!";
  } else {
    R_WARN << "ORCA Controller has already been initialized.";
  }
}

bool ORCAController::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
  if (!initialized_) {
    R_ERROR << "This planner has not been initialized";
    return false;
  }

  R_INFO << "Got new plan";

  if (goal_.x() != orig_global_plan.back().pose.position.x ||
      goal_.y() != orig_global_plan.back().pose.position.y) {
    goal_ = RVO::Vector2(orig_global_plan.back().pose.position.x,
                         orig_global_plan.back().pose.position.y);
    goal_reached_ = false;
  }

  return true;
}

bool ORCAController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  if (!initialized_) {
    R_ERROR << "This planner has not been initialized";
    return false;
  }

  nav_msgs::Odometry agent_odom = other_odoms_[agent_id_ - 1];
  RVO::Vector2 curr_pose(agent_odom.pose.pose.position.x,
                         agent_odom.pose.pose.position.y);
  if (RVO::abs(goal_ - curr_pose) < config_.goal_dist_tolerance()) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    goal_reached_ = true;
  } else {
    updateState();
    RVO::Vector2 new_speed = sim_->getAgentNewSpeed(agent_id_ - 1);

    // transform from linear.x and linear.y to linear.x and angular.z
    const double theta = tf2::getYaw(agent_odom.pose.pose.orientation);
    const double v_d = RVO::abs(new_speed);
    const double theta_d = std::atan2(new_speed.y(), new_speed.x());
    const double e_theta = normalizeAngle(theta_d - theta);
    const double w_d = e_theta / control_dt_;
    const double vt =
        std::hypot(agent_odom.twist.twist.linear.x, agent_odom.twist.twist.linear.y);
    const double wt = agent_odom.twist.twist.angular.z;

    cmd_vel.linear.x = linearRegularization(vt, v_d);
    cmd_vel.angular.z = angularRegularization(wt, w_d);
  }

  return true;
}

bool ORCAController::isGoalReached() {
  if (!initialized_) {
    R_ERROR << "ORCA Controller has not been initialized";
    return false;
  }

  if (goal_reached_) {
    R_INFO << "GOAL Reached!";
    return true;
  }
  return false;
}

void ORCAController::initState() {
  if (!odom_flag_) {
    R_ERROR << "Odom not received!";
    return;
  }

  sim_->setTimeStep(control_dt_);
  sim_->setAgentDefaults(orca_config_.neighbor_distance(), orca_config_.max_neighbors(),
                         orca_config_.time_horizon(),
                         orca_config_.time_horizon_obstacles(),
                         orca_config_.agent_radius(), config_.max_linear_velocity());

  for (int i = 0; i < agent_number_; ++i) {
    sim_->addAgent(RVO::Vector2(other_odoms_[i].pose.pose.position.x,
                                other_odoms_[i].pose.pose.position.y));
    sim_->setAgentVelocity(i, RVO::Vector2(other_odoms_[i].twist.twist.linear.x,
                                           other_odoms_[i].twist.twist.linear.y));
  }
}

void ORCAController::odometryCallback(const nav_msgs::OdometryConstPtr& msg,
                                      int agent_id) {
  if (!odom_flag_)
    odom_flag_ = true;

  other_odoms_[agent_id - 1] = *msg;
}

void ORCAController::updateState() {
  for (int i = 0; i < agent_number_; ++i) {
    sim_->setAgentPosition(i, RVO::Vector2(other_odoms_[i].pose.pose.position.x,
                                           other_odoms_[i].pose.pose.position.y));
    sim_->setAgentVelocity(i, RVO::Vector2(other_odoms_[i].twist.twist.linear.x,
                                           other_odoms_[i].twist.twist.linear.y));
  }

  sim_->setAgentPrefVelocity(
      agent_id_ - 1, RVO::normalize(goal_ - sim_->getAgentPosition(agent_id_ - 1)) *
                         config_.max_linear_velocity());
}

}  // namespace controller
}  // namespace rmp