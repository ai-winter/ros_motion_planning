/**
 * *********************************************************
 *
 * @file: controller.cpp
 * @brief: Contains the abstract local controller class
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
#include <tf2/utils.h>

#include "common/geometry/angles.h"
#include "common/geometry/point.h"
#include "common/math/math_helper.h"

#include "controller/controller.h"
#include "system_config/system_config.h"

using namespace rmp::common::math;
using namespace rmp::common::geometry;

namespace rmp {
namespace controller {
/**
 * @brief Construct a new Controller object
 */
Controller::Controller()
  : config_(system_config::SystemConfigPtr::Instance()->configure().controller())
  , costmap_ros_(nullptr) {
  control_dt_ = 1.0 / config_.control_frequency();
  odom_helper_ =
      std::make_shared<base_local_planner::OdometryHelperRos>(config_.odom_frame());
}

/**
 * @brief Destroy the Controller object
 */
Controller::~Controller() {
  // delete odom_helper_;
}

/**
 * @brief Get the Yaw Angles from PoseStamped
 * @param ps  PoseStamped to calculate
 * @return  yaw
 */
double Controller::getYawAngle(geometry_msgs::PoseStamped& ps) {
  return quaternionToHeading(ps.pose.orientation.x, ps.pose.orientation.y,
                             ps.pose.orientation.z, ps.pose.orientation.w);
}

/**
 * @brief Whether to reach the target pose through rotation operation
 * @param cur  current pose of robot
 * @param goal goal pose of robot
 * @return true if robot should perform rotation
 */
bool Controller::shouldRotateToGoal(const geometry_msgs::PoseStamped& cur,
                                    const geometry_msgs::PoseStamped& goal) {
  return std::hypot(cur.pose.position.x - goal.pose.position.x,
                    cur.pose.position.y - goal.pose.position.y) <
         config_.goal_dist_tolerance();
}

/**
 * @brief Whether to correct the tracking path with rotation operation
 * @param angle_to_path the angle deviation
 * @return true if robot should perform rotation
 */
bool Controller::shouldRotateToPath(double angle_to_path) {
  return angle_to_path > config_.rotate_tolerance();
}

/**
 * @brief linear velocity regularization
 * @param v_in  raw linear velocity
 * @param v_d   desired linear velocity
 * @return v    regulated linear velocity
 */
double Controller::linearRegularization(double v_in, double v_d) {
  double v_inc = v_d - v_in;

  if (std::fabs(v_inc) > config_.max_linear_velocity_increment()) {
    v_inc = std::copysign(config_.max_linear_velocity_increment(), v_inc);
  }

  double v_cmd = v_in + v_inc;
  if (std::fabs(v_cmd) > config_.max_linear_velocity()) {
    v_cmd = std::copysign(config_.max_linear_velocity(), v_cmd);
  } else if (std::fabs(v_cmd) < config_.min_linear_velocity()) {
    v_cmd = std::copysign(config_.min_linear_velocity(), v_cmd);
  }

  return v_cmd;
}

/**
 * @brief angular velocity regularization
 * @param w_in  raw angular velocity
 * @param w_d   desired angular velocity
 * @return  w   regulated angular velocity
 */
double Controller::angularRegularization(double w_in, double w_d) {
  if (std::fabs(w_d) > config_.max_angular_velocity()) {
    w_d = std::copysign(config_.max_angular_velocity(), w_d);
  }

  double w_inc = w_d - w_in;

  if (std::fabs(w_inc) > config_.max_angular_velocity_increment()) {
    w_inc = std::copysign(config_.max_angular_velocity_increment(), w_inc);
  }

  double w_cmd = w_in + w_inc;
  if (std::fabs(w_cmd) > config_.max_angular_velocity()) {
    w_cmd = std::copysign(config_.max_angular_velocity(), w_cmd);
  } else if (std::fabs(w_cmd) < config_.min_angular_velocity()) {
    w_cmd = std::copysign(config_.min_angular_velocity(), w_cmd);
  }

  return w_cmd;
}

/**
 * @brief Tranform from in_pose to out_pose with out frame using tf
 */
void Controller::transformPose(tf2_ros::Buffer* tf, const std::string out_frame,
                               const geometry_msgs::PoseStamped& in_pose,
                               geometry_msgs::PoseStamped& out_pose) const {
  if (in_pose.header.frame_id == out_frame)
    out_pose = in_pose;

  tf->transform(in_pose, out_pose, out_frame);
  out_pose.header.frame_id = out_frame;
}

/**
 * @brief Tranform from world map(x, y) to costmap(x, y)
 * @param mx costmap x
 * @param my costmap y
 * @param wx world map x
 * @param wy world map y
 * @return true if successfull, else false
 */
bool Controller::worldToMap(double wx, double wy, int& mx, int& my) {
  unsigned int mx_u, my_u;
  bool flag = costmap_ros_->getCostmap()->worldToMap(wx, wy, mx_u, my_u);
  mx = static_cast<int>(mx_u);
  my = static_cast<int>(my_u);
  return flag;
}

/**
 * @brief Prune the path, removing the waypoints that the robot has already passed and
 * distant waypoints
 * @param robot_pose_global the robot's pose  [global]
 * @return pruned path
 */
std::vector<geometry_msgs::PoseStamped>
Controller::prune(const geometry_msgs::PoseStamped robot_pose_global) {
  auto calPoseDistance = [](const geometry_msgs::PoseStamped& ps_1,
                            const geometry_msgs::PoseStamped& ps_2) {
    return std::hypot(ps_1.pose.position.x - ps_2.pose.position.x,
                      ps_1.pose.position.y - ps_2.pose.position.y);
  };

  /**
   * @brief Find the first element in iterator that is greater integrated distance than
   * compared value
   * @param begin   The begin of iterator
   * @param end     The end of iterator
   * @param dist    The distance metric function
   * @param cmp_val The compared value
   * @return it     The first element in iterator that is greater integrated distance than
   * compared value
   */
  auto firstIntegratedDistance =
      [](std::vector<geometry_msgs::PoseStamped>::iterator begin,
         std::vector<geometry_msgs::PoseStamped>::iterator end,
         std::function<double(const geometry_msgs::PoseStamped&,
                              const geometry_msgs::PoseStamped&)>
             dist,
         double cmp_val) {
        if (begin == end)
          return end;
        double d = 0.0;
        for (auto it = begin; it != end - 1; it++) {
          d += dist(*it, *(it + 1));
          if (d > cmp_val)
            return it + 1;
        }
        return end;
      };

  /**
   * @brief Find the first element in iterator with the minimum calculated value
   * @param begin   The begin of iterator
   * @param end     The end of iterator
   * @param cal     The customer calculated function
   * @return it     The first element in iterator with the minimum calculated value
   */
  auto getMinFuncVal = [](std::vector<geometry_msgs::PoseStamped>::iterator begin,
                          std::vector<geometry_msgs::PoseStamped>::iterator end,
                          std::function<double(const geometry_msgs::PoseStamped&)> cal) {
    if (begin == end)
      return end;

    auto min_val = cal(*begin);
    auto min_iter = begin;
    for (auto it = ++begin; it != end; it++) {
      auto val = cal(*it);
      if (val <= min_val) {
        min_val = val;
        min_iter = it;
      }
    }
    return min_iter;
  };

  auto closest_pose_upper_bound =
      firstIntegratedDistance(global_plan_.begin(), global_plan_.end(), calPoseDistance,
                              costmap_ros_->getCostmap()->getSizeInMetersX() / 2.0);

  // find the closest pose on the path to the robot
  auto transform_begin = getMinFuncVal(global_plan_.begin(), closest_pose_upper_bound,
                                       [&](const geometry_msgs::PoseStamped& ps) {
                                         return calPoseDistance(robot_pose_global, ps);
                                       });

  // Transform the near part of the global plan into the robot's frame of reference.
  std::vector<geometry_msgs::PoseStamped> prune_path;
  for (auto it = transform_begin; it < global_plan_.end(); it++)
    prune_path.push_back(*it);

  // path pruning: remove the portion of the global plan that already passed so don't
  // process it on the next iteration
  global_plan_.erase(std::begin(global_plan_), transform_begin);

  return prune_path;
}

/**
 * @brief find the point on the path that is exactly the lookahead distance away from the
 * robot
 * @param lookahead_dist    the lookahead distance
 * @param robot_pose_global the robot's pose  [global]
 * @param prune_plan        the pruned plan
 * @param pt                the lookahead point
 * @param kappa             the curvature on traj
 */
void Controller::getLookAheadPoint(
    double lookahead_dist, geometry_msgs::PoseStamped robot_pose_global,
    const std::vector<geometry_msgs::PoseStamped>& prune_plan, Point3d* pt,
    double* kappa) {
  double rx = robot_pose_global.pose.position.x;
  double ry = robot_pose_global.pose.position.y;

  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(
      prune_plan.begin(), prune_plan.end(), [&](const geometry_msgs::PoseStamped& ps) {
        return std::hypot(ps.pose.position.x - robot_pose_global.pose.position.x,
                          ps.pose.position.y - robot_pose_global.pose.position.y) >=
               lookahead_dist;
      });

  // If the no pose is not far enough, take the last pose
  if (goal_pose_it == prune_plan.end()) {
    goal_pose_it = std::prev(prune_plan.end());
    pt->setX(goal_pose_it->pose.position.x);
    pt->setY(goal_pose_it->pose.position.y);
    pt->setTheta(std::atan2(pt->y() - ry, pt->x() - rx));
    *kappa = 0.0;
  } else {
    // find the point on the line segment between the two poses
    // that is exactly the lookahead distance away from the robot pose (the origin)
    // This can be found with a closed form for the intersection of a segment and a circle
    // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside
    // the circle, and goal_pose is guaranteed to be outside the circle.
    double px, py;
    double gx = goal_pose_it->pose.position.x;
    double gy = goal_pose_it->pose.position.y;
    if (goal_pose_it == prune_plan.begin()) {
      px = rx;
      py = ry;
    } else {
      auto prev_pose_it = std::prev(goal_pose_it);
      px = prev_pose_it->pose.position.x;
      py = prev_pose_it->pose.position.y;
    }

    // transform to the robot frame so that the circle centers at (0,0)
    common::geometry::Vec2d prev_p(px - rx, py - ry);
    common::geometry::Vec2d goal_p(gx - rx, gy - ry);
    std::vector<rmp::common::geometry::Vec2d> i_points =
        circleSegmentIntersection(prev_p, goal_p, lookahead_dist);

    double dist_to_goal = std::numeric_limits<double>::max();
    for (const auto& i_point : i_points) {
      double dist = std::hypot(i_point.x() + rx - gx, i_point.y() + ry - gy);
      if (dist < dist_to_goal) {
        dist_to_goal = dist;
        pt->setX(i_point.x() + rx);
        pt->setY(i_point.y() + ry);
      }
    }

    auto next_pose_it = std::next(goal_pose_it);
    if (next_pose_it != prune_plan.end()) {
      common::geometry::Vec2d p1(px, py), p2(gx, gy),
          p3(next_pose_it->pose.position.x, next_pose_it->pose.position.y);
      *kappa = arcCenter(p1, p2, p3, false);
    } else {
      *kappa = 0.0;
    }
    pt->setTheta(std::atan2(gy - py, gx - px));
  }
}

}  // namespace controller
}  // namespace rmp