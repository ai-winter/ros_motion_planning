/**
 * *********************************************************
 *
 * @file: cost_critic.cpp
 * @brief: Cost critic used in MPPI controller.
 * @author: Zhanyu Guo
 * @date: 2026-02-21
 * @version: 1.0
 *
 * Copyright (c) 2026, Zhanyu Guo.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */

#include "controller/critics/cost_critic.hpp"

namespace rmp::controller::mppi::critics {

void CostCritic::initialize() {
  enabled_ = mppi_config_->cost_critic_enabled();
  power_ = mppi_config_->cost_critic_cost_power();
  weight_ = mppi_config_->cost_critic_cost_weight();
  critical_cost_ = mppi_config_->cost_critic_critical_cost();
  near_collision_cost_ = mppi_config_->cost_critic_near_collision_cost();
  collision_cost_ = mppi_config_->cost_critic_collision_cost();
  near_goal_distance_ = mppi_config_->cost_critic_near_goal_distance();
  trajectory_point_step_ = mppi_config_->trajectory_point_step();

  // Normalized by cost value to put in same regime as other weights
  weight_ /= 254.0f;

  collision_checker_.setCostmap(costmap_);
  possible_collision_cost_ = findCircumscribedCost(costmap_ros_);

  if (possible_collision_cost_ < 1.0f) {
    R_ERROR << "Inflation layer either not found or inflation is not set sufficiently "
               "for optimized non-circular collision checking capabilities. It is "
               "HIGHLY recommended to set the inflation radius to be at MINIMUM half of "
               "the robot's largest cross-section. See "
               "github.com/ros-planning/navigation2/tree/main/"
               "nav2_smac_planner#potential-fields for full instructions. This will "
               "substantially impact run-time performance.";
  }

  if (near_collision_cost_ > 253) {
    R_WARN << "Near collision cost is set higher than INSCRIBED_INFLATED_OBSTACLE";
  }

  R_INFO << "InflationCostCritic instantiated with " << power_ << " power and "
         << critical_cost_ << " / " << weight_ << " weights. Critic will collision check "
         << (consider_footprint_ ? "based on footprint" : "based on circular");
}

float CostCritic::findCircumscribedCost(costmap_2d::Costmap2DROS* costmap) {
  double result = -1.0;
  const double circum_radius = costmap->getLayeredCostmap()->getCircumscribedRadius();
  if (static_cast<float>(circum_radius) == circumscribed_radius_) {
    // early return if footprint size is unchanged
    return circumscribed_cost_;
  }

  // check if the costmap has an inflation layer
  const auto inflation_layer = costmap_2d::getInflationLayer(costmap);
  if (inflation_layer != nullptr) {
    const double resolution = costmap->getCostmap()->getResolution();
    result = inflation_layer->computeCost(circum_radius / resolution);
  } else {
    R_WARN << "No inflation layer found in costmap configuration. If this is an "
              "SE2-collision checking plugin, it cannot use costmap potential "
              "field to speed up collision checking by only checking the full footprint "
              "when robot is within possibly-inscribed radius of an obstacle. This may "
              "significantly slow down planning times and not avoid anything but "
              "absolute collisions!";
  }

  circumscribed_radius_ = static_cast<float>(circum_radius);
  circumscribed_cost_ = static_cast<float>(result);

  return circumscribed_cost_;
}

void CostCritic::score(CriticData& data) {
  if (!enabled_) {
    return;
  }

  // Setup cost information for various parts of the critic
  is_tracking_unknown_ = costmap_ros_->getLayeredCostmap()->isTrackingUnknown();
  auto* costmap = collision_checker_.getCostmap();
  origin_x_ = static_cast<float>(costmap->getOriginX());
  origin_y_ = static_cast<float>(costmap->getOriginY());
  resolution_ = static_cast<float>(costmap->getResolution());
  size_x_ = costmap->getSizeInCellsX();
  size_y_ = costmap->getSizeInCellsY();

  if (consider_footprint_) {
    // footprint may have changed since initialization if user has dynamic footprints
    possible_collision_cost_ = findCircumscribedCost(costmap_ros_);
  }

  // If near the goal, don't apply the preferential term since the goal is near obstacles
  bool near_goal = false;
  if (data.state.local_path_length < near_goal_distance_) {
    near_goal = true;
  }

  Eigen::ArrayXf repulsive_cost(data.costs.rows());
  repulsive_cost.setZero();
  bool all_trajectories_collide = true;

  int strided_traj_cols =
      floor((data.trajectories.x.cols() - 1) / trajectory_point_step_) + 1;
  int strided_traj_rows = data.trajectories.x.rows();
  int outer_stride = strided_traj_rows * trajectory_point_step_;

  const auto traj_x = Eigen::Map<const Eigen::ArrayXXf, 0, Eigen::Stride<-1, -1>>(
      data.trajectories.x.data(), strided_traj_rows, strided_traj_cols,
      Eigen::Stride<-1, -1>(outer_stride, 1));
  const auto traj_y = Eigen::Map<const Eigen::ArrayXXf, 0, Eigen::Stride<-1, -1>>(
      data.trajectories.y.data(), strided_traj_rows, strided_traj_cols,
      Eigen::Stride<-1, -1>(outer_stride, 1));
  const auto traj_yaw = Eigen::Map<const Eigen::ArrayXXf, 0, Eigen::Stride<-1, -1>>(
      data.trajectories.yaws.data(), strided_traj_rows, strided_traj_cols,
      Eigen::Stride<-1, -1>(outer_stride, 1));

  for (int i = 0; i < strided_traj_rows; ++i) {
    bool trajectory_collide = false;
    float pose_cost = 0.0f;
    float& traj_cost = repulsive_cost(i);

    for (int j = 0; j < strided_traj_cols; j++) {
      float Tx = traj_x(i, j);
      float Ty = traj_y(i, j);
      unsigned int x_i = 0u, y_i = 0u;

      // The getCost doesn't use orientation
      // The footprintCostAtPose will always return "INSCRIBED" if footprint is over it
      // So the center point has more information than the footprint
      if (!worldToMapFloat(Tx, Ty, x_i, y_i)) {
        pose_cost = 255.0f;  // NO_INFORMATION in float
      } else {
        pose_cost = static_cast<float>(costmap->getCost(x_i, y_i));
        if (pose_cost < 1.0f) {
          continue;  // In free space
        }
      }

      if (inCollision(pose_cost, Tx, Ty, traj_yaw(i, j))) {
        traj_cost = collision_cost_;
        trajectory_collide = true;
        break;
      }

      // Let near-collision trajectory points be punished severely
      // Note that we collision check based on the footprint actual,
      // but score based on the center-point cost regardless
      if (pose_cost >= static_cast<float>(near_collision_cost_)) {
        traj_cost += critical_cost_;
      } else if (!near_goal) {  // Generally prefer trajectories further from obstacles
        traj_cost += pose_cost;
      }
    }

    all_trajectories_collide &= trajectory_collide;
  }

  if (power_ > 1u) {
    data.costs +=
        (repulsive_cost * (weight_ / static_cast<float>(strided_traj_cols))).pow(power_);
  } else {
    data.costs += repulsive_cost * (weight_ / static_cast<float>(strided_traj_cols));
  }

  data.fail_flag = all_trajectories_collide;
}

}  // namespace rmp::controller::mppi::critics

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmp::controller::mppi::critics::CostCritic,
                       rmp::controller::mppi::critics::CriticFunction)
