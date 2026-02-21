/**
 * *********************************************************
 *
 * @file: critic_data.hpp
 * @brief: Critic data definition for models in MPPI controller.
 * @author: Zhanyu Guo
 * @date: 2026-02-19
 * @version: 1.0
 *
 * Copyright (c) 2026, Zhanyu Guo.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */

#ifndef RMP_CONTROLLER_MPPI_CRITIC_DATA_HPP_
#define RMP_CONTROLLER_MPPI_CRITIC_DATA_HPP_

#include <Eigen/Dense>

#include <memory>
#include <vector>
#include <optional>

#include <geometry_msgs/PoseStamped.h>

#include "controller/models/state.hpp"
#include "controller/models/path.hpp"
#include "controller/models/trajectories.hpp"
#include "controller/motion_models.hpp"

namespace rmp::controller::mppi {

/**
 * @struct rmp::controller::mppi::CriticData
 * @brief Data to pass to critics for scoring, including state, trajectories,
 * pruned path, global goal, costs, and important parameters to share
 */
struct CriticData {
  const models::State& state;
  const models::Trajectories& trajectories;
  const models::Path& path;
  const geometry_msgs::Pose& goal;

  Eigen::ArrayXf& costs;
  float& model_dt;

  bool fail_flag;
  std::shared_ptr<MotionModel> motion_model;
  std::optional<std::vector<bool>> path_pts_valid;
  std::optional<size_t> furthest_reached_path_point;
};

}  // namespace rmp::controller::mppi

#endif  // RMP_CONTROLLER_MPPI_CRITIC_DATA_HPP_