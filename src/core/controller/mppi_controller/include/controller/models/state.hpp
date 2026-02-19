/**
 * *********************************************************
 *
 * @file: state.hpp
 * @brief: State definition for models in MPPI controller.
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

#ifndef RMP_CONTROLLER_MPPI_STATE_HPP_
#define RMP_CONTROLLER_MPPI_STATE_HPP_

#include <Eigen/Dense>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

namespace rmp::controller::mppi {

/**
 * @struct rmp::controller::mppi::State
 * @brief State information: velocities, controls, poses, speed
 */
struct State {
  Eigen::ArrayXXf vx;
  Eigen::ArrayXXf vy;
  Eigen::ArrayXXf wz;

  Eigen::ArrayXXf cvx;
  Eigen::ArrayXXf cvy;
  Eigen::ArrayXXf cwz;

  geometry_msgs::PoseStamped pose;
  geometry_msgs::Twist speed;
  float local_path_length;

  /**
   * @brief Reset state data
   */
  void reset(unsigned int batch_size, unsigned int time_steps) {
    vx.setZero(batch_size, time_steps);
    vy.setZero(batch_size, time_steps);
    wz.setZero(batch_size, time_steps);

    cvx.setZero(batch_size, time_steps);
    cvy.setZero(batch_size, time_steps);
    cwz.setZero(batch_size, time_steps);
  }
};

}  // namespace rmp::controller::mppi

#endif  // RMP_CONTROLLER_MPPI_STATE_HPP_