/**
 * *********************************************************
 *
 * @file: control_sequence.hpp
 * @brief: Control sequence definition for MPPI controller.
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

#ifndef RMP_CONTROLLER_MPPI_CONTROL_SEQUENCE_HPP_
#define RMP_CONTROLLER_MPPI_CONTROL_SEQUENCE_HPP_

#include <Eigen/Dense>

namespace rmp::controller::mppi::models {

/**
 * @struct rmp::controller::mppi::models::Control
 * @brief A set of controls
 */
struct Control {
  float vx, vy, wz;
};

/**
 * @struct rmp::controller::mppi::models::ControlSequence
 * @brief A control sequence over time (e.g. trajectory)
 */
struct ControlSequence {
  Eigen::ArrayXf vx;
  Eigen::ArrayXf vy;
  Eigen::ArrayXf wz;

  void reset(unsigned int time_steps) {
    vx.setZero(time_steps);
    vy.setZero(time_steps);
    wz.setZero(time_steps);
  }
};

}  // namespace rmp::controller::mppi::models

#endif  // RMP_CONTROLLER_MPPI_CONTROL_SEQUENCE_HPP_