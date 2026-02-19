/**
 * *********************************************************
 *
 * @file: constraints.hpp
 * @brief: Constraints definition for models in MPPI controller.
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

#ifndef RMP_CONTROLLER_MPPI_CONSTRAINTS_HPP_
#define RMP_CONTROLLER_MPPI_CONSTRAINTS_HPP_

namespace rmp::controller::mppi {

/**
 * @struct rmp::controller::mppi::ControlConstraints
 * @brief Constraints on control
 */
struct ControlConstraints {
  float vx_max;
  float vx_min;
  float vy;
  float wz;
  float ax_max;
  float ax_min;
  float ay_min;
  float ay_max;
  float az_max;
};

/**
 * @struct rmp::controller::models::SamplingStd
 * @brief Noise parameters for sampling trajectories
 */
struct SamplingStd {
  float vx;
  float vy;
  float wz;
};

}  // namespace rmp::controller::mppi

#endif  // RMP_CONTROLLER_MPPI_CONSTRAINTS_HPP_