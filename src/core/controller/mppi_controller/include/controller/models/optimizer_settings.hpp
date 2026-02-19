/**
 * *********************************************************
 *
 * @file: optimizer_settings.hpp
 * @brief: Optimizer settings for MPPI controller.
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

#ifndef RMP_CONTROLLER_MPPI_OPTIMIZER_SETTINGS_HPP_
#define RMP_CONTROLLER_MPPI_OPTIMIZER_SETTINGS_HPP_

#include "controller/models/constraints.hpp"

namespace rmp::controller::mppi {

/**
 * @struct mppi::models::OptimizerSettings
 * @brief Settings for the optimizer to use
 */
struct OptimizerSettings {
  ControlConstraints base_constraints{ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                       0.0f, 0.0f, 0.0f, 0.0f };
  ControlConstraints constraints{ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
  SamplingStd sampling_std{ 0.0f, 0.0f, 0.0f };
  float model_dt{ 0.0f };
  float temperature{ 0.0f };
  float gamma{ 0.0f };
  unsigned int batch_size{ 0u };
  unsigned int time_steps{ 0u };
  unsigned int iteration_count{ 0u };
  bool shift_control_sequence{ false };
  unsigned int retry_attempt_limit{ 0u };
  bool open_loop{ false };
};

}  // namespace rmp::controller::mppi

#endif  // RMP_CONTROLLER_MPPI_OPTIMIZER_SETTINGS_HPP_