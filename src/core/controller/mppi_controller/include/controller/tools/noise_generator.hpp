/**
 * *********************************************************
 *
 * @file: noise_generator.hpp
 * @brief: Noise generator for MPPI controller.
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

#ifndef RMP_CONTROLLER_MPPI_NOISE_GENERATOR_HPP_
#define RMP_CONTROLLER_MPPI_NOISE_GENERATOR_HPP_

#include <Eigen/Dense>

#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <random>

#include "controller/models/optimizer_settings.hpp"
#include "controller/models/state.hpp"
#include "controller/models/control_sequence.hpp"
#include "system_config/controller_protos/mppi_controller.pb.h"

namespace rmp::controller::mppi {

/**
 * @class rmp::controller::mppi::NoiseGenerator
 * @brief Generates noise trajectories from optimal trajectory
 */
class NoiseGenerator {
public:
  /**
   * @brief Constructor for rmp::controller::mppi::NoiseGenerator
   */
  NoiseGenerator() = default;

  /**
   * @brief Initialize noise generator with settings and model types
   * @param settings Settings of controller
   * @param is_holonomic If base is holonomic
   * @param name Namespace for configs
   * @param mppi_config MPPI controller config
   */
  void initialize(mppi::models::OptimizerSettings& settings, bool is_holonomic,
                  const std::string& name, pb::controller::MPPIController* mppi_config);

  /**
   * @brief Shutdown noise generator thread
   */
  void shutdown();

  /**
   * @brief Signal to the noise thread the controller is ready to generate a new
   * noised control for the next iteration
   */
  void generateNextNoises();

  /**
   * @brief set noised control_sequence to state controls
   * @return noises vx, vy, wz
   */
  void setNoisedControls(models::State& state,
                         const models::ControlSequence& control_sequence);

  /**
   * @brief Reset noise generator with settings and model types
   * @param settings Settings of controller
   * @param is_holonomic If base is holonomic
   */
  void reset(mppi::models::OptimizerSettings& settings, bool is_holonomic);

protected:
  /**
   * @brief Thread to execute noise generation process
   */
  void noiseThread();

  /**
   * @brief Generate random controls by gaussian noise with mean in
   * control_sequence_
   *
   * @return tensor of shape [ batch_size_, time_steps_, 2]
   * where 2 stands for v, w
   */
  void generateNoisedControls();

  Eigen::ArrayXXf noises_vx_;
  Eigen::ArrayXXf noises_vy_;
  Eigen::ArrayXXf noises_wz_;

  std::default_random_engine generator_;
  std::normal_distribution<float> ndistribution_vx_;
  std::normal_distribution<float> ndistribution_wz_;
  std::normal_distribution<float> ndistribution_vy_;

  mppi::models::OptimizerSettings settings_;
  bool is_holonomic_;

  std::thread noise_thread_;
  std::condition_variable noise_cond_;
  std::mutex noise_lock_;
  bool active_{ false }, ready_{ false }, regenerate_noises_{ false };
};

}  // namespace rmp::controller::mppi

#endif  // RMP_CONTROLLER_MPPI_NOISE_GENERATOR_HPP_