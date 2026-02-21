/**
 * *********************************************************
 *
 * @file: noise_generator.cpp
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

#include "controller/tools/noise_generator.hpp"

#include <memory>
#include <mutex>

#include "system_config/controller_protos/mppi_controller.pb.h"

namespace rmp::controller::mppi {

void NoiseGenerator::initialize(mppi::models::OptimizerSettings& settings,
                                bool is_holonomic, const std::string& name,
                                pb::controller::MPPIController* mppi_config) {
  settings_ = settings;
  is_holonomic_ = is_holonomic;
  active_ = true;

  ndistribution_vx_ = std::normal_distribution(0.0f, settings_.sampling_std.vx);
  ndistribution_vy_ = std::normal_distribution(0.0f, settings_.sampling_std.vy);
  ndistribution_wz_ = std::normal_distribution(0.0f, settings_.sampling_std.wz);

  regenerate_noises_ = mppi_config->regenerate_noises();

  if (regenerate_noises_) {
    noise_thread_ = std::thread(std::bind(&NoiseGenerator::noiseThread, this));
  } else {
    generateNoisedControls();
  }
}

void NoiseGenerator::shutdown() {
  active_ = false;
  ready_ = true;
  noise_cond_.notify_all();
  if (noise_thread_.joinable()) {
    noise_thread_.join();
  }
}

void NoiseGenerator::generateNextNoises() {
  // Trigger the thread to run in parallel to this iteration
  // to generate the next iteration's noises (if applicable).
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    ready_ = true;
  }
  noise_cond_.notify_all();
}

void NoiseGenerator::setNoisedControls(models::State& state,
                                       const models::ControlSequence& control_sequence) {
  std::unique_lock<std::mutex> guard(noise_lock_);

  state.cvx = noises_vx_.rowwise() + control_sequence.vx.transpose();
  state.cvy = noises_vy_.rowwise() + control_sequence.vy.transpose();
  state.cwz = noises_wz_.rowwise() + control_sequence.wz.transpose();
}

void NoiseGenerator::reset(mppi::models::OptimizerSettings& settings, bool is_holonomic) {
  settings_ = settings;
  is_holonomic_ = is_holonomic;

  // Recompute the noises on reset, initialization, and fallback
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    noises_vx_.setZero(settings_.batch_size, settings_.time_steps);
    noises_vy_.setZero(settings_.batch_size, settings_.time_steps);
    noises_wz_.setZero(settings_.batch_size, settings_.time_steps);
    ready_ = true;
  }

  if (regenerate_noises_) {
    noise_cond_.notify_all();
  } else {
    generateNoisedControls();
  }
}

void NoiseGenerator::noiseThread() {
  do {
    std::unique_lock<std::mutex> guard(noise_lock_);
    noise_cond_.wait(guard, [this]() { return ready_; });
    ready_ = false;
    generateNoisedControls();
  } while (active_);
}

void NoiseGenerator::generateNoisedControls() {
  auto& s = settings_;
  noises_vx_ = Eigen::ArrayXXf::NullaryExpr(
      s.batch_size, s.time_steps, [&]() { return ndistribution_vx_(generator_); });
  noises_wz_ = Eigen::ArrayXXf::NullaryExpr(
      s.batch_size, s.time_steps, [&]() { return ndistribution_wz_(generator_); });
  if (is_holonomic_) {
    noises_vy_ = Eigen::ArrayXXf::NullaryExpr(
        s.batch_size, s.time_steps, [&]() { return ndistribution_vy_(generator_); });
  }
}

}  // namespace rmp::controller::mppi