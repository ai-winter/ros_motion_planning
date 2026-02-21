/**
 * *********************************************************
 *
 * @file: constraint_critic.cpp
 * @brief: Constraint critic used in MPPI controller.
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

#include "controller/critics/constraint_critic.hpp"

namespace rmp::controller::mppi::critics {

void ConstraintCritic::initialize() {
  enabled_ = mppi_config_->constraint_critic_enabled();
  power_ = mppi_config_->constraint_critic_cost_power();
  weight_ = mppi_config_->constraint_critic_cost_weight();

  float vx_max, vy_max, vx_min;
  vx_max = mppi_config_->constraint_critic_vx_max();
  vy_max = mppi_config_->constraint_critic_vy_max();
  vx_min = mppi_config_->constraint_critic_vx_min();

  const float min_sgn = vx_min > 0.0f ? 1.0f : -1.0f;
  max_vel_ = sqrtf(vx_max * vx_max + vy_max * vy_max);
  min_vel_ = min_sgn * sqrtf(vx_min * vx_min + vy_max * vy_max);
}

void ConstraintCritic::score(CriticData& data) {
  if (!enabled_) {
    return;
  }

  // Differential motion model
  auto diff = dynamic_cast<DiffDriveMotionModel*>(data.motion_model.get());
  if (diff != nullptr) {
    if (power_ > 1u) {
      data.costs += (((((data.state.vx - max_vel_).max(0.0f) +
                        (min_vel_ - data.state.vx).max(0.0f)) *
                       data.model_dt)
                          .rowwise()
                          .sum()
                          .eval()) *
                     weight_)
                        .pow(power_)
                        .eval();
    } else {
      data.costs += (((((data.state.vx - max_vel_).max(0.0f) +
                        (min_vel_ - data.state.vx).max(0.0f)) *
                       data.model_dt)
                          .rowwise()
                          .sum()
                          .eval()) *
                     weight_)
                        .eval();
    }
    return;
  }

  // Omnidirectional motion model
  auto omni = dynamic_cast<OmniMotionModel*>(data.motion_model.get());
  if (omni != nullptr) {
    auto& vx = data.state.vx;
    unsigned int n_rows = data.state.vx.rows();
    unsigned int n_cols = data.state.vx.cols();
    Eigen::ArrayXXf sgn(n_rows, n_cols);
    sgn = vx.unaryExpr([](const float x) { return copysignf(1.0f, x); });

    auto vel_total = sgn * (data.state.vx.square() + data.state.vy.square()).sqrt();
    if (power_ > 1u) {
      data.costs +=
          ((((vel_total - max_vel_).max(0.0f) + (min_vel_ - vel_total).max(0.0f)) *
            data.model_dt)
               .rowwise()
               .sum()
               .eval() *
           weight_)
              .pow(power_)
              .eval();
    } else {
      data.costs +=
          ((((vel_total - max_vel_).max(0.0f) + (min_vel_ - vel_total).max(0.0f)) *
            data.model_dt)
               .rowwise()
               .sum()
               .eval() *
           weight_)
              .eval();
    }
    return;
  }

  // Ackermann motion model
  auto acker = dynamic_cast<AckermannMotionModel*>(data.motion_model.get());
  if (acker != nullptr) {
    auto& vx = data.state.vx;
    auto& wz = data.state.wz;
    const float min_turning_rad = acker->getMinTurningRadius();

    const float epsilon = 1e-6f;
    auto wz_safe =
        wz.abs().max(epsilon);  // Replace small wz values to avoid division by 0
    auto out_of_turning_rad_motion = (min_turning_rad - (vx.abs() / wz_safe)).max(0.0f);

    if (power_ > 1u) {
      data.costs += ((((vx - max_vel_).max(0.0f) + (min_vel_ - vx).max(0.0f) +
                       out_of_turning_rad_motion) *
                      data.model_dt)
                         .rowwise()
                         .sum()
                         .eval() *
                     weight_)
                        .pow(power_)
                        .eval();
    } else {
      data.costs += ((((vx - max_vel_).max(0.0f) + (min_vel_ - vx).max(0.0f) +
                       out_of_turning_rad_motion) *
                      data.model_dt)
                         .rowwise()
                         .sum()
                         .eval() *
                     weight_)
                        .eval();
    }
    return;
  }
}

}  // namespace rmp::controller::mppi::critics