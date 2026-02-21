/**
 * *********************************************************
 *
 * @file: motion_models.hpp
 * @brief: Motion models used in MPPI controller.
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

#ifndef RMP_CONTROLLER_MPPI_MOTION_MODELS_HPP_
#define RMP_CONTROLLER_MPPI_MOTION_MODELS_HPP_

#include <Eigen/Dense>

#include "controller/models/constraints.hpp"
#include "controller/models/control_sequence.hpp"
#include "controller/models/state.hpp"

#include "system_config/controller_protos/mppi_controller.pb.h"

namespace rmp::controller::mppi {

namespace utils {
float clamp(const float lower_bound, const float upper_bound, const float input);
}

/**
 * @class rmp::controller::mppi::MotionModel
 * @brief Abstract motion model for modeling a vehicle
 */
class MotionModel {
public:
  /**
   * @brief Constructor for rmp::controller::mppi::MotionModel
   */
  MotionModel() = default;

  /**
   * @brief Destructor for rmp::controller::mppi::MotionModel
   */
  virtual ~MotionModel() = default;

  /**
   * @brief Initialize motion model on bringup and set required variables
   * @param control_constraints Constraints on control
   * @param model_dt duration of a time step
   */
  void initialize(const models::ControlConstraints& control_constraints, float model_dt) {
    control_constraints_ = control_constraints;
    model_dt_ = model_dt;
  }

  /**
   * @brief With input velocities, find the vehicle's output velocities
   * @param state Contains control velocities to use to populate vehicle velocities
   */
  virtual void predict(models::State& state) {
    const bool is_holo = isHolonomic();
    float max_delta_vx = model_dt_ * control_constraints_.ax_max;
    float min_delta_vx = model_dt_ * control_constraints_.ax_min;
    float max_delta_vy = model_dt_ * control_constraints_.ay_max;
    float min_delta_vy = model_dt_ * control_constraints_.ay_min;
    float max_delta_wz = model_dt_ * control_constraints_.az_max;

    unsigned int n_cols = state.vx.cols();

    for (unsigned int i = 1; i < n_cols; i++) {
      auto lower_bound_vx = (state.vx.col(i - 1) > 0)
                                .select(state.vx.col(i - 1) + min_delta_vx,
                                        state.vx.col(i - 1) - max_delta_vx);
      auto upper_bound_vx = (state.vx.col(i - 1) > 0)
                                .select(state.vx.col(i - 1) + max_delta_vx,
                                        state.vx.col(i - 1) - min_delta_vx);

      state.cvx.col(i - 1) =
          state.cvx.col(i - 1).cwiseMax(lower_bound_vx).cwiseMin(upper_bound_vx);
      state.vx.col(i) = state.cvx.col(i - 1);

      state.cwz.col(i - 1) = state.cwz.col(i - 1)
                                 .cwiseMax(state.wz.col(i - 1) - max_delta_wz)
                                 .cwiseMin(state.wz.col(i - 1) + max_delta_wz);
      state.wz.col(i) = state.cwz.col(i - 1);

      if (is_holo) {
        auto lower_bound_vy = (state.vy.col(i - 1) > 0)
                                  .select(state.vy.col(i - 1) + min_delta_vy,
                                          state.vy.col(i - 1) - max_delta_vy);
        auto upper_bound_vy = (state.vy.col(i - 1) > 0)
                                  .select(state.vy.col(i - 1) + max_delta_vy,
                                          state.vy.col(i - 1) - min_delta_vy);
        state.cvy.col(i - 1) =
            state.cvy.col(i - 1).cwiseMax(lower_bound_vy).cwiseMin(upper_bound_vy);
        state.vy.col(i) = state.cvy.col(i - 1);
      }
    }
  }

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  virtual bool isHolonomic() = 0;

  /**
   * @brief Apply hard vehicle constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   */
  virtual void applyConstraints(models::ControlSequence& /*control_sequence*/) {
  }

protected:
  float model_dt_{ 0.0 };
  models::ControlConstraints control_constraints_{ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                                   0.0f, 0.0f, 0.0f, 0.0f };
};

/**
 * @class rmp::controller::mppi::AckermannMotionModel
 * @brief Ackermann motion model
 */
class AckermannMotionModel : public MotionModel {
public:
  /**
   * @brief Constructor for rmp::controller::mppi::AckermannMotionModel
   */
  explicit AckermannMotionModel(pb::controller::MPPIController* mppi_config,
                                const std::string& name) {
    min_turning_r_ = mppi_config->min_turning_r();
  }

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override {
    return false;
  }

  /**
   * @brief Apply hard vehicle constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   */
  void applyConstraints(models::ControlSequence& control_sequence) override {
    const auto wz_constrained = control_sequence.vx.abs() / min_turning_r_;
    control_sequence.wz = control_sequence.wz.max((-wz_constrained)).min(wz_constrained);
  }

  /**
   * @brief Get minimum turning radius of ackermann drive
   * @return Minimum turning radius
   */
  float getMinTurningRadius() {
    return min_turning_r_;
  }

private:
  float min_turning_r_{ 0.0f };
};

/**
 * @class rmp::controller::mppi::DiffDriveMotionModel
 * @brief Differential drive motion model
 */
class DiffDriveMotionModel : public MotionModel {
public:
  /**
   * @brief Constructor for rmp::controller::mppi::DiffDriveMotionModel
   */
  DiffDriveMotionModel() = default;

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override {
    return false;
  }
};

/**
 * @class rmp::controller::mppi::OmniMotionModel
 * @brief Omnidirectional motion model
 */
class OmniMotionModel : public MotionModel {
public:
  /**
   * @brief Constructor for rmp::controller::mppi::OmniMotionModel
   */
  OmniMotionModel() = default;

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override {
    return true;
  }
};

}  // namespace rmp::controller::mppi

#endif  // RMP_CONTROLLER_MPPI_MOTION_MODELS_HPP_