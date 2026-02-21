/**
 * *********************************************************
 *
 * @file: constraint_critic.hpp
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

#ifndef RMP_CONTROLLER_MPPI_CONSTRAINT_CRITIC_HPP_
#define RMP_CONTROLLER_MPPI_CONSTRAINT_CRITIC_HPP_

#include "controller/critic_function.hpp"
#include "controller/models/state.hpp"
#include "controller/tools/utils.hpp"

namespace rmp::controller::mppi::critics {

/**
 * @class rmp::controller::mppi::critics::ConstraintCritic
 * @brief Critic objective function for enforcing feasible constraints
 */
class ConstraintCritic : public CriticFunction {
public:
  /**
   * @brief Initialize critic
   */
  void initialize() override;

  /**
   * @brief Evaluate cost related to goal following
   *
   * @param costs [out] add reference cost values to this tensor
   */
  void score(CriticData& data) override;

  float getMaxVelConstraint() {
    return max_vel_;
  }
  float getMinVelConstraint() {
    return min_vel_;
  }

protected:
  unsigned int power_{ 0 };
  float weight_{ 0.0f };
  float min_vel_;
  float max_vel_;
};

}  // namespace rmp::controller::mppi::critics

#endif  // RMP_CONTROLLER_MPPI_CONSTRAINT_CRITIC_HPP_