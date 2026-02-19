/**
 * *********************************************************
 *
 * @file: trajectories.hpp
 * @brief: Trajectories definition for models in MPPI controller.
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

#ifndef RMP_CONTROLLER_MPPI_TRAJECTORIES_HPP_
#define RMP_CONTROLLER_MPPI_TRAJECTORIES_HPP_

#include <Eigen/Dense>

namespace rmp::controller::mppi {
/**
 * @class rmp::controller::mppi::Trajectories
 * @brief Candidate Trajectories
 */
struct Trajectories {
  Eigen::ArrayXXf x;
  Eigen::ArrayXXf y;
  Eigen::ArrayXXf yaws;

  /**
   * @brief Reset state data
   */
  void reset(unsigned int batch_size, unsigned int time_steps) {
    x.setZero(batch_size, time_steps);
    y.setZero(batch_size, time_steps);
    yaws.setZero(batch_size, time_steps);
  }
};

}  // namespace rmp::controller::mppi

#endif  // RMP_CONTROLLER_MPPI_TRAJECTORIES_HPP_