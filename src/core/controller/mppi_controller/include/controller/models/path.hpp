/**
 * *********************************************************
 *
 * @file: path.hpp
 * @brief: Path definition for models in MPPI controller.
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

#ifndef RMP_CONTROLLER_MPPI_PATH_HPP_
#define RMP_CONTROLLER_MPPI_PATH_HPP_

#include <Eigen/Dense>

namespace rmp::controller::mppi {

/**
 * @struct rmp::controller::mppi::Path
 * @brief Path represented as Eigen Array
 */
struct Path {
  Eigen::ArrayXf x;
  Eigen::ArrayXf y;
  Eigen::ArrayXf yaws;

  /**
   * @brief Reset path data
   */
  void reset(unsigned int size) {
    x.setZero(size);
    y.setZero(size);
    yaws.setZero(size);
  }
};

}  // namespace rmp::controller::mppi

#endif  // RMP_CONTROLLER_MPPI_PATH_HPP_