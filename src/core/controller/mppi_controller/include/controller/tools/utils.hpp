/**
 * *********************************************************
 *
 * @file: utils.hpp
 * @brief: Utility functions for MPPI controller.
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

#ifndef RMP_CONTROLLER_MPPI_UTILS_HPP_
#define RMP_CONTROLLER_MPPI_UTILS_HPP_

#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <string>
#include <limits>
#include <memory>
#include <vector>

#define M_PIF 3.141592653589793238462643383279502884e+00F
#define M_PIF_2 1.5707963267948966e+00F

namespace rmp::controller::mppi::utils {

/**
 * @brief Clamps the input between the given lower and upper bounds.
 * @param lower_bound Lower bound.
 * @param upper_bound Upper bound.
 * @return Clamped output.
 */
inline float clamp(const float lower_bound, const float upper_bound, const float input) {
  return std::min(upper_bound, std::max(input, lower_bound));
}

}  // namespace rmp::controller::mppi::utils

#endif  // RMP_CONTROLLER_MPPI_UTILS_HPP_