/***********************************************************
 *
 * @file: math_helper.h
 * @breif: Contains common/commonly used math function
 * @author: Yang Haodong
 * @update: 2024-1-3
 * @version: 1.0
 *
 * Copyright (c) 2024， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef MATH_HELPER_H
#define MATH_HELPER_H

#include <Eigen/Dense>
#include "nodes.h"

namespace helper
{
/**
 * @brief Calculate distance between the 2 nodes.
 * @param n1 Node 1
 * @param n2 Node 2
 * @return distance between nodes
 */
double dist(const Node& node1, const Node& node2);
double dist(const std::pair<double, double>& node1, const std::pair<double, double>& node2);
double dist(const Eigen::Vector2d& node1, const Eigen::Vector2d& node2);

/**
 * @brief Calculate the angle of x-axis between the 2 nodes.
 * @param n1 Node 1
 * @param n2 Node 2
 * @return the angle of x-axis between the 2 node
 */
double angle(const Node& node1, const Node& node2);
double angle(const std::pair<double, double>& node1, const std::pair<double, double>& node2);

/**
 * @brief Perform modulus operation on 2π.
 * @param theta    the angle to modulu
 * @return theta_m the angle after modulus operator
 */
double mod2pi(double theta);

/**
 * @brief Truncate the angle to the interval of -π to π.
 * @param theta    the angle to truncate
 * @return theta_t the truncated angle
 */
double pi2pi(double theta);

/**
 * @brief Clamps a value within a specified range.
 * @tparam T             The type of the values to be clamped.
 * @param value          The value to be clamped.
 * @param low            The lower bound of the range.
 * @param high           The upper bound of the range.
 * @return const T&      The clamped value within the specified range.
 */
template <typename T>
const T& clamp(const T& value, const T& low, const T& high)
{
  return std::max(low, std::min(value, high));
}
}  // namespace helper

#endif