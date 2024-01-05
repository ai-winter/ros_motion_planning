/***********************************************************
 *
 * @file: math_helper.cpp
 * @breif: Contains common/commonly used function
 * @author: Yang Haodong
 * @update: 2024-1-3
 * @version: 1.0
 *
 * Copyright (c) 2024， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include "math_helper.h"

namespace helper
{
/**
 * @brief Calculate distance between the 2 nodes.
 * @param n1 Node 1
 * @param n2 Node 2
 * @return distance between nodes
 */
double dist(const Node& node1, const Node& node2)
{
  return std::hypot(node1.x_ - node2.x_, node1.y_ - node2.y_);
}

double dist(const std::pair<double, double>& node1, const std::pair<double, double>& node2)
{
  return std::hypot(node1.first - node2.first, node1.second - node2.second);
}

double dist(const Eigen::Vector2d& point1, const Eigen::Vector2d& point2)
{
  return std::hypot(point1.x() - point2.x(), point1.y() - point2.y());
}

/**
 * @brief Calculate the angle of x-axis between the 2 nodes.
 * @param n1 Node 1
 * @param n2 Node 2
 * @return the angle of x-axis between the 2 node
 */
double angle(const Node& node1, const Node& node2)
{
  return atan2(node2.y_ - node1.y_, node2.x_ - node1.x_);
}

double angle(const std::pair<double, double>& node1, const std::pair<double, double>& node2)
{
  return atan2(node2.second - node1.second, node2.first - node1.first);
}

/**
 * @brief Perform modulus operation on 2π.
 * @param theta    the angle to modulu
 * @return theta_m the angle after modulus operator
 */
double mod2pi(double theta)
{
  return theta - 2.0 * M_PI * floor(theta / M_PI / 2.0);
}

/**
 * @brief Truncate the angle to the interval of -π to π.
 * @param theta    the angle to truncate
 * @return theta_t the truncated angle
 */
double pi2pi(double theta)
{
  while (theta > M_PI)
    theta -= 2.0 * M_PI;
  while (theta < -M_PI)
    theta += 2.0 * M_PI;
  return theta;
}
}  // namespace helper