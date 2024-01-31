/**
 * *********************************************************
 *
 * @file: math_helper.h
 * @brief: Contains common/commonly used math function
 * @author: Yang Haodong
 * @date: 2024-01-03
 * @version: 1.2
 *
 * Copyright (c) 2024, Yang Haodong. 
 * All rights reserved.
 * 
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef MATH_HELPER_H
#define MATH_HELPER_H

// ROS headers
#include <geometry_msgs/PoseStamped.h>

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
double dist(const geometry_msgs::PoseStamped& node1, const geometry_msgs::PoseStamped& node2);

/**
 * @brief Calculate the angle of x-axis between the 2 nodes.
 * @param n1 Node 1
 * @param n2 Node 2
 * @return the angle of x-axis between the 2 node
 */
double angle(const Node& node1, const Node& node2);
double angle(const std::pair<double, double>& node1, const std::pair<double, double>& node2);
double angle(const geometry_msgs::PoseStamped& node1, const geometry_msgs::PoseStamped& node2);

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
 * @brief Formula for intersection of a line with a circle centered at the origin
 * @note  https://mathworld.wolfram.com/Circle-LineIntersection.html
 * @param p1/p2     the two point in the segment
 * @param r         the radius of circle centered at the origin
 * @return points   the intersection points of a line and the circle
 */
std::vector<std::pair<double, double>> circleSegmentIntersection(const std::pair<double, double>& p1,
                                                                 const std::pair<double, double>& p2, double r);

/**
 * @brief Clamps a value within a specified range.
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

/**
 * @brief Find the first element in iterator with the minimum calculated value
 * @param begin   The begin of iterator
 * @param end     The end of iterator
 * @param cal     The customer calculated function
 * @return it     The first element in iterator with the minimum calculated value
 */
template <typename Iter, typename Func>
Iter getMinFuncVal(Iter begin, Iter end, Func cal)
{
  if (begin == end)
    return end;

  auto min_val = cal(*begin);
  Iter min_iter = begin;
  for (Iter it = ++begin; it != end; it++)
  {
    auto val = cal(*it);
    if (val <= min_val)
    {
      min_val = val;
      min_iter = it;
    }
  }
  return min_iter;
}

/**
 * @brief Find the first element in iterator that is greater integrated distance than compared value
 * @param begin   The begin of iterator
 * @param end     The end of iterator
 * @param dist    The distance metric function
 * @param cmp_val The compared value
 * @return it     The first element in iterator that is greater integrated distance than compared value
 */
template <typename Iter, typename Func, typename Val>
Iter firstIntegratedDistance(Iter begin, Iter end, Func dist, Val cmp_val)
{
  if (begin == end)
    return end;
  Val d = 0.0;
  for (Iter it = begin; it != end - 1; it++)
  {
    d += dist(*it, *(it + 1));
    if (d > cmp_val)
      return it + 1;
  }
  return end;
}

}  // namespace helper

#endif