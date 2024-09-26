/**
 * *********************************************************
 *
 * @file: math_helper.h
 * @brief: math helper function
 * @author: Yang Haodong
 * @date: 2024-9-6
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */

#ifndef RMP_COMMON_MATH_MATH_HELPER_H_
#define RMP_COMMON_MATH_MATH_HELPER_H_
#pragma once

#include <cmath>
#include <limits>
#include <vector>

#include "common/geometry/vec2d.h"

namespace rmp
{
namespace common
{
namespace math
{
constexpr double kMathEpsilon = 1e-10;

template <typename T>
T abs(T val)
{
  return val < 0 ? -val : val;
}

template <typename T>
bool less(T query, T target)
{
  return query < target && abs(query - target) > kMathEpsilon ? true : false;
}

template <typename T>
bool large(T query, T target)
{
  return query > target && abs(query - target) > kMathEpsilon ? true : false;
}

template <typename T>
bool equal(T query, T target)
{
  return abs(query - target) <= kMathEpsilon ? true : false;
}

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
 * @brief Compute squared value.
 * @param value The target value to get its squared value.
 * @return Squared value of the input value.
 */
template <typename T>
inline T Square(const T value)
{
  return value * value;
}

/**
 * @brief Clamp a value between two bounds.
 *        If the value goes beyond the bounds, return one of the bounds,
 *        otherwise, return the original value.
 * @param value The original value to be clamped.
 * @param bound1 One bound to clamp the value.
 * @param bound2 The other bound to clamp the value.
 * @return The clamped value.
 */
template <typename T>
T clamp(const T value, T bound1, T bound2)
{
  if (bound1 > bound2)
  {
    std::swap(bound1, bound2);
  }

  if (value < bound1)
  {
    return bound1;
  }
  else if (value > bound2)
  {
    return bound2;
  }
  return value;
}

/**
 * @brief Cross product between two 2-D vectors from the common start point,
 *        and end at two other points.
 * @param start_point The common start point of two vectors in 2-D.
 * @param end_point_1 The end point of the first vector.
 * @param end_point_2 The end point of the second vector.
 *
 * @return The cross product result.
 */
double crossProd(const rmp::common::geometry::Vec2d& start_point, const rmp::common::geometry::Vec2d& end_point_1,
                 const rmp::common::geometry::Vec2d& end_point_2);

/**
 * @brief Inner product between two 2-D vectors from the common start point,
 *        and end at two other points.
 * @param start_point The common start point of two vectors in 2-D.
 * @param end_point_1 The end point of the first vector.
 * @param end_point_2 The end point of the second vector.
 *
 * @return The inner product result.
 */
double innerProd(const rmp::common::geometry::Vec2d& start_point, const rmp::common::geometry::Vec2d& end_point_1,
                 const rmp::common::geometry::Vec2d& end_point_2);

/**
 * @brief Cross product between two vectors.
 *        One vector is formed by 1st and 2nd parameters of the function.
 *        The other vector is formed by 3rd and 4th parameters of the function.
 * @param x0 The x coordinate of the first vector.
 * @param y0 The y coordinate of the first vector.
 * @param x1 The x coordinate of the second vector.
 * @param y1 The y coordinate of the second vector.
 *
 * @return The cross product result.
 */
double crossProd(const double x0, const double y0, const double x1, const double y1);

/**
 * @brief Inner product between two vectors.
 *        One vector is formed by 1st and 2nd parameters of the function.
 *        The other vector is formed by 3rd and 4th parameters of the function.
 * @param x0 The x coordinate of the first vector.
 * @param y0 The y coordinate of the first vector.
 * @param x1 The x coordinate of the second vector.
 * @param y1 The y coordinate of the second vector.
 *
 * @return The inner product result.
 */
double innerProd(const double x0, const double y0, const double x1, const double y1);

/**
 * @brief Formula for intersection of a line with a circle centered at the origin
 * @note  https://mathworld.wolfram.com/Circle-LineIntersection.html
 * @param p1/p2     the two point in the segment
 * @param r         the radius of circle centered at the origin
 * @return points   the intersection points of a line and the circle
 */
std::vector<std::pair<double, double>> circleSegmentIntersection(const std::pair<double, double>& p1,
                                                                 const std::pair<double, double>& p2, double r);

}  // namespace math
}  // namespace common
}  // namespace rmp
#endif