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

#include <cmath>
#include <cstdint>
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

/**
 * @brief factorial calculation
 * @param n number
 * @return n!
 */
constexpr uint64_t factorial(uint64_t n) noexcept
{
  return (n <= 1) ? 1 : (n * factorial(n - 1));
}

/**
 * @brief absolute value calculation
 * @param val value
 * @return |val|
 */
template <typename T>
T abs(T val)
{
  return val < 0 ? -val : val;
}

/**
 * @brief Determine whether the query is less than the target within the epsilon
 * @param query value
 * @param target value
 * @return query < target
 */
template <typename T>
bool less(T query, T target)
{
  return query < target && abs(query - target) > kMathEpsilon ? true : false;
}

/**
 * @brief Determine whether the query is higher than the target within the epsilon
 * @param query value
 * @param target value
 * @return query > target
 */
template <typename T>
bool large(T query, T target)
{
  return query > target && abs(query - target) > kMathEpsilon ? true : false;
}

/**
 * @brief Determine whether the query is euqal to the target within the epsilon
 * @param query value
 * @param target value
 * @return query = target
 */
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
std::vector<rmp::common::geometry::Vec2d> circleSegmentIntersection(const rmp::common::geometry::Vec2d& p1,
                                                                    const rmp::common::geometry::Vec2d& p2, double r);

/**
 * @brief Center of an arc between three points
 * @note http://paulbourke.net/geometry/circlesphere/
 * @note https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Intersection%20of%20two%20lines
 * @param pt_prev Starting point of the arc
 * @param pt Mid point of the arc
 * @param pt_next Last point of the arc
 * @param is_cusp True if pt is a cusp point
 * @result position of the center or Vector2(inf, inf) for straight lines and 180 deg turns
 */
double arcCenter(const rmp::common::geometry::Vec2d& pt_prev, const rmp::common::geometry::Vec2d& pt,
                 const rmp::common::geometry::Vec2d& pt_next, bool is_cusp,
                 rmp::common::geometry::Vec2d* center = nullptr);

/**
 * @brief Direction of a line which contains pt and is tangential to arc
 * @param pt_prev Starting point of the arc
 * @param pt Mid point of the arc
 * @param pt_next Last point of the arc
 * @param is_cusp True if pt is a cusp point
 * @result Tangential line direction.
 * @note the sign of tangentDir is undefined here, should be assigned in post-process depending
 *       on movement direction. Also, for speed reasons, direction vector is not normalized.
 */
rmp::common::geometry::Vec2d tangentDir(const rmp::common::geometry::Vec2d& pt_prev,
                                        const rmp::common::geometry::Vec2d& pt,
                                        const rmp::common::geometry::Vec2d& pt_next, bool is_cusp);

/**
 * @brief Sort a set of points in counterclockwise order
 * @param points a set of points to sort
 */
void sortPoints(std::vector<rmp::common::geometry::Vec2d>& points);

}  // namespace math
}  // namespace common
}  // namespace rmp
#endif