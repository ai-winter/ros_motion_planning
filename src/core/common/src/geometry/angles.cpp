/**
 * *********************************************************
 *
 * @file: angles.cpp
 * @brief: useful functions for angles calculation
 * @author: Yang Haodong
 * @date: 2025-5-24
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <cmath>

#include "common/geometry/angles.h"

namespace rmp
{
namespace common
{
namespace geometry
{
/**
 * Convert degrees to radians.
 * @param degrees Angle in degrees.
 * @return Angle in radians.
 */
double fromDegrees(const double degrees)
{
  return degrees * M_PI / 180.0;
}

/**
 * Convert radians to degrees.
 * @param radians Angle in radians.
 * @return Angle in degrees.
 */
double toDegrees(const double radians)
{
  return radians * 180.0 / M_PI;
}

/**
 * Normalize angle to the range [0, 2*pi).
 * @param angle Angle in radians.
 * @return Angle normalized to [0, 2*pi).
 */
double normalizeAnglePositive(double angle)
{
  angle = std::fmod(angle, 2.0 * M_PI);
  return angle < 0.0f ? angle + 2.0 * M_PI : angle;
}

/**
 * Normalize angle to the range (-pi, pi].
 * @param angle Angle in radians.
 * @return Angle normalized to (-pi, pi].
 */
double normalizeAngle(double angle)
{
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  return (angle <= 0.0) ? angle + M_PI : angle - M_PI;
}

/**
 * Compute the shortest angular distance between two angles.
 * @param from_angle Starting angle in radians.
 * @param to_angle Target angle in radians.
 * @return Shortest signed angular distance in radians.
 */
double shortestAngularDistance(const double from_angle, const double to_angle)
{
  return normalizeAngle(to_angle - from_angle);
}

/**
 * Compute the complement of the angle with respect to 2*pi.
 * @param angle Angle in radians.
 * @return Complement angle in radians.
 */
double angleComplement(const double angle)
{
  const double mod_angle = std::fmod(angle, 2.0 * M_PI);

  if (mod_angle < 0.0f)
  {
    return 2.0 * M_PI + mod_angle;
  }
  else if (mod_angle > 0.0f)
  {
    return -2.0 * M_PI + mod_angle;
  }
  return 2.0 * M_PI;
}

/**
 * @brief Computes a 2D rotation matrix for a given angle.
 * @param angle The rotation angle in radians.
 * @return A 2x2 Eigen::Matrix2d representing the rotation matrix.
 *         The matrix is computed as:
 *         [ cos(angle)  -sin(angle) ]
 *         [ sin(angle)   cos(angle) ]
 */
Eigen::Matrix2d getRotationMatrix(const double angle)
{
  Eigen::Matrix2d R;
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  R << c, -s, s, c;
  return R;
}

/**
 * @brief Calculate the smallest angular difference between two angles.
 * @param angle_1 The starting angle in radians.
 * @param angle_2 The ending angle in radians.
 * @return The smallest angular difference.
 */
double angleDifference(double angle_1, double angle_2)
{
  const double diff = std::abs(angle_1 - angle_2);
  return (diff <= M_PI) ? diff : 2.0 * M_PI - diff;
}

/**
 * @brief Calculate the angular difference between two angles with direction consideration.
 * @param angle_1 The starting angle in radians.
 * @param angle_2 The ending angle in radians.
 * @param left_turn true for left turn, false for right turn,
 * @return The angular difference in radians according to specified direction.
 */
double angleDifference(double angle_1, double angle_2, bool left_turn)
{
  const double diff = std::abs(angle_1 - angle_2);
  if (left_turn)
  {  // Left turn (counter-clockwise)
    return (angle_2 >= angle_1) ? diff : 2.0 * M_PI - diff;
  }
  else
  {  // Right turn (clockwise)
    return (angle_1 >= angle_2) ? diff : 2.0 * M_PI - diff;
  }
}

/**
 * @brief Creates equally spaced yaws between two angles following specified turn direction.
 * @param start_angle Starting angle in radians
 * @param end_angle Ending angle in radians
 * @param left_turn Turn direction (true=left, false=right)
 * @param steps Number of interpolation points to generate
 * @return Vector of interpolated angles in [0, 2π) range:
 *         - When left_turn=true: path follows counterclockwise direction
 *         - When left_turn=false: path follows clockwise direction
 */
void interpolateAngles(double start_angle, double end_angle, bool left_turn, int steps, std::vector<double>& result)
{
  result.clear();
  if (steps < 2)
  {
    result.push_back(normalizeAnglePositive(start_angle));
    return;
  }

  // Adjust end angle for direction
  if (left_turn)
  {
    if (start_angle > end_angle)
      end_angle += 2 * M_PI;
  }
  else
  {
    if (end_angle > start_angle)
      end_angle -= 2 * M_PI;
  }

  // Generate interpolated angles
  result.reserve(steps);
  double step = (end_angle - start_angle) / (steps - 1);

  for (int i = 0; i < steps; ++i)
  {
    result.push_back(normalizeAnglePositive(start_angle + i * step));
  }
}
}  // namespace geometry
}  // namespace common
}  // namespace rmp