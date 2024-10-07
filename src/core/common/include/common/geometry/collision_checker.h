/***********************************************************
 *
 * @file: collision_checker.h
 * @breif: Collision Checker
 * @author: Yang Haodong
 * @update: 2024-10-07
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef RMP_COMMON_GEOMETRY_COLLISION_CHECKER_H_
#define RMP_COMMON_GEOMETRY_COLLISION_CHECKER_H_

#include <cmath>

namespace rmp
{
namespace common
{
namespace geometry
{
class CollisionChecker
{
public:
  CollisionChecker() = default;
  ~CollisionChecker() = default;

public:
  /**
   * @brief 1d-collision checker Bresenham: to check if there is any obstacle between pt1 and pt2
   * @param pt1 point1
   * @param pt2 point2
   * @param func_is_obs the function to judge whether the position (x, y) is obstacle
   * @return true if collision occurs, else false
   */
  template <typename Point, typename F_is_obs>
  static bool BresenhamCollisionDetection(const Point& pt1, const Point& pt2, F_is_obs func_is_obs)
  {
    int s_x = (pt1.x() - pt2.x() == 0) ? 0 : (pt1.x() - pt2.x()) / std::abs(pt1.x() - pt2.x());
    int s_y = (pt1.y() - pt2.y() == 0) ? 0 : (pt1.y() - pt2.y()) / std::abs(pt1.y() - pt2.y());
    int d_x = std::abs(pt1.x() - pt2.x());
    int d_y = std::abs(pt1.y() - pt2.y());

    // check if any obstacle exists between pt1 and pt2
    if (d_x > d_y)
    {
      int tau = d_y - d_x;
      int x = pt2.x(), y = pt2.y();
      int e = 0;
      while (x != pt1.x())
      {
        if (e * 2 > tau)
        {
          x += s_x;
          e -= d_y;
        }
        else if (e * 2 < tau)
        {
          y += s_y;
          e += d_x;
        }
        else
        {
          x += s_x;
          y += s_y;
          e += d_x - d_y;
        }
        if (func_is_obs(Point(x, y)))
          // obstacle detected
          return true;
      }
    }
    else
    {
      // similar. swap x and y
      int tau = d_x - d_y;
      int x = pt2.x(), y = pt2.y();
      int e = 0;
      while (y != pt1.y())
      {
        if (e * 2 > tau)
        {
          y += s_y;
          e -= d_x;
        }
        else if (e * 2 < tau)
        {
          x += s_x;
          e += d_y;
        }
        else
        {
          x += s_x;
          y += s_y;
          e += d_y - d_x;
        }
        if (func_is_obs(Point(x, y)))
          // obstacle detected
          return true;
      }
    }
    return false;
  }
};
}  // namespace geometry
}  // namespace common
}  // namespace rmp

#endif