/**
 * *********************************************************
 *
 * @file: dubins_curve.h
 * @brief: Dubins curve generation
 * @author: Yang Haodong
 * @date: 2023-12-23
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_COMMON_GEOMETRY_DUBINS_CURVE_H_
#define RMP_COMMON_GEOMETRY_DUBINS_CURVE_H_

#include <functional>

#include "common/geometry/curve.h"

namespace rmp
{
namespace common
{
namespace geometry
{
class DubinsCurve : public Curve
{
private:
  using DubinsMode = std::tuple<int, int, int>;
  using DubinsLength = std::tuple<double, double, double>;

public:
  /**
   * @brief Construct a new Dubins generation object
   * @param step        Simulation or interpolation size (default: 0.1)
   * @param max_curv    The maximum curvature of the curve (default: 0.25)
   */
  DubinsCurve();
  DubinsCurve(double step, double max_curv);

  /**
   * @brief Destroy the Dubins generation object
   */
  ~DubinsCurve();

  /**
   * @brief Running trajectory generation
   * @param points path points <x, y>
   * @param path generated trajectory
   * @return true if generate successfully, else failed
   */
  bool run(const Points2d points, Points2d& path);

  /**
   * @brief Running trajectory generation
   * @param points path points <x, y, theta>
   * @param path generated trajectory
   * @return true if generate successfully, else failed
   */
  bool run(const Points3d points, Points2d& path);

  /**
   * @brief Left-Straight-Left generation mode.
   * @param alpha   Initial pose of (0, 0, alpha)
   * @param beta    Goal pose of (dist, 0, beta)
   * @param dist    Goal pose of (dist, 0, beta)
   * @param length  Moving lenght of segments (t, s, p)
   * @param mode    Motion mode
   */
  void LSL(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode);

  /**
   * @brief Right-Straight-Right generation mode.
   * @param alpha   Initial pose of (0, 0, alpha)
   * @param beta    Goal pose of (dist, 0, beta)
   * @param dist    Goal pose of (dist, 0, beta)
   * @param length  Moving lenght of segments (t, s, p)
   * @param mode    Motion mode
   */
  void RSR(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode);

  /**
   * @brief Left-Straight-Right generation mode.
   * @param alpha   Initial pose of (0, 0, alpha)
   * @param beta    Goal pose of (dist, 0, beta)
   * @param dist    Goal pose of (dist, 0, beta)
   * @param length  Moving lenght of segments (t, s, p)
   * @param mode    Motion mode
   */
  void LSR(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode);

  /**
   * @brief Right-Straight-Left generation mode.
   * @param alpha   Initial pose of (0, 0, alpha)
   * @param beta    Goal pose of (dist, 0, beta)
   * @param dist    Goal pose of (dist, 0, beta)
   * @param length  Moving lenght of segments (t, s, p)
   * @param mode    Motion mode
   */
  void RSL(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode);

  /**
   * @brief Right-Left-Right generation mode.
   * @param alpha   Initial pose of (0, 0, alpha)
   * @param beta    Goal pose of (dist, 0, beta)
   * @param dist    Goal pose of (dist, 0, beta)
   * @param length  Moving lenght of segments (t, s, p)
   * @param mode    Motion mode
   */
  void RLR(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode);

  /**
   * @brief Left-Right-Left generation mode.
   * @param alpha   Initial pose of (0, 0, alpha)
   * @param beta    Goal pose of (dist, 0, beta)
   * @param dist    Goal pose of (dist, 0, beta)
   * @param length  Moving lenght of segments (t, s, p)
   * @param mode    Motion mode
   */
  void LRL(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode);

  /**
   * @brief Planning path interpolation.
   * @param mode      motion, i.e., DUBINS_L, DUBINS_S, DUBINS_R
   * @param length    Single step motion path length
   * @param init_pose Initial pose (x, y, yaw)
   * @return new_pose	New pose (new_x, new_y, new_yaw) after moving
   */
  Point3d interpolate(int mode, double length, Point3d init_pose);

  /**
   * @brief Generate the path.
   * @param start Initial pose (x, y, yaw)
   * @param goal  Target pose (x, y, yaw)
   * @return path The smoothed trajectory points
   */
  Points2d generation(Point3d start, Point3d goal);

  /**
   * @brief Configure the maximum curvature.
   * @param max_curv  the maximum curvature
   */
  void setMaxCurv(double max_curv);

protected:
  /**
   * @brief Update the best motion mode.
   * @param length      the current motion length
   * @param mode        the current motion mode
   * @param best_length the best motion length so far
   * @param best_mode   the best motion mode so far
   * @param best_cost   the best motion cost so far
   */
  void _update(DubinsLength length, DubinsMode mode, DubinsLength& best_length, DubinsMode& best_mode,
               double& best_cost);

protected:
  double max_curv_;  // The maximum curvature of the curve

private:
  using DubinsSolver = std::function<void(double, double, double, DubinsLength&, DubinsMode&)>;
  const std::array<DubinsSolver, 6> dubins_solvers = {
    std::bind(&DubinsCurve::LRL, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
              std::placeholders::_4, std::placeholders::_5),
    std::bind(&DubinsCurve::LSL, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
              std::placeholders::_4, std::placeholders::_5),
    std::bind(&DubinsCurve::LSR, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
              std::placeholders::_4, std::placeholders::_5),
    std::bind(&DubinsCurve::RLR, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
              std::placeholders::_4, std::placeholders::_5),
    std::bind(&DubinsCurve::RSL, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
              std::placeholders::_4, std::placeholders::_5),
    std::bind(&DubinsCurve::RSR, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
              std::placeholders::_4, std::placeholders::_5),
  };
};

}  // namespace geometry
}  // namespace common
}  // namespace rmp
#endif