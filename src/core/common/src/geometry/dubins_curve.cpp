/**
 * *********************************************************
 *
 * @file: dubins_curve.cpp
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
#include <Eigen/Dense>
#include <climits>
#include <cassert>
#include <iostream>

#include "common/math/math_helper.h"
#include "common/geometry/dubins_curve.h"

namespace rmp
{
namespace common
{
namespace geometry
{
namespace
{
#define UNPACK_DUBINS_INPUTS(alpha, beta)                                                                              \
  double sin_a = sin(alpha);                                                                                           \
  double sin_b = sin(beta);                                                                                            \
  double cos_a = cos(alpha);                                                                                           \
  double cos_b = cos(beta);                                                                                            \
  double cos_a_b = cos(alpha - beta);
}  // namespace

enum
{
  DUBINS_NONE = -1,
  DUBINS_L = 0,
  DUBINS_S = 1,
  DUBINS_R = 2,
};

/**
 * @brief Construct a new Dubins generation object
 * @param step        Simulation or interpolation size (default: 0.1)
 * @param max_curv    The maximum curvature of the curve (default: 0.25)
 */
DubinsCurve::DubinsCurve(double step, double max_curv) : Curve(step), max_curv_(max_curv)
{
}
DubinsCurve::DubinsCurve() : Curve(0.1), max_curv_(0.25)
{
}

/**
 * @brief Destroy the Dubins generation object
 */
DubinsCurve::~DubinsCurve()
{
}

/**
 * @brief Left-Straight-Left generation mode.
 * @param alpha   Initial pose of (0, 0, alpha)
 * @param beta    Goal pose of (dist, 0, beta)
 * @param dist    Goal pose of (dist, 0, beta)
 * @param length  Moving lenght of segments (t, s, p)
 * @param mode    Motion mode
 */
void DubinsCurve::LSL(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode)
{
  UNPACK_DUBINS_INPUTS(alpha, beta);
  double p_lsl = 2 + std::pow(dist, 2) - 2 * cos_a_b + 2 * dist * (sin_a - sin_b);

  if (p_lsl < 0)
  {
    length = { DUBINS_NONE, DUBINS_NONE, DUBINS_NONE };
    mode = { DUBINS_L, DUBINS_S, DUBINS_L };
  }
  else
  {
    p_lsl = sqrt(p_lsl);
    double t_lsl = rmp::common::math::mod2pi(-alpha + atan2(cos_b - cos_a, dist + sin_a - sin_b));
    double q_lsl = rmp::common::math::mod2pi(beta - atan2(cos_b - cos_a, dist + sin_a - sin_b));
    length = { t_lsl, p_lsl, q_lsl };
    mode = { DUBINS_L, DUBINS_S, DUBINS_L };
  }
}

/**
 * @brief Right-Straight-Right generation mode.
 * @param alpha   Initial pose of (0, 0, alpha)
 * @param beta    Goal pose of (dist, 0, beta)
 * @param dist    Goal pose of (dist, 0, beta)
 * @param length  Moving lenght of segments (t, s, p)
 * @param mode    Motion mode
 */
void DubinsCurve::RSR(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode)
{
  UNPACK_DUBINS_INPUTS(alpha, beta);
  double p_rsr = 2 + std::pow(dist, 2) - 2 * cos_a_b + 2 * dist * (sin_b - sin_a);
  if (p_rsr < 0)
  {
    length = { DUBINS_NONE, DUBINS_NONE, DUBINS_NONE };
    mode = { DUBINS_R, DUBINS_S, DUBINS_R };
  }
  else
  {
    p_rsr = sqrt(p_rsr);
    double t_rsr = rmp::common::math::mod2pi(alpha - atan2(cos_a - cos_b, dist - sin_a + sin_b));
    double q_rsr = rmp::common::math::mod2pi(-beta + atan2(cos_a - cos_b, dist - sin_a + sin_b));
    length = { t_rsr, p_rsr, q_rsr };
    mode = { DUBINS_R, DUBINS_S, DUBINS_R };
  }
}

/**
 * @brief Left-Straight-Right generation mode.
 * @param alpha   Initial pose of (0, 0, alpha)
 * @param beta    Goal pose of (dist, 0, beta)
 * @param dist    Goal pose of (dist, 0, beta)
 * @param length  Moving lenght of segments (t, s, p)
 * @param mode    Motion mode
 */
void DubinsCurve::LSR(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode)
{
  UNPACK_DUBINS_INPUTS(alpha, beta);
  double p_lsr = -2 + std::pow(dist, 2) + 2 * cos_a_b + 2 * dist * (sin_a + sin_b);

  if (p_lsr < 0)
  {
    length = { DUBINS_NONE, DUBINS_NONE, DUBINS_NONE };
    mode = { DUBINS_L, DUBINS_S, DUBINS_R };
  }
  else
  {
    p_lsr = sqrt(p_lsr);
    double t_lsr = rmp::common::math::mod2pi(-alpha + atan2(-cos_a - cos_b, dist + sin_a + sin_b) - atan2(-2.0, p_lsr));
    double q_lsr = rmp::common::math::mod2pi(-beta + atan2(-cos_a - cos_b, dist + sin_a + sin_b) - atan2(-2.0, p_lsr));
    length = { t_lsr, p_lsr, q_lsr };
    mode = { DUBINS_L, DUBINS_S, DUBINS_R };
  }
}

/**
 * @brief Right-Straight-Left generation mode.
 * @param alpha   Initial pose of (0, 0, alpha)
 * @param beta    Goal pose of (dist, 0, beta)
 * @param dist    Goal pose of (dist, 0, beta)
 * @param length  Moving lenght of segments (t, s, p)
 * @param mode    Motion mode
 */
void DubinsCurve::RSL(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode)
{
  UNPACK_DUBINS_INPUTS(alpha, beta);
  double p_rsl = -2 + std::pow(dist, 2) + 2 * cos_a_b - 2 * dist * (sin_a + sin_b);

  if (p_rsl < 0)
  {
    length = { DUBINS_NONE, DUBINS_NONE, DUBINS_NONE };
    mode = { DUBINS_R, DUBINS_S, DUBINS_L };
  }
  else
  {
    p_rsl = sqrt(p_rsl);
    double t_rsl = rmp::common::math::mod2pi(alpha - atan2(cos_a + cos_b, dist - sin_a - sin_b) + atan2(2.0, p_rsl));
    double q_rsl = rmp::common::math::mod2pi(beta - atan2(cos_a + cos_b, dist - sin_a - sin_b) + atan2(2.0, p_rsl));
    length = { t_rsl, p_rsl, q_rsl };
    mode = { DUBINS_R, DUBINS_S, DUBINS_L };
  }
}

/**
 * @brief Right-Left-Right generation mode.
 * @param alpha   Initial pose of (0, 0, alpha)
 * @param beta    Goal pose of (dist, 0, beta)
 * @param dist    Goal pose of (dist, 0, beta)
 * @param length  Moving lenght of segments (t, s, p)
 * @param mode    Motion mode
 */
void DubinsCurve::RLR(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode)
{
  UNPACK_DUBINS_INPUTS(alpha, beta);
  double p_rlr = (6.0 - std::pow(dist, 2) + 2.0 * cos_a_b + 2.0 * dist * (sin_a - sin_b)) / 8.0;

  if (fabs(p_rlr) > 1.0)
  {
    length = { DUBINS_NONE, DUBINS_NONE, DUBINS_NONE };
    mode = { DUBINS_R, DUBINS_L, DUBINS_R };
  }
  else
  {
    p_rlr = rmp::common::math::mod2pi(2 * M_PI - acos(p_rlr));
    double t_rlr = rmp::common::math::mod2pi(alpha - atan2(cos_a - cos_b, dist - sin_a + sin_b) + p_rlr / 2.0);
    double q_rlr = rmp::common::math::mod2pi(alpha - beta - t_rlr + p_rlr);
    length = { t_rlr, p_rlr, q_rlr };
    mode = { DUBINS_R, DUBINS_L, DUBINS_R };
  }
}

/**
 * @brief Left-Right-Left generation mode.
 * @param alpha   Initial pose of (0, 0, alpha)
 * @param beta    Goal pose of (dist, 0, beta)
 * @param dist    Goal pose of (dist, 0, beta)
 * @param length  Moving lenght of segments (t, s, p)
 * @param mode    Motion mode
 */
void DubinsCurve::LRL(double alpha, double beta, double dist, DubinsLength& length, DubinsMode& mode)
{
  UNPACK_DUBINS_INPUTS(alpha, beta);
  double p_lrl = (6.0 - std::pow(dist, 2) + 2.0 * cos_a_b + 2.0 * dist * (sin_a - sin_b)) / 8.0;

  if (fabs(p_lrl) > 1.0)
  {
    length = { DUBINS_NONE, DUBINS_NONE, DUBINS_NONE };
    mode = { DUBINS_L, DUBINS_R, DUBINS_L };
  }
  else
  {
    p_lrl = rmp::common::math::mod2pi(2 * M_PI - acos(p_lrl));
    double t_lrl = rmp::common::math::mod2pi(-alpha + atan2(-cos_a + cos_b, dist + sin_a - sin_b) + p_lrl / 2.0);
    double q_lrl = rmp::common::math::mod2pi(beta - alpha - t_lrl + p_lrl);
    length = { t_lrl, p_lrl, q_lrl };
    mode = { DUBINS_L, DUBINS_R, DUBINS_L };
  }
}

/**
 * @brief Planning path interpolation.
 * @param mode      motion, i.e., DUBINS_L, DUBINS_S, DUBINS_R
 * @param length    Single step motion path length
 * @param init_pose Initial pose (x, y, yaw)
 * @return new_pose	New pose (new_x, new_y, new_yaw) after moving
 */
Point3d DubinsCurve::interpolate(int mode, double length, Point3d init_pose)
{
  double x = init_pose.x(), y = init_pose.y(), yaw = init_pose.theta();
  double new_x, new_y, new_yaw;

  if (mode == DUBINS_S)
  {
    new_x = x + length / max_curv_ * cos(yaw);
    new_y = y + length / max_curv_ * sin(yaw);
    new_yaw = yaw;
  }
  else if (mode == DUBINS_L)
  {
    new_x = x + (sin(yaw + length) - sin(yaw)) / max_curv_;
    new_y = y - (cos(yaw + length) - cos(yaw)) / max_curv_;
    new_yaw = yaw + length;
  }
  else if (mode == DUBINS_R)
  {
    new_x = x - (sin(yaw - length) - sin(yaw)) / max_curv_;
    new_y = y + (cos(yaw - length) - cos(yaw)) / max_curv_;
    new_yaw = yaw - length;
  }
  else
    std::cerr << "Error mode" << std::endl;

  return { new_x, new_y, new_yaw };
}

/**
 * @brief Generate the path.
 * @param start Initial pose (x, y, yaw)
 * @param goal  Target pose (x, y, yaw)
 * @return path The smoothed trajectory points
 */
Points2d DubinsCurve::generation(Point3d start, Point3d goal)
{
  Points2d path;
  double sx = start.x(), sy = start.y(), syaw = start.theta();
  double gx = goal.x(), gy = goal.y(), gyaw = goal.theta();

  // coordinate transformation
  gx -= sx;
  gy -= sy;
  double theta = rmp::common::math::mod2pi(atan2(gy, gx));
  double dist = std::hypot(gx, gy) * max_curv_;
  double alpha = rmp::common::math::mod2pi(syaw - theta);
  double beta = rmp::common::math::mod2pi(gyaw - theta);

  // select the best motion
  DubinsMode best_mode;
  double best_cost = std::numeric_limits<double>::max();
  DubinsLength length;
  DubinsLength best_length = { DUBINS_NONE, DUBINS_NONE, DUBINS_NONE };
  DubinsMode mode;

  for (const auto solver : dubins_solvers)
  {
    solver(alpha, beta, dist, length, mode);
    _update(length, mode, best_length, best_mode, best_cost);
  }

  if (best_cost == std::numeric_limits<double>::max())
    return path;

  // interpolation
  int points_num = int(best_cost / step_) + 6;

  std::vector<double> path_x(points_num);
  std::vector<double> path_y(points_num);
  std::vector<double> path_yaw(points_num, alpha);

  std::vector<int> mode_v = { std::get<0>(best_mode), std::get<1>(best_mode), std::get<2>(best_mode) };
  std::vector<double> length_v = { std::get<0>(best_length), std::get<1>(best_length), std::get<2>(best_length) };

  int i = 0;
  for (int j = 0; j < 3; j++)
  {
    int m = mode_v[j];
    double seg_length = length_v[j];
    // path increment
    double d_l = seg_length > 0.0 ? step_ : -step_;
    double x = path_x[i];
    double y = path_y[i];
    double yaw = path_yaw[i];
    // current path length
    double l = d_l;
    while (fabs(l) <= fabs(seg_length))
    {
      i += 1;
      auto inter_pose = interpolate(m, l, { x, y, yaw });
      path_x[i] = inter_pose.x(), path_y[i] = inter_pose.y(), path_yaw[i] = inter_pose.theta();
      l += d_l;
    }
    i += 1;
    auto inter_pose = interpolate(m, seg_length, { x, y, yaw });
    path_x[i] = inter_pose.x(), path_y[i] = inter_pose.y(), path_yaw[i] = inter_pose.theta();
  }

  // remove unused data
  while ((path_x.size() >= 1) && (path_x.back() == 0.0))
  {
    path_x.pop_back();
    path_y.pop_back();
    path_yaw.pop_back();
  }

  // coordinate transformation
  Eigen::AngleAxisd r_vec(theta, Eigen::Vector3d(0, 0, 1));
  Eigen::Matrix3d R = r_vec.toRotationMatrix();
  Eigen::MatrixXd P = Eigen::MatrixXd::Ones(3, path_x.size());

  for (size_t i = 0; i < path_x.size(); i++)
  {
    P(0, i) = path_x[i];
    P(1, i) = path_y[i];
  }
  P = R * P;

  for (size_t i = 0; i < path_x.size(); i++)
    path.push_back({ P(0, i) + sx, P(1, i) + sy });

  return path;
}

/**
 * @brief Running trajectory generation
 * @param points path points <x, y>
 * @param path generated trajectory
 * @return true if generate successfully, else failed
 */
bool DubinsCurve::run(const Points2d points, Points2d& path)
{
  if (points.size() < 2)
    return false;
  else
  {
    Points3d poses;
    poses.emplace_back(points.begin()->x(), points.begin()->y());
    for (size_t i = 1; i < points.size() - 1; i++)
    {
      double theta1 = std::atan2(points[i].y() - points[i - 1].y(), points[i].x() - points[i - 1].x());
      double theta2 = std::atan2(points[i + 1].y() - points[i].y(), points[i + 1].x() - points[i].x());
      poses.emplace_back(points[i].x(), points[i].y(), (theta1 + theta2) / 2);
    }
    poses.emplace_back(points.back().x(), points.back().y(), 0);

    return run(poses, path);
  }
}

/**
 * @brief Running trajectory generation
 * @param points path points <x, y, theta>
 * @param path generated trajectory
 * @return true if generate successfully, else failed
 */
bool DubinsCurve::run(const Points3d points, Points2d& path)
{
  if (points.size() < 2)
    return false;
  else
  {
    path.clear();
    for (size_t i = 0; i < points.size() - 1; i++)
    {
      Points2d path_i = generation(points[i], points[i + 1]);
      if (!path_i.empty())
        path.insert(path.end(), path_i.begin(), path_i.end());
    }

    return !path.empty();
  }
}

/**
 * @brief Configure the maximum curvature.
 * @param max_curv  the maximum curvature
 */
void DubinsCurve::setMaxCurv(double max_curv)
{
  assert(max_curv > 0);
  max_curv_ = max_curv;
}

/**
 * @brief Update the best motion mode.
 * @param length      the current motion length
 * @param mode        the current motion mode
 * @param best_length the best motion length so far
 * @param best_mode   the best motion mode so far
 * @param best_cost   the best motion cost so far
 */
void DubinsCurve::_update(DubinsLength length, DubinsMode mode, DubinsLength& best_length, DubinsMode& best_mode,
                          double& best_cost)
{
  if (std::get<0>(length) != DUBINS_NONE)
  {
    double t, p, q;
    std::tie(t, p, q) = length;
    double cost = fabs(t) + fabs(p) + fabs(q);
    if (best_cost > cost)
    {
      best_length = length;
      best_mode = mode;
      best_cost = cost;
    }
  }
}
}  // namespace geometry
}  // namespace common
}  // namespace rmp