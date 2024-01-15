/**
 * *********************************************************
 *
 * @file: polynomial_curve.cpp
 * @brief: Polynomial curve generation
 * @author: Yang Haodong
 * @date: 2023-12-26
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
#include <algorithm>
#include <cassert>
#include "polynomial_curve.h"

namespace trajectory_generation
{
// Polynomial interpolation solver
Poly::Poly(std::tuple<double, double, double> state_0, std::tuple<double, double, double> state_1, double t)
{
  double x0, v0, a0;
  double xt, vt, at;
  std::tie(x0, v0, a0) = state_0;
  std::tie(xt, vt, at) = state_1;

  Eigen::Matrix3d A;
  A << std::pow(t, 3), std::pow(t, 4), std::pow(t, 5), 3 * std::pow(t, 2), 4 * std::pow(t, 3), 5 * std::pow(t, 4),
      6 * t, 12 * std::pow(t, 2), 20 * std::pow(t, 3);

  Eigen::Vector3d b(xt - x0 - v0 * t - a0 * t * t / 2, vt - v0 - a0 * t, at - a0);
  Eigen::Vector3d x = A.lu().solve(b);

  // Quintic polynomial coefficient
  p0 = x0;
  p1 = v0;
  p2 = a0 / 2.0;
  p3 = x(0);
  p4 = x(1);
  p5 = x(2);
}

Poly::~Poly()
{
}

double Poly::x(double t)
{
  return p0 + p1 * t + p2 * std::pow(t, 2) + p3 * std::pow(t, 3) + p4 * std::pow(t, 4) + p5 * std::pow(t, 5);
}
double Poly::dx(double t)
{
  return p1 + 2 * p2 * t + 3 * p3 * std::pow(t, 2) + 4 * p4 * std::pow(t, 3) + 5 * p5 * std::pow(t, 4);
}
double Poly::ddx(double t)
{
  return 2 * p2 + 6 * p3 * t + 12 * p4 * std::pow(t, 2) + 20 * p5 * std::pow(t, 3);
}
double Poly::dddx(double t)
{
  return 6 * p3 + 24 * p4 * t + 60 * p5 * std::pow(t, 2);
}

/**
 * @brief Construct a new Polynomial trajectory  object
 */
PolyTrajectory::PolyTrajectory()
{
  clear();
}

/**
 * @brief Destroy the Polynomial trajectory object
 */
PolyTrajectory::~PolyTrajectory()
{
}

/**
 * @brief Clear the Polynomial trajectory
 */
void PolyTrajectory::clear()
{
  time_.clear();
  x_.clear();
  y_.clear();
  yaw_.clear();
  v_.clear();
  a_.clear();
  jerk_.clear();
}

/**
 * @brief Append the state to Polynomial trajectory
 * @param time  Current time step
 * @param x     Current x-position
 * @param y     Current y-position
 * @param yaw   Current yaw angle
 * @param v     Current speed
 * @param a     Current acceleration
 * @param jerk  Current jerk
 */
void PolyTrajectory::append(double time, double x, double y, double yaw, double v, double a, double jerk)
{
  time_.push_back(time);
  x_.push_back(x);
  y_.push_back(y);
  yaw_.push_back(yaw);
  v_.push_back(v);
  a_.push_back(a);
  jerk_.push_back(jerk);
}

/**
 * @brief Determine the direction of the motion trajectory
 * @param mode  trajectory mode, i.e., acceleration or jerk
 * @return direction  1 is positive direction and -1 is nagetive, 0 is invalid
 */
int PolyTrajectory::dir(int mode)
{
  if (mode == POLY_DIR_ACC)
  {
    if (v_.size() >= 2)
    {
      if (v_[v_.size() - 1] < v_[v_.size() - 2])
        return -1;
      else
        return 1;
    }
    else
      return 0;
  }
  else
  {
    if (a_.size() >= 2)
    {
      if (a_[a_.size() - 1] < a_[a_.size() - 2])
        return -1;
      else
        return 1;
    }
    else
      return 0;
  }
}

/**
 * @brief Determine whether the generated trajectory is valid
 * @param max_acc     Maximum acceleration
 * @param max_jerk    Maximum jerk
 * @return flag       true is valid else invalid
 */
bool PolyTrajectory::valid(double max_acc, double max_jerk)
{
  if ((*std::max_element(a_.begin(), a_.end()) <= max_acc) &&
      (*std::max_element(jerk_.begin(), jerk_.end()) <= max_jerk))
    return true;
  else
    return false;
}

/**
 * @brief Calculate the size of trajectory
 * @return size   the size of trajectory
 */
double PolyTrajectory::size()
{
  assert(time_.size() == x_.size());
  assert(x_.size() == y_.size());
  assert(y_.size() == yaw_.size());
  assert(yaw_.size() == v_.size());
  assert(v_.size() == a_.size());
  assert(a_.size() == jerk_.size());
  return time_.size();
}

/**
 * @brief Convert the trajectory to path points
 * @return path  path points (x, y)
 */
Points2d PolyTrajectory::toPath()
{
  Points2d path;
  for (size_t i = 0; i < size(); i++)
    path.push_back({ x_[i], y_[i] });
  return path;
}

/**
 * @brief Construct a new Polynomial generation object
 * @param max_acc     Maximum acceleration (default: 1.0)
 * @param max_jerk    Maximum jerk (default: 0.5)
 */
Polynomial::Polynomial(double step, double max_acc, double max_jerk)
  : Curve(step), max_acc_(max_acc), max_jerk_(max_jerk)
{
}
Polynomial::Polynomial() : Curve(2.0), max_acc_(3.0), max_jerk_(1.0)
{
}

/**
 * @brief Destroy the Polynomial generation object
 */
Polynomial::~Polynomial()
{
}

/**
 * @brief Generate a valid trajectory from start state to goal state
 * @param start_state   start state
 * @param goal_state    goal state
 * @param traj    the trajectory
 */
void Polynomial::generation(PolyState start_state, PolyState goal_state, PolyTrajectory& traj)
{
  //  simulation parameters
  double t_min = 1.0;
  double t_max = 30.0;
  double dt = 0.5;

  double sx, sy, syaw, sv, sa;
  double gx, gy, gyaw, gv, ga;
  std::tie(sx, sy, syaw, sv, sa) = start_state;
  std::tie(gx, gy, gyaw, gv, ga) = goal_state;

  double sv_x = sv * cos(syaw);
  double sv_y = sv * sin(syaw);
  double gv_x = gv * cos(gyaw);
  double gv_y = gv * sin(gyaw);

  double sa_x = sa * cos(syaw);
  double sa_y = sa * sin(syaw);
  double ga_x = ga * cos(gyaw);
  double ga_y = ga * sin(gyaw);

  traj.clear();

  double T = t_min;
  while (T < t_max)
  {
    Poly x_psolver({ sx, sv_x, sa_x }, { gx, gv_x, ga_x }, T);
    Poly y_psolver({ sy, sv_y, sa_y }, { gy, gv_y, ga_y }, T);
    double t = 0.0;
    while (t < T + dt)
    {
      double vx = x_psolver.dx(t);
      double vy = y_psolver.dx(t);
      double v = hypot(vx, vy);
      double yaw = atan2(vy, vx);

      double ax = x_psolver.ddx(t);
      double ay = y_psolver.ddx(t);
      double a = hypot(ax, ay);
      a = traj.dir(POLY_DIR_ACC) ? a * traj.dir(POLY_DIR_ACC) : a;

      double jx = x_psolver.dddx(t);
      double jy = y_psolver.dddx(t);
      double j = hypot(jx, jy);
      j = traj.dir(POLY_DIR_JERK) ? j * traj.dir(POLY_DIR_JERK) : j;

      traj.append(t, x_psolver.x(t), y_psolver.x(t), v, yaw, a, j);

      t += dt;
    }

    if (traj.valid(max_acc_, max_jerk_))
      break;
    else
      traj.clear();

    T += step_;
  }
}

/**
 * @brief Running trajectory generation
 * @param points path points <x, y>
 * @param path generated trajectory
 * @return true if generate successfully, else failed
 */
bool Polynomial::run(const Points2d points, Points2d& path)
{
  if (points.size() < 4)
    return false;
  else
  {
    Poses2d poses;
    poses.emplace_back(points.begin()->first, points.begin()->second, 0);
    for (size_t i = 1; i < points.size() - 1; i++)
    {
      double theta1 = helper::angle(points[i - 1], points[i]);
      double theta2 = helper::angle(points[i], points[i + 1]);
      poses.emplace_back(points[i].first, points[i].second, (theta1 + theta2) / 2);
    }
    poses.emplace_back(points.back().first, points.back().second, 0);

    return run(poses, path);
  }
}

/**
 * @brief Running trajectory generation
 * @param points path points <x, y, theta>
 * @param path generated trajectory
 * @return true if generate successfully, else failed
 */
bool Polynomial::run(const Poses2d points, Points2d& path)
{
  if (points.size() < 4)
    return false;
  else
  {
    path.clear();

    // generate velocity and acceleration constraints heuristically
    std::vector<double> v(points.size(), 1.0);
    v[0] = 0.0;

    std::vector<double> a;
    for (size_t i = 0; i < points.size() - 1; i++)
      a.push_back((v[i + 1] - v[i]) / 5);
    a.push_back(0.0);

    for (size_t i = 0; i < points.size() - 1; i++)
    {
      PolyTrajectory traj;
      PolyState start(std::get<0>(points[i]), std::get<1>(points[i]), std::get<2>(points[i]), v[i], a[i]);
      PolyState goal(std::get<0>(points[i + 1]), std::get<1>(points[i + 1]), std::get<2>(points[i + 1]), v[i + 1],
                     a[i + 1]);

      generation(start, goal, traj);

      Points2d path_i = traj.toPath();
      path.insert(path.end(), path_i.begin(), path_i.end());
    }

    return !path.empty();
  }
}

/**
 * @brief Configure the maximum acceleration.
 * @param max_acc  The maximum acceleration
 */
void Polynomial::setMaxAcceleration(double max_acc)
{
  assert(max_acc > 0);
  max_acc_ = max_acc;
}

/**
 * @brief Configure the maximum jerk.
 * @param max_jerk  The maximum jerk
 */
void Polynomial::setMaxJerk(double max_jerk)
{
  assert(max_jerk > 0);
  max_jerk_ = max_jerk;
}

}  // namespace trajectory_generation