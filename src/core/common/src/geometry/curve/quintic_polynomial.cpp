/**
 * *********************************************************
 *
 * @file: quintic_polynomial.h
 * @brief: quintic polynomial generation
 * @author: Yang Haodong
 * @date: 2025-1-17
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
#include <Eigen/Dense>

#include "common/geometry/curve/quintic_polynomial.h"

namespace rmp
{
namespace common
{
namespace geometry
{
QuinticPolynomial::QuinticPolynomial() : a0_(0.0), a1_(0.0), a2_(0.0), a3_(0.0), a4_(0.0), a5_(0.0)
{
}

QuinticPolynomial::QuinticPolynomial(const std::array<double, 6>& params)
  : a0_(params[0]), a1_(params[1]), a2_(params[2]), a3_(params[3]), a4_(params[4]), a5_(params[5]){};

void QuinticPolynomial::update(const std::array<double, 6>& params)
{
  a0_ = params[0];
  a1_ = params[1];
  a2_ = params[2];
  a3_ = params[3];
  a4_ = params[4];
  a5_ = params[5];
}

void QuinticPolynomial::solve(const std::array<double, 3>& start_pva, const std::array<double, 3>& end_pva, double T)
{
  double x0 = start_pva[0];
  double v0 = start_pva[1];
  double a0 = start_pva[2];
  double xt = end_pva[0];
  double vt = end_pva[1];
  double at = end_pva[2];

  Eigen::Matrix3d A;
  A << std::pow(T, 3), std::pow(T, 4), std::pow(T, 5), 3 * std::pow(T, 2), 4 * std::pow(T, 3), 5 * std::pow(T, 4),
      6 * T, 12 * std::pow(T, 2), 20 * std::pow(T, 3);

  Eigen::Vector3d b(xt - x0 - v0 * T - a0 * T * T / 2, vt - v0 - a0 * T, at - a0);
  Eigen::Vector3d x = A.lu().solve(b);

  // Quintic polynomial coefficient
  a0_ = x0;
  a1_ = v0;
  a2_ = a0 / 2.0;
  a3_ = x(0);
  a4_ = x(1);
  a5_ = x(2);
}

double QuinticPolynomial::x(double t) const
{
  return a0_ + a1_ * t + a2_ * std::pow(t, 2) + a3_ * std::pow(t, 3) + a4_ * std::pow(t, 4) + a5_ * std::pow(t, 5);
}

double QuinticPolynomial::dx(double t) const
{
  return a1_ + 2 * a2_ * t + 3 * a3_ * std::pow(t, 2) + 4 * a4_ * std::pow(t, 3) + 5 * a5_ * std::pow(t, 4);
}

double QuinticPolynomial::ddx(double t) const
{
  return 2 * a2_ + 6 * a3_ * t + 12 * a4_ * std::pow(t, 2) + 20 * a5_ * std::pow(t, 3);
}

double QuinticPolynomial::dddx(double t) const
{
  return 6 * a3_ + 24 * a4_ * t + 60 * a5_ * std::pow(t, 2);
}

double QuinticPolynomial::ddddx(double t) const
{
  return 24 * a4_ + 120 * a5_ * t;
}
}  // namespace geometry
}  // namespace common
}  // namespace rmp