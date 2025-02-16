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
#ifndef RMP_COMMON_GEOMETRY_QUINTIC_POLYNOMIAL_H_
#define RMP_COMMON_GEOMETRY_QUINTIC_POLYNOMIAL_H_

#include <array>

namespace rmp
{
namespace common
{
namespace geometry
{
class QuinticPolynomial
{
public:
  QuinticPolynomial();
  QuinticPolynomial(const std::array<double, 6>& params);
  ~QuinticPolynomial() = default;

  void update(const std::array<double, 6>& params);
  void solve(const std::array<double, 3>& start_pva, const std::array<double, 3>& end_pva, double T);

  double x(double t) const;
  double dx(double t) const;
  double ddx(double t) const;
  double dddx(double t) const;
  double ddddx(double t) const;

private:
  double a0_, a1_, a2_, a3_, a4_, a5_;
};
}  // namespace geometry
}  // namespace common
}  // namespace rmp

#endif