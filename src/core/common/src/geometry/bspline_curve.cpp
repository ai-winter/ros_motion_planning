/**
 * *********************************************************
 *
 * @file: bspline_curve.cpp
 * @brief: B-Spline curve generation
 * @author: Yang Haodong
 * @date: 2023-12-20
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
#include <cassert>
#include "common/geometry/bspline_curve.h"

namespace rmp
{
namespace common
{
namespace geometry
{
/**
 * @brief Construct a new B-Spline generation object
 * @param step        Simulation or interpolation size (default: 0.01)
 * @param order       Degree of curve (default: 3)
 * @param param_mode  Parameterization mode (default: PARAM_MODE_CHORDLENGTH)
 * @param spline_mode B-Spline generation mode (default: SPLINE_MODE_INTERPOLATION)
 */
BSplineCurve::BSplineCurve(double step, int order, int param_mode, int spline_mode)
  : Curve(step), order_(order), param_mode_(param_mode), spline_mode_(spline_mode)
{
}
BSplineCurve::BSplineCurve()
  : Curve(0.01), order_(3), param_mode_(PARAM_MODE::CENTRIPETAL), spline_mode_(SPLINE_MODE::INTERPOLATION)
{
}

/**
 * @brief Destroy the B-Spline generation object
 */
BSplineCurve::~BSplineCurve()
{
}

/**
 * @brief Calculate base function using Cox-deBoor function.
 * @param i       The index of base function
 * @param k       The degree of curve
 * @param t       Parameter
 * @param knot    Knot vector
 * @return  Nik_t The value of base function Nik(t)
 */
double BSplineCurve::baseFunction(int i, int k, double t, std::vector<double> knot)
{
  double Nik_t = 0;

  // 1st order B-spline
  if (k == 0)
    Nik_t = ((t >= knot[i]) && (t < knot[i + 1])) ? 1.0 : 0.0;
  // 2nd order and higher B-spline
  else
  {
    double length1 = double(knot[i + k]) - knot[i];
    double length2 = double(knot[i + k + 1]) - knot[i + 1];

    // Handle the case where the denominator is 0 by replacing it with 1, defining 0/0 as 0
    if ((length1 == 0) && (length2 == 0))
      Nik_t = 0;
    else if (length1 == 0)
      Nik_t = (knot[i + k + 1] - t) / length2 * baseFunction(i + 1, k - 1, t, knot);
    else if (length2 == 0)
      Nik_t = (t - knot[i]) / length1 * baseFunction(i, k - 1, t, knot);
    else
      Nik_t = (t - knot[i]) / length1 * baseFunction(i, k - 1, t, knot) +
              (knot[i + k + 1] - t) / length2 * baseFunction(i + 1, k - 1, t, knot);
  }
  return Nik_t;
}

/**
 * @brief Calculate parameters using the `uniform spaced` or `chrod length` or `centripetal` method.
 * @param points      Path points
 * @return parameters The parameters of given points
 */
std::vector<double> BSplineCurve::paramSelection(const Points2d points)
{
  size_t n = points.size();
  std::vector<double> parameters(n);

  if (param_mode_ == PARAM_MODE::UNIFORMSPACED)
  {
    for (size_t i = 0; i < n; i++)
      parameters[i] = (double)(i) / (double)(n - 1);
  }
  else
  {
    parameters[0] = 0.0;
    std::vector<double> s(n - 1);

    double d_cumsum = 0.0;
    for (size_t i = 0; i < n - 1; i++)
    {
      double d;
      if (param_mode_ == PARAM_MODE::CHORDLENGTH)
      {
        d = std::hypot(points[i].x() - points[i + 1].x(), points[i].y() - points[i + 1].y());
      }
      else
      {
        double alpha = 0.5;
        d = std::pow(std::hypot(points[i].x() - points[i + 1].x(), points[i].y() - points[i + 1].y()), alpha);
      }
      d_cumsum += d;
      s[i] = d_cumsum;
    }
    for (size_t i = 1; i < n; i++)
      parameters[i] = s[i - 1] / s[n - 2];
  }
  return parameters;
}

/**
 * @brief Generate knot vector.
 * @param parameters The parameters of given points
 * @param n          The number of data points
 * @return knot The knot vector
 */
std::vector<double> BSplineCurve::knotGeneration(const std::vector<double> param, int n)
{
  int m = n + order_ + 1;
  std::vector<double> knot(m);

  for (size_t i = 0; i < n; i++)
    knot[i] = 0.0;
  for (size_t i = n; i < m; i++)
    knot[i] = 1.0;
  for (size_t i = order_ + 1; i < n; i++)
  {
    for (size_t j = i - order_; j < i; j++)
      knot[i] += param[j];
    knot[i] /= order_;
  }
  return knot;
}

/**
 * @brief Given a set of N data points, D0, D1, ..., Dn and a degree k, find a B-spline curve of degree
 *      k defined by N control points that passes all data points in the given order.
 * @param points          Path points
 * @param parameters      The parameters of given points
 * @param knot            The knot vector
 * @return control_points The control points
 */
Points2d BSplineCurve::interpolation(const Points2d points, const std::vector<double> param,
                                     const std::vector<double> knot)
{
  size_t n = points.size();
  Eigen::MatrixXd N = Eigen::MatrixXd::Zero(n, n);
  Eigen::MatrixXd D(n, 2);

  for (size_t i = 0; i < n; i++)
    for (size_t j = 0; j < n; j++)
      N(i, j) = baseFunction(j, order_, param[i], knot);
  N(n - 1, n - 1) = 1;

  for (size_t i = 0; i < n; i++)
  {
    D(i, 0) = points[i].x();
    D(i, 1) = points[i].y();
  }

  Eigen::MatrixXd C = N.inverse() * D;

  Points2d control_points(n);

  for (size_t i = 0; i < n; i++)
  {
    control_points[i] = { C(i, 0), C(i, 1) };
  }

  return control_points;
}

/**
 * @brief Given a set of N data points, D0, D1, ..., Dn, a degree k, and a number H, where N > H > k >= 1,
 *    find a B-spline curve of degree k defined by H control points that satisfies the following conditions:
        1. this curve contains the first and last data points;
        2. this curve approximates the data polygon in the sense of least square
 * @param points          Path points
 * @param parameters      The parameters of given points
 * @param knot            The knot vector
 * @return control_points The control points
 */
Points2d BSplineCurve::approximation(const Points2d points, const std::vector<double> param,
                                     const std::vector<double> knot)
{
  size_t n = points.size();
  Eigen::MatrixXd D(n, 2);
  for (size_t i = 0; i < n; i++)
  {
    D(i, 0) = points[i].x();
    D(i, 1) = points[i].y();
  }

  // heuristically setting the number of control points
  size_t h = n - 1;
  Eigen::MatrixXd N = Eigen::MatrixXd::Zero(n, h);
  for (size_t i = 0; i < n; i++)
    for (size_t j = 0; j < h; j++)
      N(i, j) = baseFunction(j, order_, param[i], knot);

  Eigen::MatrixXd N_ = Eigen::MatrixXd::Zero(n - 2, h - 2);
  for (size_t i = 1; i < n - 1; i++)
    for (size_t j = 1; j < h - 1; j++)
      N_(i - 1, j - 1) = N(i, j);

  Eigen::MatrixXd qk = Eigen::MatrixXd::Zero(n - 2, 2);
  for (size_t i = 1; i < n - 1; i++)
  {
    qk(i - 1, 0) = D(i, 0) - N(i, 0) * D(0, 0) - N(i, h - 1) * D(n - 1, 0);
    qk(i - 1, 1) = D(i, 1) - N(i, 0) * D(0, 1) - N(i, h - 1) * D(n - 1, 1);
  }

  Eigen::MatrixXd Q = N_.transpose() * qk;
  Eigen::MatrixXd P = (N_.transpose() * N_).inverse() * Q;

  Points2d control_points(h);
  control_points[0] = { D(0, 0), D(0, 1) };
  control_points[h - 1] = { D(n - 1, 0), D(n - 1, 1) };
  for (size_t i = 1; i < h - 1; i++)
    control_points[i] = { P(i - 1, 0), P(i - 1, 1) };

  return control_points;
}

/**
 * @brief Generate the path.
 * @param k               The degree of curve
 * @param knot            The knot vector
 * @param control_points  The control points
 * @return path The smoothed trajectory points
 */
Points2d BSplineCurve::generation(int k, const std::vector<double> knot, Points2d control_pts)
{
  size_t n = static_cast<int>(1.0 / step_);
  std::vector<double> t(n);
  for (size_t i = 0; i < n; i++)
    t[i] = (double)(i) / (double)(n - 1);

  Eigen::MatrixXd N(n, control_pts.size());
  for (size_t i = 0; i < n; i++)
    for (size_t j = 0; j < control_pts.size(); j++)
      N(i, j) = baseFunction(j, order_, t[i], knot);

  N(n - 1, control_pts.size() - 1) = 1.0;

  Eigen::MatrixXd C(control_pts.size(), 2);
  for (size_t i = 0; i < control_pts.size(); i++)
  {
    C(i, 0) = control_pts[i].x();
    C(i, 1) = control_pts[i].y();
  }

  Eigen::MatrixXd P = N * C;
  Points2d points(n);
  for (size_t i = 0; i < n; i++)
    points[i] = { P(i, 0), P(i, 1) };

  return points;
}

/**
 * @brief Running trajectory generation
 * @param points path points
 * @param path generated trajectory
 * @return true if generate successfully, else failed
 */
bool BSplineCurve::run(const Points2d points, Points2d& path)
{
  if (points.size() < 4)
    return false;
  else
  {
    Points2d control_pts;
    std::vector<double> params = paramSelection(points);
    std::vector<double> knot = knotGeneration(params, points.size());
    if (spline_mode_ == SPLINE_MODE::INTERPOLATION)
    {
      control_pts = interpolation(points, params, knot);
    }
    else if (spline_mode_ == SPLINE_MODE::APPROXIMATION)
    {
      control_pts = approximation(points, params, knot);
      params = paramSelection(control_pts);
      knot = knotGeneration(params, control_pts.size());
    }
    else
      return false;

    path = generation(order_, knot, control_pts);

    return !path.empty();
  }
}

/**
 * @brief Running trajectory generation
 * @param points path points <x, y, theta>
 * @param path generated trajectory
 * @return true if generate successfully, else failed
 */
bool BSplineCurve::run(const Points3d points, Points2d& path)
{
  Points2d points_pair;
  for (const auto& p : points)
  {
    points_pair.emplace_back(p.x(), p.y());
  }
  return run(points_pair, path);
}

/**
 * @brief Configure the degree of the curve.
 * @param order  The degree of curve
 */
void BSplineCurve::setSplineOrder(int order)
{
  assert(order > 0);
  order_ = order;
}

/**
 * @brief Configure the parameterization mode .
 * @param param_mode  The parameterization mode
 */
void BSplineCurve::setParamMode(int param_mode)
{
  assert((param_mode == PARAM_MODE::CENTRIPETAL) || (param_mode == PARAM_MODE::CHORDLENGTH) ||
         (param_mode == PARAM_MODE::UNIFORMSPACED));
  param_mode_ = param_mode;
}

/**
 * @brief Configure the B-Spline generation mode.
 * @param spline_mode  The B-Spline generation mode
 */
void BSplineCurve::setSPlineMode(int spline_mode)
{
  assert((spline_mode == SPLINE_MODE::APPROXIMATION) || (spline_mode == SPLINE_MODE::INTERPOLATION));
  spline_mode_ = spline_mode;
}
}  // namespace geometry
}  // namespace common
}  // namespace rmp