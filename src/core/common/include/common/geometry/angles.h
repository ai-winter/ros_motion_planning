/**
 * *********************************************************
 *
 * @file: angles.h
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
#ifndef RMP_COMMON_MATH_ANGLES_H_
#define RMP_COMMON_MATH_ANGLES_H_

#include <vector>
#include <Eigen/Dense>

#include "common/math/math_helper.h"

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
double fromDegrees(const double degrees);

/**
 * Convert radians to degrees.
 * @param radians Angle in radians.
 * @return Angle in degrees.
 */
double toDegrees(const double radians);

/**
 * Normalize angle to the range [0, 2*pi).
 * @param angle Angle in radians.
 * @return Angle normalized to [0, 2*pi).
 */
double normalizeAnglePositive(double angle);

/**
 * Normalize angle to the range (-pi, pi].
 * @param angle Angle in radians.
 * @return Angle normalized to (-pi, pi].
 */
double normalizeAngle(double angle);

/**
 * Compute the shortest angular distance between two angles.
 * @param from_angle Starting angle in radians.
 * @param to_angle Target angle in radians.
 * @return Shortest signed angular distance in radians.
 */
double shortestAngularDistance(const double from_angle, const double to_angle);

/**
 * @brief Computes a 2D rotation matrix for a given angle.
 * @param angle The rotation angle in radians.
 * @return A 2x2 Eigen::Matrix2d representing the rotation matrix.
 *         The matrix is computed as:
 *         [ cos(angle)  -sin(angle) ]
 *         [ sin(angle)   cos(angle) ]
 */
Eigen::Matrix2d getRotationMatrix(const double angle);

/**
 * @brief Calculate the smallest angular difference between two angles.
 * @param angle_1 The starting angle in radians.
 * @param angle_2 The ending angle in radians.
 * @return The smallest angular difference.
 */
double angleDifference(double angle_1, double angle_2);

/**
 * @brief Calculate the angular difference between two angles with direction consideration.
 * @param angle_1 The starting angle in radians.
 * @param angle_2 The ending angle in radians.
 * @param left_turn true for left turn, false for right turn,
 * @return The angular difference in radians according to specified direction.
 */
double angleDifference(double angle_1, double angle_2, bool left_turn);

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
void interpolateAngles(double start_angle, double end_angle, bool left_turn, int steps, std::vector<double>& result);

template <typename T>
class Quaternion;

template <typename T>
class EulerAnglesZYX
{
  /**
   * @note
   *
   * Any orientation of a rigid body on a 3-D space can be achieved by
   * composing three rotations about the axes of an orthogonal coordinate system.
   * These rotations are said to be extrinsic if the axes are assumed to be
   * motionless, and intrinsic otherwise. Here, we use an intrinsic referential,
   * which is relative to the car's orientation.
   * Our vehicle reference frame follows:
   * Forward/Left/Up (FLU) respectively for the axes x/y/z.
   * In particular, we describe the orientation of the car by three angles:
   * 1) the pitch, in (-pi/2, pi/2), corresponds to a rotation around the y-axis;
   * 2) the roll, in [-pi, pi), corresponds to a rotation around the x-axis;
   * 3) the yaw, in [-pi, pi), corresponds to a rotation around the z-axis.
   * The pitch is zero when the car is level and positive when the nose is up.
   * The roll is zero when the car is level and positive when the left part is up.
   * The yaw is zero when the car is facing North, and positive when facing West.
   * In turn, in the world frame, the x/y/z axes point to East/North/Up (ENU).
   * These angles represent the rotation from the world to the vehicle frames.
   * EulerAnglesZYX implements a class of Euler angles (actually, Tait-Bryan angles),
   * with intrinsic sequence ZYX.
   */
public:
  /**
   * @brief Constructs an identity rotation.
   */
  EulerAnglesZYX() : roll_(0), pitch_(0), yaw_(0)
  {
  }

  /**
   * @brief Constructs a rotation using only yaw (i.e., around the z-axis).
   *
   * @param yaw The yaw of the car
   */
  explicit EulerAnglesZYX(T yaw) : roll_(0), pitch_(0), yaw_(yaw)
  {
  }

  /**
   * @brief Constructs a rotation using arbitrary roll, pitch, and yaw.
   *
   * @param roll The roll of the car
   * @param pitch The pitch of the car
   * @param yaw The yaw of the car
   */
  EulerAnglesZYX(T roll, T pitch, T yaw) : roll_(roll), pitch_(pitch), yaw_(yaw)
  {
  }

  /**
   * @brief Constructs a rotation using components of a quaternion.
   *
   * @param qw Quaternion w-coordinate
   * @param qx Quaternion x-coordinate
   * @param qy Quaternion y-coordinate
   * @param qz Quaternion z-coordinate
   */
  EulerAnglesZYX(T qw, T qx, T qy, T qz)
  {
    const T d = qw * qw + qx * qx + qy * qy + qz * qz;
    const T s = static_cast<T>(2.0) / d;
    const T xs = qx * s, ys = qy * s, zs = qz * s;
    const T wx = qw * xs, wy = qw * ys, wz = qw * zs;
    const T xx = qx * xs, xy = qx * ys, xz = qx * zs;
    const T yy = qy * ys, yz = qy * zs, zz = qz * zs;
    const T rot_xx = static_cast<T>(1.0) - (yy + zz);
    const T rot_xy = xy - wz;
    const T rot_xz = xz + wy;
    const T rot_yx = xy + wz;
    const T rot_yy = static_cast<T>(1.0) - (xx + zz);
    const T rot_yz = yz - wx;
    const T rot_zx = xz - wy;
    const T rot_zy = yz + wx;
    const T rot_zz = static_cast<T>(1.0) - (xx + yy);

    // Check that pitch is not at a singularity
    if (std::fabs(rot_zx) >= 1)
    {
      yaw_ = static_cast<T>(0);
      const T delta = std::atan2(rot_zy, rot_zz);
      if (rot_zx < 0)
      {
        pitch_ = static_cast<T>(M_PI_2);
        roll_ = delta;
      }
      else
      {
        pitch_ = -static_cast<T>(M_PI_2);
        roll_ = delta;
      }
    }
    else
    {
      pitch_ = -std::asin(rot_zx);
      roll_ = std::atan2(rot_zy, rot_zz);
      yaw_ = std::atan2(rot_yx, rot_xx);
    }
  }

  /**
   * @brief Constructs a rotation from quaternion.
   * @param q Quaternion
   */
  explicit EulerAnglesZYX(const Quaternion<T>& q) : EulerAnglesZYX(q.w(), q.x(), q.y(), q.z())
  {
  }

  explicit EulerAnglesZYX(const Eigen::Matrix<T, 3, 3>& rot_matrix)
    : roll_(std::atan2(rot_matrix(2, 1), rot_matrix(2, 2)))
    , pitch_(std::atan2(-rot_matrix(2, 0), std::hypot(rot_matrix(2, 1), rot_matrix(2, 2))))
    , yaw_(std::atan2(rot_matrix(1, 0), rot_matrix(0, 0)))
  {
  }

  /**
   * @brief Getter for roll_
   * @return The roll of the car
   */
  T roll() const
  {
    return roll_;
  }

  /**
   * @brief Getter for pitch_
   * @return The pitch of the car
   */
  T pitch() const
  {
    return pitch_;
  }

  /**
   * @brief Getter for yaw_
   * @return The yaw of the car
   */
  T yaw() const
  {
    return yaw_;
  }

  /**
   * @brief Normalizes roll_, pitch_, and yaw_ to [-PI, PI).
   */
  void normalize()
  {
    roll_ = normalizeAngle(roll_);
    pitch_ = normalizeAngle(pitch_);
    yaw_ = normalizeAngle(yaw_);
  }

  /**
   * @brief Verifies the validity of the specified rotation.
   * @return True iff -PI/2 < pitch < PI/2
   */
  bool isValid()
  {
    normalize();
    return pitch_ < M_PI_2 && pitch_ > -M_PI_2;
  }

  /**
   * @brief Converts to a quaternion with a non-negative scalar part
   * @return Quaternion encoding this rotation.
   */
  Quaternion<T> toQuaternion() const
  {
    T coeff = static_cast<T>(0.5);
    T half_r = roll_ * coeff;
    T half_p = pitch_ * coeff;
    T half_y = yaw_ * coeff;

    T sr = std::sin(half_r);
    T sp = std::sin(half_p);
    T sy = std::sin(half_y);
    T cr = std::cos(half_r);
    T cp = std::cos(half_p);
    T cy = std::cos(half_y);

    T qw = cr * cp * cy + sr * sp * sy;
    T qx = sr * cp * cy - cr * sp * sy;
    T qy = cr * sp * cy + sr * cp * sy;
    T qz = cr * cp * sy - sr * sp * cy;
    if (qw < 0.0)
    {
      return { -qw, -qx, -qy, -qz };
    }
    return { qw, qx, qy, qz };
  }

  Eigen::Matrix<T, 3, 3> toRotationMatrix() const
  {
    const T cos_roll = std::cos(roll_);
    const T sin_roll = std::sin(roll_);
    const T cos_pitch = std::cos(pitch_);
    const T sin_pitch = std::sin(pitch_);
    const T cos_yaw = std::cos(yaw_);
    const T sin_yaw = std::sin(yaw_);

    double rot_xx = cos_yaw * cos_pitch;
    double rot_xy = cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
    double rot_xz = cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll;
    double rot_yx = sin_yaw * cos_pitch;
    double rot_yy = sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll;
    double rot_yz = sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll;
    double rot_zx = -sin_pitch;
    double rot_zy = cos_pitch * sin_roll;
    double rot_zz = cos_pitch * cos_roll;

    Eigen::Matrix<T, 3, 3> rot_matrix;
    rot_matrix << rot_xx, rot_xy, rot_xz, rot_yx, rot_yy, rot_yz, rot_zx, rot_zy, rot_zz;
    return rot_matrix;
  }

private:
  T roll_;
  T pitch_;
  T yaw_;
};

using EulerAnglesZYXf = EulerAnglesZYX<float>;
using EulerAnglesZYXd = EulerAnglesZYX<double>;

template <typename T>
class Quaternion
{
public:
  Quaternion(T w, T x, T t, T z) : w_(w), x_(x), y_(y), z_(z)
  {
  }

  Quaternion(T roll, T pitch, T yaw)
  {
    T coeff = static_cast<T>(0.5);
    T half_r = roll * coeff;
    T half_p = pitch * coeff;
    T half_y = yaw * coeff;

    T sr = std::sin(half_r);
    T sp = std::sin(half_p);
    T sy = std::sin(half_y);
    T cr = std::cos(half_r);
    T cp = std::cos(half_p);
    T cy = std::cos(half_y);

    T qw = cr * cp * cy + sr * sp * sy;
    T qx = sr * cp * cy - cr * sp * sy;
    T qy = cr * sp * cy + sr * cp * sy;
    T qz = cr * cp * sy - sr * sp * cy;
    if (qw < 0.0)
    {
      w_ = -qw;
      x_ = -qx;
      y_ = -qy;
      z_ = -qz;
    }
    else
    {
      w_ = qw;
      x_ = qx;
      y_ = qy;
      z_ = qz;
    }
  }

  explicit Quaternion(const EulerAnglesZYX<T>& rpy) : Quaternion(rpy.roll(), rpy.pitch(), rpy.yaw())
  {
  }

  explicit Quaternion(const Eigen::Matrix<T, 3, 3>& rot_matrix)
  {
    const T trace = rot_matrix.trace();
    if (trace > 0.0)
    {
      const T s = std::sqrt(trace + 1.0) * 2.0;
      w_ = static_cast<T>(0.25) * s;
      x_ = (rot_matrix(2, 1) - rot_matrix(1, 2)) / s;
      y_ = (rot_matrix(0, 2) - rot_matrix(2, 0)) / s;
      z_ = (rot_matrix(1, 0) - rot_matrix(0, 1)) / s;
    }
    else if ((rot_matrix(0, 0) > rot_matrix(1, 1)) && (rot_matrix(0, 0) > rot_matrix(2, 2)))
    {
      const T s = std::sqrt(1.0 + rot_matrix(0, 0) - rot_matrix(1, 1) - rot_matrix(2, 2)) * 2.0;
      w_ = (rot_matrix(2, 1) - rot_matrix(1, 2)) / s;
      x_ = 0.25 * s;
      y_ = (rot_matrix(0, 1) + rot_matrix(1, 0)) / s;
      z_ = (rot_matrix(0, 2) + rot_matrix(2, 0)) / s;
    }
    else if (rot_matrix(1, 1) > rot_matrix(2, 2))
    {
      const T s = std::sqrt(1.0 + rot_matrix(1, 1) - rot_matrix(0, 0) - rot_matrix(2, 2)) * 2.0;
      w_ = (rot_matrix(0, 2) - rot_matrix(2, 0)) / s;
      x_ = (rot_matrix(0, 1) + rot_matrix(1, 0)) / s;
      y_ = 0.25 * s;
      z_ = (rot_matrix(1, 2) + rot_matrix(2, 1)) / s;
    }
    else
    {
      const T s = std::sqrt(1.0 + rot_matrix(2, 2) - rot_matrix(0, 0) - rot_matrix(1, 1)) * 2.0;
      w_ = (rot_matrix(1, 0) - rot_matrix(0, 1)) / s;
      x_ = (rot_matrix(0, 2) + rot_matrix(2, 0)) / s;
      y_ = (rot_matrix(1, 2) + rot_matrix(2, 1)) / s;
      z_ = 0.25 * s;
    }
  }

  Eigen::Matrix<T, 3, 3> toRotationMatrix() const
  {
    const T d = w_ * w_ + x_ * x_ + y_ * y_ + z_ * z_;
    const T s = static_cast<T>(2.0) / d;
    const T xs = x_ * s, ys = y_ * s, zs = z_ * s;
    const T wx = w_ * xs, wy = w_ * ys, wz = w_ * zs;
    const T xx = x_ * xs, xy = x_ * ys, xz = x_ * zs;
    const T yy = y_ * ys, yz = y_ * zs, zz = z_ * zs;
    const T rot_xx = static_cast<T>(1.0) - (yy + zz);
    const T rot_xy = xy - wz;
    const T rot_xz = xz + wy;
    const T rot_yx = xy + wz;
    const T rot_yy = static_cast<T>(1.0) - (xx + zz);
    const T rot_yz = yz - wx;
    const T rot_zx = xz - wy;
    const T rot_zy = yz + wx;
    const T rot_zz = static_cast<T>(1.0) - (xx + yy);
    Eigen::Matrix<T, 3, 3> rot_matrix;
    rot_matrix << rot_xx, rot_xy, rot_xz, rot_yx, rot_yy, rot_yz, rot_zx, rot_zy, rot_zz;
    return rot_matrix;
  }

  T w()
  {
    return w_;
  }

  T x()
  {
    return x_;
  }

  T y()
  {
    return y_;
  }

  T z()
  {
    return z_;
  }

private:
  T w_, x_, y_, z_;
};

inline double quaternionToHeading(const double qw, const double qx, const double qy, const double qz)
{
  EulerAnglesZYXd euler_angles(qw, qx, qy, qz);
  return normalizeAngle(euler_angles.yaw());
}

template <typename T>
inline T quaternionToHeading(const Eigen::Quaternion<T>& q)
{
  return static_cast<T>(quaternionToHeading(q.w(), q.x(), q.y(), q.z()));
}
}  // namespace geometry
}  // namespace common
}  // namespace rmp

#endif