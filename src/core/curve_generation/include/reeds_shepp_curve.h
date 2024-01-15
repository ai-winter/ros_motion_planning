/**
 * *********************************************************
 *
 * @file: reeds_shepp_curve.h
 * @brief: Reeds shepp curve generation
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
#ifndef REEDS_SHEPP_CURVE_H
#define REEDS_SHEPP_CURVE_H

#define REEDS_SHEPP_NONE -1
#define REEDS_SHEPP_L 0
#define REEDS_SHEPP_S 1
#define REEDS_SHEPP_R 2
#define REEDS_SHEPP_MAX 1e10

#include "curve.h"

namespace trajectory_generation
{
using RSMode = std::tuple<int, int, int>;
// <t, u, v>
using RSLength = std::tuple<double, double, double>;

class RSPath
{
public:
  /**
   * @brief Construct a new Reeds-Shepp Path object
   * @param lengths  the length of segments
   * @param ctypes   the motion patterns of segments
   */
  RSPath(std::vector<double> lengths, std::vector<int> ctypes);

  /**
   * @brief Destroy the Reeds-Shepp Path object
   */
  ~RSPath();

  /**
   * @brief Calculate the length of the generated trajectory
   * @return length   the length of the generated trajectory
   */
  double len();

  /**
   * @brief Determine whether the generated trajectory is valid
   * @return flag   true is valid else invalid
   */
  bool valid();

  /**
   * @brief Calculate the number of segments for generating trajectories
   * @return number   the number of segments for generating trajectories
   */
  size_t size();

  /**
   * @brief Obtain segment i with its motion pattern
   * @param i       index
   * @param length  the length of segment i
   * @param ctype   the motion of segment i
   */
  void get(int i, double& length, int& ctype);

private:
  std::vector<double> lengths_;  // the length of segments
  std::vector<int> ctypes_;      // the motion patterns of segments
};

class ReedsShepp : public Curve
{
public:
  /**
   * @brief Construct a new ReedsShepp generation object
   * @param step        Simulation or interpolation size (default: 0.1)
   * @param max_curv    The maximum curvature of the curve (default: 0.25)
   */
  ReedsShepp();
  ReedsShepp(double step, double max_curv);

  /**
   * @brief Destroy the ReedsShepp generation object
   */
  ~ReedsShepp();

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
  bool run(const Poses2d points, Points2d& path);

  /**
   * @brief Return the polar coordinates (r, theta) of the point (x, y), i.e. rcos(theta) = x; rsin(theta) = y
   */
  void R(double x, double y, double& r, double& theta);

  /**
   * @brief Truncate the angle to the interval of -π to π.
   */
  double M(double theta);

  /**
   * @brief Straight-Left-Straight generation mode.
   */
  bool SLS(double x, double y, double phi, RSLength& length);
  /**
   * @brief Left-Right-Left generation mode. (L+R-L-)
   */
  bool LRL(double x, double y, double phi, RSLength& length);
  /**
   * @brief Left-Straight-Left generation mode. (L+S+L+)
   */
  bool LSL(double x, double y, double phi, RSLength& length);
  /**
   * @brief Left-Straight-Right generation mode. (L+S+R+)
   */
  bool LSR(double x, double y, double phi, RSLength& length);
  /**
   * @brief Left-Right(beta)-Left(beta)-Right generation mode. (L+R+L-R-)
   */
  bool LRLRn(double x, double y, double phi, RSLength& length);
  /**
   * @brief Left-Right(beta)-Left(beta)-Right generation mode. (L+R-L-R+)
   */
  bool LRLRp(double x, double y, double phi, RSLength& length);
  /**
   * @brief Left-Right(pi/2)-Straight-Right generation mode. (L+R-S-R-)
   */
  bool LRSR(double x, double y, double phi, RSLength& length);
  /**
   * @brief Left-Right(pi/2)-Straight-Left generation mode. (L+R-S-L-)
   */
  bool LRSL(double x, double y, double phi, RSLength& length);
  /**
   * @brief Left-Right(pi/2)-Straight-Left(pi/2)-Right generation mode. (L+R-S-L-R+)
   */
  bool LRSLR(double x, double y, double phi, RSLength& length);

  /**
   * @brief # 2 Straight-Circle-Straight generation mode(using reflect).
   * @param x/y Goal position
   * @param phi Goal pose
   * @return Reeds-Shepp Paths
   */
  std::vector<RSPath> SCS(double x, double y, double phi);
  /**
   * @brief # 8 Circle-Circle-Circle generation mode(using reflect, timeflip and backwards).
   * @param x/y Goal position
   * @param phi Goal pose
   * @return Reeds-Shepp Paths
   */
  std::vector<RSPath> CCC(double x, double y, double phi);
  /**
   * @brief # 8 Circle-Straight-Circle generation mode(using reflect, timeflip and backwards).
   * @param x/y Goal position
   * @param phi Goal pose
   * @return Reeds-Shepp Paths
   */
  std::vector<RSPath> CSC(double x, double y, double phi);
  /**
   * @brief # 8 Circle-Circle(beta)-Circle(beta)-Circle generation mode (using reflect, timeflip and backwards).
   * @param x/y Goal position
   * @param phi Goal pose
   * @return Reeds-Shepp Paths
   */
  std::vector<RSPath> CCCC(double x, double y, double phi);
  /**
   * @brief # 16 Circle-Circle(pi/2)-Straight-Circle and Circle-Straight-Circle(pi/2)-Circle
   * generation mode (using reflect, timeflip and backwards).
   * @param x/y Goal position
   * @param phi Goal pose
   * @return Reeds-Shepp Paths
   */
  std::vector<RSPath> CCSC(double x, double y, double phi);
  /**
   * @brief # 4 Circle-Circle(pi/2)-Straight--Circle(pi/2)-Circle generation mode (using reflect, timeflip and
   * backwards).
   * @param x/y Goal position
   * @param phi Goal pose
   * @return Reeds-Shepp Paths
   */
  std::vector<RSPath> CCSCC(double x, double y, double phi);

  /**
   * @brief Planning path interpolation.
   * @param mode      motion, i.e., REEDS_SHEPP_L, REEDS_SHEPP_S, REEDS_SHEPP_R
   * @param length    Single step motion path length
   * @param init_pose Initial pose (x, y, yaw)
   * @return new_pose	New pose (new_x, new_y, new_yaw) after moving
   */
  Pose2d interpolate(int mode, double length, Pose2d init_pose);

  /**
   * @brief Generate the path.
   * @param start Initial pose (x, y, yaw)
   * @param goal  Target pose (x, y, yaw)
   * @return path The smoothed trajectory points
   */
  Points2d generation(Pose2d start, Pose2d goal);

  /**
   * @brief Configure the maximum curvature.
   * @param max_curv  the maximum curvature
   */
  void setMaxCurv(double max_curv);

protected:
  void _calTauOmega(double u, double v, double xi, double eta, double phi, double& tau, double& omega);

  /**
   * @brief Update the best motion mode.
   * @param cur_paths   current generated Reeds-Shepp paths
   * @param best_path   The best generated Reeds-Shepp path so far
   */
  void _update(std::vector<RSPath> cur_paths, RSPath& best_path);

protected:
  double max_curv_;  // The maximum curvature of the curve
};
}  // namespace trajectory_generation

#endif