/**
 * *********************************************************
 *
 * @file: motions.h
 * @brief: Contains the motion model for path planning
 * @author: Yang Haodong
 * @date: 2025-01-17
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_PATH_PLANNER_UTILS_MOTIONS_H_
#define RMP_PATH_PLANNER_UTILS_MOTIONS_H_

#include <vector>

namespace rmp
{
namespace path_planner
{
enum class MotionModel
{
  UNKNOWN = 0,
  GRID2D = 1,
  DUBIN = 2,
  REEDS_SHEPP = 3,
  STATE_LATTICE = 4,
};

/**
 * @brief A struct with the motion primitive's direction embedded
 */
enum class TurnDirection
{
  UNKNOWN = 0,
  FORWARD = 1,
  LEFT = 2,
  RIGHT = 3,
  REVERSE = 4,
  REV_LEFT = 5,
  REV_RIGHT = 6
};

/**
 * @brief A struct for poses in motion primitives
 */
struct MotionPose
{
  /**
   * @brief A constructor for MotionPose
   */
  MotionPose() = default;

  /**
   * @brief A constructor for nav2_smac_planner::MotionPose
   * @param x X pose
   * @param y Y pose
   * @param theta Angle of pose
   * @param TurnDirection Direction of the primitive's turn
   */
  MotionPose(const float& x, const float& y, const float& theta, const TurnDirection& turn_dir)
    : x_(x), y_(y), theta_(theta), turn_dir_(turn_dir)
  {
  }

  MotionPose operator-(const MotionPose& p2)
  {
    return MotionPose(x_ - p2.x_, y_ - p2.y_, theta_ - p2.theta_, TurnDirection::UNKNOWN);
  }

  double x_;
  double y_;
  double theta_;
  TurnDirection turn_dir_;
};

using MotionPoses = std::vector<MotionPose>;

}  // namespace path_planner
}  // namespace rmp
#endif