/**
 * *********************************************************
 *
 * @file: motion_table.h
 * @brief: Motions for Hybrid A* path planning
 * @author: Yang Haodong
 * @date: 2025-05-04
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_HYBRID_ASTAR_PLANNER_MOTION_TABLE_H_
#define RMP_PATH_PLANNER_GRAPH_PLANNER_HYBRID_ASTAR_PLANNER_MOTION_TABLE_H_

#include <memory>

#include "common/geometry/curve/curve.h"
#include "path_planner/utils/motions.h"

namespace rmp
{
namespace path_planner
{
class HybridAStarMotionTable
{
public:
  /**
   * @brief Constuct the HybridAStarMotionTable object
   */
  HybridAStarMotionTable() = default;

  /**
   * @brief Destroy the HybridAStarMotionTable object
   */
  ~HybridAStarMotionTable() = default;

  /**
   * @brief Retrieve all motion primitives available for a given pose.
   * @param pose The current pose consisting of x, y, and theta (orientation in radians).
   * @return A vector of projected poses (MotionPoses), each representing a possible motion from the current pose.
   */
  MotionPoses getMotionPrimitives(const rmp::common::geometry::Point3d& pose) const;

  /**
   * @brief Converts a quantized orientation bin index to a continuous angle in radians.
   * @param bin_idx The index of the orientation bin (quantized angle).
   * @return The corresponding angle in radians.
   */
  double getAngleFromBin(int bin_idx) const;

  /**
   * @brief Converts a continuous angle in radians to a quantized orientation bin index.
   * @param theta The angle in radians.
   * @return The quantized orientation bin index.
   */
  int getOrientationBin(double theta) const;

  /**
   * @brief Initialize the motion table using Dubins primitives.
   */
  void initDubins(int num_angle_quantization, double min_turning_radius, int map_width, double curve_sample_ratio,
                  double change_penalty, double non_straight_penalty, double reverse_penalty,
                  double retrospective_penalty);

private:
  /**
   * @brief Initializes the motion primitives for the Hybrid A* motion table.
   *        Precomputes the trigonometric values, motion deltas, and travel costs for all motion primitives.
   * @param angle The maximum steering angle (used for certain motion calculations).
   * @param d_dist The distance traveled for each motion primitive.
   */
  void initMotionPrimitives(double angle, double d_dist);

public:
  // common
  int num_angle_quantization;
  double min_turning_radius;
  int map_width;

  // cost
  double change_penalty;
  double non_straight_penalty;
  double reverse_penalty;
  double travel_distance_reward;

  // primitives
  double bin_size;
  MotionPoses projections;
  std::vector<std::vector<double>> delta_xs;
  std::vector<std::vector<double>> delta_ys;
  std::vector<std::pair<double, double>> trig_values;
  std::vector<double> travel_costs;

  std::unique_ptr<rmp::common::geometry::Curve> curve_gen;
};
}  // namespace path_planner
}  // namespace rmp

#endif