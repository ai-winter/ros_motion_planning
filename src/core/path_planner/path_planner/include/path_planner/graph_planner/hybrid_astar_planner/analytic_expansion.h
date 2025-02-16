/**
 * *********************************************************
 *
 * @file: analytic_expansion.h
 * @brief: Analytic expansion using geometry methods
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
#ifndef RMP_PATH_PLANNER_UTILS_ANALYTIC_EXPANSION_H_
#define RMP_PATH_PLANNER_UTILS_ANALYTIC_EXPANSION_H_

#include "common/geometry/curve/curve.h"
#include "common/geometry/collision_checker.h"

#include "path_planner/utils/motions.h"

namespace rmp
{
namespace path_planner
{
template <typename NodeT>
class AnalyticExpansion
{
public:
  typedef NodeT* NodePtr;
  typedef typename NodeT::Pose Pose;

  /**
   * @brief Constructor for analytic expansion object
   */
  AnalyticExpansion(const MotionModel& motion_model, const HybridSearchInfo& search_info);
  ~AnalyticExpansion();

  /**
   * @brief Sets the collision checker and costmap to use in expansion validation
   * @param collision_checker Collision checker to use
   */
  void setCollisionChecker(const std::shared_ptr<rmp::common::geometry::CollisionChecker>& collision_checker);

  /**
   * @brief Attempt an analytic path completion
   * @param node The node to start the analytic path from
   * @param goal The goal node to plan to
   * @param getter Gets a node at a set of coordinates
   * @param iterations Iterations to run over
   * @param best_cost Best heuristic cost to propertionally expand more closer to the goal
   * @return Node pointer reference to goal node if successful, else
   * return nullptr
   */
  NodePtr tryAnalyticExpansion(const NodePtr& current_node, const NodePtr& goal_node);

protected:
  MotionModel motion_model_;
  HybridSearchInfo search_info_;
  std::shared_ptr<rmp::common::geometry::CollisionChecker> collision_checker_;
  std::vector<NodePtr> expansions_node_;
};

}  // namespace path_planner
}  // namespace rmp

#endif  // NAV2_SMAC_PLANNER__ANALYTIC_EXPANSION_HPP_
