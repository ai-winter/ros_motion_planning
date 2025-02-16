/**
 * *********************************************************
 *
 * @file: hybrid_astar_planner.cpp
 * @brief: Contains the Hybrid A* planner class
 * @author: Yang Haodong
 * @date: 2024-01-03
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <iostream>
#include <queue>
#include <unordered_set>

#include "common/math/math_helper.h"
#include "path_planner/graph_planner/hybrid_astar_planner/hybrid_astar_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Construct a new Hybrid A* object
 * @param costmap   the environment for path planning
 * @param obstacle_factor obstacle factor(greater means obstacles)
 */
HybridAStarPathPlanner::HybridAStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor,
                                               const HybridSearchInfo& info)
  : PathPlanner(costmap_ros, obstacle_factor)
{
  MotionModel motion_model;
  if (info.motion_model == 2)
  {
    motion_model = MotionModel::DUBIN;
  }
  else if (info.motion_model == 3)
  {
    motion_model = MotionModel::REEDS_SHEPP;
  }
  else
  {
    R_ERROR << "Invalid motion model for HybridAStarPathPlanner";
  }

  astar_framework_ = std::make_unique<AStarFramework<NodeHybrid>>(motion_model, info);
  astar_framework_->initialize();
}

/**
 * @brief Hybrid A* implementation
 * @param start          start node
 * @param goal           goal node
 * @param path           optimal path consists of Node
 * @param expand         containing the node been search during the process
 * @return true if path found, else false
 */
bool HybridAStarPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  double m_start_x, m_start_y, m_goal_x, m_goal_y;
  if ((!validityCheck(start.x(), start.y(), m_start_x, m_start_y)) ||
      (!validityCheck(goal.x(), goal.y(), m_goal_x, m_goal_y)))
  {
    return false;
  }

  path.clear();
  expand.clear();

  if ((!last_path_.empty()) && (goal_ == goal))
  {
    bool is_collision = false;
    Points3d::iterator closest_iter;
    double min_dist = std::numeric_limits<double>::max();
    for (auto iter = last_path_.begin(); iter != last_path_.end(); ++iter)
    {
      unsigned int mx, my;
      costmap_->worldToMap((*iter).x(), (*iter).y(), mx, my);
      if (collision_checker_->inCollision(grid2Index(mx, my)))
      {
        is_collision = true;
        break;
      }
      double dist = std::hypot((*iter).x() - start.x(), (*iter).y() - start.y());
      if (dist < min_dist)
      {
        min_dist = dist;
        closest_iter = iter;
      }
    }

    if (!is_collision)
    {
      for (auto iter = closest_iter; iter != last_path_.end(); ++iter)
      {
        path.emplace_back((*iter).x(), (*iter).y(), (*iter).theta());
      }
      last_path_ = path;
      return true;
    }
    else
    {
      last_path_.clear();
    }
  }

  goal_ = goal;
  astar_framework_->setCollisionChecker(collision_checker_);
  astar_framework_->setStart({ m_start_x, m_start_y, NodeHybrid::motion_table.getOrientationBin(start.theta()) });
  astar_framework_->setGoal({ m_goal_x, m_goal_y, NodeHybrid::motion_table.getOrientationBin(goal.theta()) });

  Points3d path_in_map, origin_plan, prune_plan;
  if (astar_framework_->createPath(path_in_map, expand))
  {
    for (auto iter = path_in_map.rbegin(); iter != path_in_map.rend(); ++iter)
    {
      // convert to world frame
      double wx, wy;
      costmap_->mapToWorld(iter->x(), iter->y(), wx, wy);
      path.emplace_back(wx, wy, iter->theta());
    }
    last_path_ = path;
    return true;
  }

  return false;
}

}  // namespace path_planner

}  // namespace rmp