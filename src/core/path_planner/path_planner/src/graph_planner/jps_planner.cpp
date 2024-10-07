/**
 * *********************************************************
 *
 * @file: jps_planner.cpp
 * @brief: Contains the Jump Point Search(JPS) planner class
 * @author: Yang Haodong
 * @date: 2023-12-14
 * @version: 1.1
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <costmap_2d/cost_values.h>

#include "path_planner/graph_planner/jps_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Constructor
 * @param costmap   the environment for path planning
 */
JPSPathPlanner::JPSPathPlanner(costmap_2d::Costmap2DROS* costmap_ros) : PathPlanner(costmap_ros)
{
}

/**
 * @brief Jump Point Search(JPS) implementation
 * @param start          start node
 * @param goal           goal node
 * @param expand         containing the node been search during the process
 * @return tuple contatining a bool as to whether a path was found, and the path
 */
bool JPSPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  start_.set_x(start.x());
  start_.set_y(start.y());
  start_.set_id(grid2Index(start_.x(), start_.y()));
  goal_.set_x(goal.x());
  goal_.set_y(goal.y());
  goal_.set_id(grid2Index(goal_.x(), goal_.y()));

  // clear vector
  path.clear();
  expand.clear();

  // open list and closed list
  std::priority_queue<Node, std::vector<Node>, Node::compare_cost> open_list;
  std::unordered_map<int, Node> closed_list;

  open_list.push(start_);

  // main loop
  while (!open_list.empty())
  {
    // pop current node from open list
    auto current = open_list.top();
    open_list.pop();

    // current node do not exist in closed list
    if (closed_list.find(current.id()) != closed_list.end())
      continue;

    closed_list.insert(std::make_pair(current.id(), current));
    expand.emplace_back(current.x(), current.y());

    // goal found
    if (current == goal_)
    {
      const auto& backtrace = _convertClosedListToPath<Node>(closed_list, start_, goal_);
      for (auto iter = backtrace.rbegin(); iter != backtrace.rend(); ++iter)
      {
        path.emplace_back(iter->x(), iter->y());
      }
      return true;
    }

    // explore neighbor of current node
    for (const auto& motion : motions)
    {
      auto node_new = jump(current, motion);

      // node_new not exits or in closed list
      if (node_new.id() == -1 || closed_list.find(node_new.id()) != closed_list.end())
        continue;

      node_new.set_pid(current.id());
      open_list.push(node_new);
    }
  }

  return false;
}

/**
 * @brief Calculate jump node recursively
 * @param point  current node
 * @param motion the motion that current node executes
 * @return jump node
 */
JPSPathPlanner::Node JPSPathPlanner::jump(const Node& point, const Node& motion)
{
  auto new_point = point + motion;
  new_point.set_g(point.g() + motion.g());
  new_point.set_id(grid2Index(new_point.x(), new_point.y()));
  new_point.set_pid(point.id());
  new_point.set_h(std::hypot(new_point.x() - goal_.x(), new_point.y() - goal_.y()));

  // next node hit the boundary or obstacle
  if (new_point.id() < 0 || new_point.id() >= map_size_ ||
      costmap_->getCharMap()[new_point.id()] >= costmap_2d::LETHAL_OBSTACLE * factor_)
    return { -1, -1, -1, -1, -1, -1 };

  // goal found
  if (new_point == goal_)
    return new_point;

  // diagonal
  if (motion.x() && motion.y())
  {
    // if exists jump point at horizontal or vertical
    Node x_dir(motion.x(), 0, 1, 0, 0, 0);
    Node y_dir(0, motion.y(), 1, 0, 0, 0);
    if (jump(new_point, x_dir).id() != -1 || jump(new_point, y_dir).id() != -1)
      return new_point;
  }

  // exists forced neighbor
  if (detectForceNeighbor(new_point, motion))
    return new_point;
  else
    return jump(new_point, motion);
}

/**
 * @brief Detect whether current node has forced neighbors
 * @param point  current node
 * @param motion the motion that current node executes
 * @return true if current node has forced neighbor else false
 */
bool JPSPathPlanner::detectForceNeighbor(const Node& point, const Node& motion)
{
  int x = point.x();
  int y = point.y();
  int x_dir = motion.x();
  int y_dir = motion.y();
  auto costs = costmap_->getCharMap();

  // horizontal
  if (x_dir && !y_dir)
  {
    if (costs[grid2Index(x, y + 1)] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
        costs[grid2Index(x + x_dir, y + 1)] < costmap_2d::LETHAL_OBSTACLE * factor_)
      return true;
    if (costs[grid2Index(x, y - 1)] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
        costs[grid2Index(x + x_dir, y - 1)] < costmap_2d::LETHAL_OBSTACLE * factor_)
      return true;
  }

  // vertical
  if (!x_dir && y_dir)
  {
    if (costs[grid2Index(x + 1, y)] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
        costs[grid2Index(x + 1, y + y_dir)] < costmap_2d::LETHAL_OBSTACLE * factor_)
      return true;
    if (costs[grid2Index(x - 1, y)] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
        costs[grid2Index(x - 1, y + y_dir)] < costmap_2d::LETHAL_OBSTACLE * factor_)
      return true;
  }

  // diagonal
  if (x_dir && y_dir)
  {
    if (costs[grid2Index(x - x_dir, y)] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
        costs[grid2Index(x - x_dir, y + y_dir)] < costmap_2d::LETHAL_OBSTACLE * factor_)
      return true;
    if (costs[grid2Index(x, y - y_dir)] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
        costs[grid2Index(x + x_dir, y - y_dir)] < costmap_2d::LETHAL_OBSTACLE * factor_)
      return true;
  }

  return false;
}
}  // namespace path_planner
}  // namespace rmp