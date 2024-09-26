/**
 * *********************************************************
 *
 * @file: lazy_theta_star_planner.cpp
 * @brief: Contains the lazy Theta* planner class
 * @author: Wu Maojia, Yang Haodong
 * @date: 2023-10-01
 * @version: 1.3
 *
 * Copyright (c) 2024, Wu Maojia, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <queue>

#include <costmap_2d/cost_values.h>

#include "path_planner/graph_planner/lazy_theta_star_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Construct a new LazyThetaStar object
 * @param costmap the environment for path planning
 */
LazyThetaStarPathPlanner::LazyThetaStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros)
  : ThetaStarPathPlanner(costmap_ros){};

/**
 * @brief Lazy Theta* implementation
 * @param start         start node
 * @param goal          goal node
 * @param path          optimal path consists of Node
 * @param expand        containing the node been search during the process
 * @return  true if path found, else false
 */
bool LazyThetaStarPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  Node start_node(start.x(), start.y());
  Node goal_node(goal.x(), goal.y());
  start_node.set_id(grid2Index(start_node.x(), start_node.y()));
  goal_node.set_id(grid2Index(goal_node.x(), goal_node.y()));
  // initialize
  path.clear();
  expand.clear();

  // open list and closed list
  std::priority_queue<Node, std::vector<Node>, Node::compare_cost> open_list;
  open_list.push(start_node);

  // main process
  while (!open_list.empty())
  {
    // pop current node from open list
    auto current = open_list.top();
    open_list.pop();

    _setVertex(current);

    if (current.g() >= std::numeric_limits<double>::max())
      continue;

    // current node exist in closed list, continue
    if (closed_list_.count(current.id()))
      continue;

    closed_list_.insert(std::make_pair(current.id(), current));
    expand.emplace_back(current.x(), current.y());

    // goal found
    if (current == goal_node)
    {
      const auto& backtrace = _convertClosedListToPath<int>(closed_list_, start_node, goal_node);
      for (auto iter = backtrace.rbegin(); iter != backtrace.rend(); ++iter)
      {
        path.emplace_back(iter->x(), iter->y());
      }
      return true;
    }

    // explore neighbor of current node
    for (const auto& m : motions)
    {
      // explore a new node
      // path 1
      auto node_new = current + m;  // add the .x(), .y(), g_
      node_new.set_g(current.g() + m.g());
      node_new.set_h(std::hypot(node_new.x() - goal_node.x(), node_new.y() - goal_node.y()));
      node_new.set_id(grid2Index(node_new.x(), node_new.y()));
      node_new.set_pid(current.id());

      // current node do not exist in closed list
      if (closed_list_.find(node_new.id()) != closed_list_.end())
        continue;

      // next node hit the boundary or obstacle
      if ((node_new.id() < 0) || (node_new.id() >= map_size_) ||
          (costmap_->getCharMap()[node_new.id()] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
           costmap_->getCharMap()[node_new.id()] >= costmap_->getCharMap()[current.id()]))
        continue;

      node_new.set_h(helper::dist(node_new, goal));

      // path 1: same to a_star
      open_list.push(node_new);

      // get parent node
      auto current_parent_it = closed_list_.find(current.pid());
      if (current_parent_it != closed_list_.end())
      {
        auto current_parent = current_parent_it->second;
        node_new.set_g(current_parent.g() + helper::dist(current_parent, node_new));
        node_new.set_pid(current_parent.id());
        open_list.push(node_new);
      }
    }
  }

  return false;
}

/**
 * @brief update the g value of child node
 * @param parent
 * @param child
 */
void LazyThetaStarPathPlanner::_updateVertex(const Node& parent, Node& child)
{
  // path 2
  const double dist = std::hypot(parent.x() - child.x(), parent.y() - child.y());
  if (parent.g() + dist < child.g())
  {
    child.set_g(parent.g() + dist);
    child.set_pid(parent.id());
  }
}

/**
 * @brief check if the parent of vertex need to be updated. if so, update it
 * @param node
 */
void LazyThetaStarPathPlanner::_setVertex(Node& node)
{
  auto parent_it = closed_list_.find(node.pid());
  if (parent_it == closed_list_.end())
    return;

  auto parent = parent_it->second;

  if (!_lineOfSight(parent, node))
  {
    // path 1
    node.set_g(std::numeric_limits<double>::max());
    for (const auto& m : motions)
    {
      auto parent_new = node + m;
      parent_new.set_id(grid2Index(parent_new.x(), parent_new.y()));

      auto parent_new_it = closed_list_.find(parent_new.id());
      if (parent_new_it != closed_list_.end())
      {
        // parent_new exists in closed list
        parent_new = find_parent_new->second;
        const double dist = std::hypot(parent_new.x() - node.x(), parent_new.y() - node.y());
        if (parent_new.g() + dist < node.g())
        {
          node.set_g(parent_new.g() + dist);
          node.set_pid(parent_new.id());
        }
      }
    }
  }
}
}  // namespace path_planner
}  // namespace rmp
