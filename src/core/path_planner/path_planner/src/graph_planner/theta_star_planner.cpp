/**
 * *********************************************************
 *
 * @file: theta_star.cpp
 * @brief: Contains the Theta* planner class
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
#include <unordered_map>

#include <costmap_2d/cost_values.h>

#include "path_planner/graph_planner/theta_star_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Construct a new ThetaStar object
 * @param costmap the environment for path planning
 */
ThetaStarPathPlanner::ThetaStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros) : PathPlanner(costmap_ros){};

/**
 * @brief Theta* implementation
 * @param start  start node
 * @param goal   goal node
 * @param path   optimal path consists of Node
 * @param expand containing the node been search during the process
 * @return  true if path found, else false
 */
bool ThetaStarPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  // initialize
  Node start_node(start.x(), start.y());
  Node goal_node(goal.x(), goal.y());
  start_node.set_id(grid2Index(start_node.x(), start_node.y()));
  goal_node.set_id(grid2Index(goal_node.x(), goal_node.y()));
  path.clear();
  expand.clear();

  // open list and closed list
  std::priority_queue<Node, std::vector<Node>, Node::compare_cost> open_list;
  std::unordered_map<int, Node> closed_list;

  open_list.push(start_node);

  // main process
  while (!open_list.empty())
  {
    // pop current node from open list
    auto current = open_list.top();
    open_list.pop();

    // current node exist in closed list, continue
    if (closed_list.count(current.id()))
      continue;

    closed_list.insert(std::make_pair(current.id(), current));
    expand.emplace_back(current.x(), current.y());

    // goal found
    if (current == goal_node)
    {
      const auto& backtrace = _convertClosedListToPath<int>(closed_list, start_node, goal_node);
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
      auto node_new = current + m;  // add the x_, y_, g_
      node_new.set_g(current.g() + m.g());
      node_new.set_h(std::hypot(node_new.x() - goal_node.x(), node_new.y() - goal_node.y()));
      node_new.set_id(grid2Index(node_new.x(), node_new.y()));
      node_new.set_pid(current.id());

      // node_new in closed list, continue
      if (closed_list.count(node_new.id()))
        continue;

      // next node hit the boundary or obstacle
      if ((node_new.id() < 0) || (node_new.id() >= map_size_) ||
          (costmap_->getCharMap()[node_new.id()] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
           costmap_->getCharMap()[node_new.id()] >= costmap_->getCharMap()[current.id()]))
        continue;

      node_new.set_h(helper::dist(node_new, goal));

      // path 1: same to a_star
      open_list.push(node_new);

      // path 2: connect current's parent and current's neigbhour directly
      auto current_parent_it = closed_list.find(current.pid());
      if (current_parent_it != closed_list.end())
      {
        auto current_parent = current_parent_it->second;
        if (_lineOfSight(current_parent, node_new))
        {
          node_new.set_g(current_parent.g() + helper::dist(current_parent, node_new));
          node_new.set_pid(current_parent.id());
          open_list.push(node_new);
        }
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
void ThetaStarPathPlanner::_updateVertex(const Node& parent, Node& child)
{
  if (_lineOfSight(parent, child))
  {
    // path 2
    const double dist = std::hypot(parent.x() - child.x(), parent.y() - child.y());
    if (parent.g() + dist < child.g())
    {
      child.set_g(parent.g() + dist);
      child.set_pid(parent.id());
    }
  }
}

/**
 * @brief Bresenham algorithm to check if there is any obstacle between parent and child
 * @param parent
 * @param child
 * @return true if no obstacle, else false
 */
bool ThetaStarPathPlanner::_lineOfSight(const Node& parent, const Node& child)
{
  int dx = node1.x() - node2.x();
  int dy = node1.y() - node2.y();

  int sx = (dx == 0) ? 0 : dx / std::abs(dx);
  int sy = (dy == 0) ? 0 : dy / std::abs(dy);

  dx = std::abs(dx);
  dy = std::abs(dy);

  if (dx > dy)
  {
    int tau = dy - dx;
    int x = node2.x(), y = node2.y(), e = 0;

    while (x != node1.x())
    {
      if (e * 2 > tau)
      {
        x += sx;
        e -= dy;
      }
      else if (e * 2 < tau)
      {
        y += sy;
        e += dx;
      }
      else
      {
        x += sx;
        y += sy;
        e += dx - dy;
      }

      if (costmap_->getCharMap()[grid2Index(x, y)] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
          costmap_->getCharMap()[grid2Index(x, y)] >= costmap_->getCharMap()[node1.id()])
        // obstacle detected
        return false;
    }
  }
  else
  {
    // similar. swap x and y
    int tau = dx - dy;
    int x = node2.x(), y = node2.y(), e = 0;

    while (y != node1.y())
    {
      if (e * 2 > tau)
      {
        y += sy;
        e -= dx;
      }
      else if (e * 2 < tau)
      {
        x += sx;
        e += dy;
      }
      else
      {
        x += sx;
        y += sy;
        e += dy - dx;
      }

      if (costmap_->getCharMap()[grid2Index(x, y)] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
          costmap_->getCharMap()[grid2Index(x, y)] >= costmap_->getCharMap()[node1.id()])
        // obstacle detected
        return false;
    }
  }

  return true;
}
}  // namespace path_planner
}  // namespace rmp
