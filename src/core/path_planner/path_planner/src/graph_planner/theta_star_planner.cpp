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
 * @param costmap   the environment for path planning
 */
ThetaStarPathPlanner::ThetaStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros) : PathPlanner(costmap_ros){};

/**
 * @brief Theta* implementation
 * @param start         start node
 * @param goal          goal node
 * @param path          optimal path consists of Node
 * @param expand        containing the node been search during the process
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

    // current node does not exist in closed list
    if (closed_list.find(current.id()) != closed_list.end())
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

      // current node do not exist in closed list
      if (closed_list.find(node_new.id()) != closed_list.end())
        continue;

      // next node hit the boundary or obstacle
      if ((node_new.id() < 0) || (node_new.id() >= map_size_) ||
          (costmap_->getCharMap()[node_new.id()] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
           costmap_->getCharMap()[node_new.id()] >= costmap_->getCharMap()[current.id()]))
        continue;

      // get the coordinate of parent node
      Node parent;
      parent.set_id(current.pid());
      int tmp_x, tmp_y;
      index2Grid(parent.id(), tmp_x, tmp_y);
      parent.set_x(tmp_x);
      parent.set_y(tmp_y);

      // update g value
      auto find_parent = closed_list.find(parent.id());
      if (find_parent != closed_list.end())
      {
        parent = find_parent->second;
        _updateVertex(parent, node_new);
      }

      open_list.push(node_new);
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
  int s_x = (parent.x() - child.x() == 0) ? 0 : (parent.x() - child.x()) / std::abs(parent.x() - child.x());
  int s_y = (parent.y() - child.y() == 0) ? 0 : (parent.y() - child.y()) / std::abs(parent.y() - child.y());
  int d_x = std::abs(parent.x() - child.x());
  int d_y = std::abs(parent.y() - child.y());

  // check if any obstacle exists between parent and child
  if (d_x > d_y)
  {
    int tau = d_y - d_x;
    int x = child.x(), y = child.y();
    int e = 0;
    while (x != parent.x())
    {
      if (e * 2 > tau)
      {
        x += s_x;
        e -= d_y;
      }
      else if (e * 2 < tau)
      {
        y += s_y;
        e += d_x;
      }
      else
      {
        x += s_x;
        y += s_y;
        e += d_x - d_y;
      }
      if (costmap_->getCharMap()[grid2Index(x, y)] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
          costmap_->getCharMap()[grid2Index(x, y)] >= costmap_->getCharMap()[parent.id()])
        // obstacle detected
        return false;
    }
  }
  else
  {
    // similar. swap x and y
    int tau = d_x - d_y;
    int x = child.x(), y = child.y();
    int e = 0;
    while (y != parent.y())
    {
      if (e * 2 > tau)
      {
        y += s_y;
        e -= d_x;
      }
      else if (e * 2 < tau)
      {
        x += s_x;
        e += d_y;
      }
      else
      {
        x += s_x;
        y += s_y;
        e += d_y - d_x;
      }
      if (costmap_->getCharMap()[grid2Index(x, y)] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
          costmap_->getCharMap()[grid2Index(x, y)] >= costmap_->getCharMap()[parent.id()])
        // obstacle detected
        return false;
    }
  }
  return true;
}
}  // namespace path_planner
}  // namespace rmp
