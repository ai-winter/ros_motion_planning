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
#include "theta_star.h"

namespace global_planner
{
/**
 * @brief Construct a new ThetaStar object
 * @param costmap the environment for path planning
 */
ThetaStar::ThetaStar(costmap_2d::Costmap2D* costmap) : GlobalPlanner(costmap) {};

/**
 * @brief Theta* implementation
 * @param start  start node
 * @param goal   goal node
 * @param path   optimal path consists of Node
 * @param expand containing the node been search during the process
 * @return  true if path found, else false
 */
bool ThetaStar::plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand)
{
  // initialize
  path.clear();
  expand.clear();

  // open list and closed list
  std::priority_queue<Node, std::vector<Node>, Node::compare_cost> open_list;
  std::unordered_map<int, Node> closed_list;

  open_list.push(start);

  // get all possible motions
  const std::vector<Node> motions = Node::getMotion();

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
    expand.push_back(current);

    // goal found
    if (current == goal)
    {
      path = _convertClosedListToPath(closed_list, start, goal);
      return true;
    }

    // explore neighbor of current node
    for (const auto& motion : motions)
    {
      // explore a new node
      auto node_new = current + motion;  // including current.g + motion.g
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
 * @brief Bresenham algorithm to check if there is any obstacle between node1 and node2
 * @param node1 node1
 * @param node2 node2
 * @return true if no obstacle, else false
 */
bool ThetaStar::_lineOfSight(const Node& node1, const Node& node2)
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
}  // namespace global_planner
