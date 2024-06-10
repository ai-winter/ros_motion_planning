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
 * @param costmap   the environment for path planning
 */
ThetaStar::ThetaStar(costmap_2d::Costmap2D* costmap) : GlobalPlanner(costmap){};

/**
 * @brief Theta* implementation
 * @param start         start node
 * @param goal          goal node
 * @param path          optimal path consists of Node
 * @param expand        containing the node been search during the process
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
  const std::vector<Node> motion = Node::getMotion();

  // main process
  while (!open_list.empty())
  {
    // pop current node from open list
    Node current = open_list.top();
    open_list.pop();

    // current node does not exist in closed list
    if (closed_list.find(current.id()) != closed_list.end())
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
    for (const auto& m : motion)
    {
      // explore a new node
      // path 1
      Node node_new = current + m;  // add the x_, y_, g_
      node_new.set_h(helper::dist(node_new, goal));
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
void ThetaStar::_updateVertex(const Node& parent, Node& child)
{
  if (_lineOfSight(parent, child))
  {
    // path 2
    if (parent.g() + helper::dist(parent, child) < child.g())
    {
      child.set_g(parent.g() + helper::dist(parent, child));
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
bool ThetaStar::_lineOfSight(const Node& parent, const Node& child)
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

}  // namespace global_planner
