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
 * @param nx          pixel number in costmap x direction
 * @param ny          pixel number in costmap y direction
 * @param resolution  costmap resolution
 */
ThetaStar::ThetaStar(int nx, int ny, double resolution) : GlobalPlanner(nx, ny, resolution)
{
  factor_ = 0.35;
};

/**
 * @brief Theta* implementation
 * @param global_costmap global costmap
 * @param start         start node
 * @param goal          goal node
 * @param path          optimal path consists of Node
 * @param expand        containing the node been search during the process
 * @return  true if path found, else false
 */
bool ThetaStar::plan(const unsigned char* global_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
                     std::vector<Node>& expand)
{
  // initialize
  costs_ = global_costmap;
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
    if (closed_list.find(current.id_) != closed_list.end())
      continue;

    closed_list.insert(std::make_pair(current.id_, current));
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
      node_new.h_ = helper::dist(node_new, goal);
      node_new.id_ = grid2Index(node_new.x_, node_new.y_);
      node_new.pid_ = current.id_;

      // current node do not exist in closed list
      if (closed_list.find(node_new.id_) != closed_list.end())
        continue;

      // next node hit the boundary or obstacle
      if ((node_new.id_ < 0) || (node_new.id_ >= ns_) ||
          (costs_[node_new.id_] >= lethal_cost_ * factor_ && costs_[node_new.id_] >= costs_[current.id_]))
        continue;

      // get the coordinate of parent node
      Node parent;
      parent.id_ = current.pid_;
      index2Grid(parent.id_, parent.x_, parent.y_);

      // update g value
      auto find_parent = closed_list.find(parent.id_);
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
    if (parent.g_ + helper::dist(parent, child) < child.g_)
    {
      child.g_ = parent.g_ + helper::dist(parent, child);
      child.pid_ = parent.id_;
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
  int s_x = (parent.x_ - child.x_ == 0) ? 0 : (parent.x_ - child.x_) / std::abs(parent.x_ - child.x_);
  int s_y = (parent.y_ - child.y_ == 0) ? 0 : (parent.y_ - child.y_) / std::abs(parent.y_ - child.y_);
  int d_x = std::abs(parent.x_ - child.x_);
  int d_y = std::abs(parent.y_ - child.y_);

  // check if any obstacle exists between parent and child
  if (d_x > d_y)
  {
    int tau = d_y - d_x;
    int x = child.x_, y = child.y_;
    int e = 0;
    while (x != parent.x_)
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
      if (costs_[grid2Index(x, y)] >= lethal_cost_ * factor_ && costs_[grid2Index(x, y)] >= costs_[parent.id_])
        // obstacle detected
        return false;
    }
  }
  else
  {
    // similar. swap x and y
    int tau = d_x - d_y;
    int x = child.x_, y = child.y_;
    int e = 0;
    while (y != parent.y_)
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
      if (costs_[grid2Index(x, y)] >= lethal_cost_ * factor_ && costs_[grid2Index(x, y)] >= costs_[parent.id_])
        // obstacle detected
        return false;
    }
  }
  return true;
}

}  // namespace global_planner
