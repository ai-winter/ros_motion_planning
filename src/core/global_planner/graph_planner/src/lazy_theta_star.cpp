/**
 * *********************************************************
 *
 * @file: lazy_theta_star.cpp
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
#include "lazy_theta_star.h"

namespace global_planner
{
/**
 * @brief Construct a new LazyThetaStar object
 * @param nx          pixel number in costmap x direction
 * @param ny          pixel number in costmap y direction
 * @param resolution  costmap resolution
 */
LazyThetaStar::LazyThetaStar(int nx, int ny, double resolution) : ThetaStar(nx, ny, resolution)
{
  factor_ = 0.35;
};

/**
 * @brief Lazy Theta* implementation
 * @param global_costmap global costmap
 * @param start         start node
 * @param goal          goal node
 * @param path          optimal path consists of Node
 * @param expand        containing the node been search during the process
 * @return  true if path found, else false
 */
bool LazyThetaStar::plan(const unsigned char* global_costmap, const Node& start, const Node& goal,
                         std::vector<Node>& path, std::vector<Node>& expand)
{
  // initialize
  costs_ = global_costmap;
  closed_list_.clear();
  path.clear();
  expand.clear();
  motion_ = Node::getMotion();

  // push the start node into open list
  std::priority_queue<Node, std::vector<Node>, Node::compare_cost> open_list;
  open_list.push(start);

  // main process
  while (!open_list.empty())
  {
    // pop current node from open list
    Node current = open_list.top();
    open_list.pop();

    _setVertex(current);

    if (current.g_ >= INFINITE_COST)
      continue;

    // current node does not exist in closed list
    if (closed_list_.find(current.id_) != closed_list_.end())
      continue;

    closed_list_.insert(std::make_pair(current.id_, current));
    expand.push_back(current);

    // goal found
    if (current == goal)
    {
      path = _convertClosedListToPath(closed_list_, start, goal);
      return true;
    }

    // explore neighbor of current node
    for (const auto& m : motion_)
    {
      // explore a new node
      // path 1
      Node node_new = current + m;  // add the x_, y_, g_
      node_new.h_ = helper::dist(node_new, goal);
      node_new.id_ = grid2Index(node_new.x_, node_new.y_);
      node_new.pid_ = current.id_;

      // current node do not exist in closed list
      if (closed_list_.find(node_new.id_) != closed_list_.end())
        continue;

      // next node hit the boundary or obstacle
      if ((node_new.id_ < 0) || (node_new.id_ >= ns_) ||
          (costs_[node_new.id_] >= lethal_cost_ * factor_ && costs_[node_new.id_] >= costs_[current.id_]))
        continue;

      // get parent node
      Node parent;
      parent.id_ = current.pid_;
      index2Grid(parent.id_, parent.x_, parent.y_);
      auto find_parent = closed_list_.find(parent.id_);
      if (find_parent != closed_list_.end())
      {
        parent = find_parent->second;
        // path 2
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
void LazyThetaStar::_updateVertex(const Node& parent, Node& child)
{
  // path 2
  if (parent.g_ + helper::dist(parent, child) < child.g_)
  {
    child.g_ = parent.g_ + helper::dist(parent, child);
    child.pid_ = parent.id_;
  }
}

/**
 * @brief check if the parent of vertex need to be updated. if so, update it
 * @param node
 */
void LazyThetaStar::_setVertex(Node& node)
{
  // get the coordinate of parent node
  Node parent;
  parent.id_ = node.pid_;
  index2Grid(parent.id_, parent.x_, parent.y_);

  // if no parent, no need to check the line of sight
  auto find_parent = closed_list_.find(parent.id_);
  if (find_parent == closed_list_.end())
    return;
  parent = find_parent->second;

  if (!_lineOfSight(parent, node))
  {
    // path 1
    node.g_ = INFINITE_COST;
    for (const auto& m : motion_)
    {
      Node parent_new = node + m;
      parent_new.id_ = grid2Index(parent_new.x_, parent_new.y_);
      auto find_parent_new = closed_list_.find(parent_new.id_);

      if (find_parent_new != closed_list_.end())
      {
        // parent_new exists in closed list
        parent_new = find_parent_new->second;
        if (parent_new.g_ + helper::dist(parent_new, node) < node.g_)
        {
          node.g_ = parent_new.g_ + helper::dist(parent_new, node);
          node.pid_ = parent_new.id_;
        }
      }
    }
  }
}

}  // namespace global_planner
