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
 * @param costmap   the environment for path planning
 */
LazyThetaStar::LazyThetaStar(costmap_2d::Costmap2D* costmap) : ThetaStar(costmap)
{
  motion_ = Node::getMotion();
};

/**
 * @brief Lazy Theta* implementation
 * @param start         start node
 * @param goal          goal node
 * @param path          optimal path consists of Node
 * @param expand        containing the node been search during the process
 * @return  true if path found, else false
 */
bool LazyThetaStar::plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand)
{
  // initialize
  closed_list_.clear();
  path.clear();
  expand.clear();

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

    if (current.g() >= std::numeric_limits<double>::max())
      continue;

    // current node does not exist in closed list
    if (closed_list_.find(current.id()) != closed_list_.end())
      continue;

    closed_list_.insert(std::make_pair(current.id(), current));
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
      Node node_new = current + m;  // add the .x(), .y(), g_
      node_new.set_h(helper::dist(node_new, goal));
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

      // get parent node
      Node parent;
      parent.set_id(current.pid());
      int tmp_x, tmp_y;
      index2Grid(parent.id(), tmp_x, tmp_y);
      parent.set_x(tmp_x);
      parent.set_y(tmp_y);
      auto find_parent = closed_list_.find(parent.id());
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
  if (parent.g() + helper::dist(parent, child) < child.g())
  {
    child.set_g(parent.g() + helper::dist(parent, child));
    child.set_pid(parent.id());
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
  parent.set_id(node.pid());
  int tmp_x, tmp_y;
  index2Grid(parent.id(), tmp_x, tmp_y);
  parent.set_x(tmp_x);
  parent.set_y(tmp_y);

  // if no parent, no need to check the line of sight
  auto find_parent = closed_list_.find(parent.id());
  if (find_parent == closed_list_.end())
    return;
  parent = find_parent->second;

  if (!_lineOfSight(parent, node))
  {
    // path 1
    node.set_g(std::numeric_limits<double>::max());
    for (const auto& m : motion_)
    {
      Node parent_new = node + m;
      parent_new.set_id(grid2Index(parent_new.x(), parent_new.y()));
      auto find_parent_new = closed_list_.find(parent_new.id());

      if (find_parent_new != closed_list_.end())
      {
        // parent_new exists in closed list
        parent_new = find_parent_new->second;
        if (parent_new.g() + helper::dist(parent_new, node) < node.g())
        {
          node.set_g(parent_new.g() + helper::dist(parent_new, node));
          node.set_pid(parent_new.id());
        }
      }
    }
  }
}

}  // namespace global_planner
