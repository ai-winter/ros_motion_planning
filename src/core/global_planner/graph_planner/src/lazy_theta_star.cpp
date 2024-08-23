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
 * @param costmap the environment for path planning
 */
LazyThetaStar::LazyThetaStar(costmap_2d::Costmap2D* costmap) : ThetaStar(costmap)
{
  motions_ = Node::getMotion();
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
  path.clear();
  expand.clear();

  // open list and closed list
  std::priority_queue<Node, std::vector<Node>, Node::compare_cost> open_list;
  closed_list_.clear();

  open_list.push(start);

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
    expand.push_back(current);

    // goal found
    if (current == goal)
    {
      path = _convertClosedListToPath(closed_list_, start, goal);
      return true;
    }

    // explore neighbor of current node
    for (const auto& motion : motions_)
    {
      // explore a new node
      auto node_new = current + motion;  // including current.g + motion.g
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
 * @brief check if the parent of vertex need to be updated. if so, update it
 * @param node
 */
void LazyThetaStar::_setVertex(Node& node)
{
  auto parent_it = closed_list_.find(node.pid());
  if (parent_it == closed_list_.end())
    return;

  auto parent = parent_it->second;

  if (!_lineOfSight(parent, node))
  {
    // path 1
    node.set_g(std::numeric_limits<double>::max());
    for (const auto& motion : motions_)
    {
      auto parent_new = node + motion;
      parent_new.set_id(grid2Index(parent_new.x(), parent_new.y()));

      auto parent_new_it = closed_list_.find(parent_new.id());
      if (parent_new_it != closed_list_.end())
      {
        parent_new = parent_new_it->second;
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
