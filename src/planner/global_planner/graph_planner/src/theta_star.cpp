/***********************************************************
 *
 * @file: theta_star.cpp
 * @breif: Contains the Theta* planner class
 * @author: Wu Maojia
 * @update: 2023-8-14
 * @version: 1.0
 *
 * Copyright (c) 2023， Wu Maojia
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
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
  open_list_.clear();
  closed_list_.clear();
  path.clear();
  expand.clear();

  // closed list

  open_list_.push_back(start);
  std::push_heap(open_list_.begin(), open_list_.end(), compare_cost());

  // get all possible motions
  const std::vector<Node> motion = getMotion();

  // main process
  while (1)
  {
    if (open_list_.empty())
      break;

    // pop current node from open list
    Node current = open_list_.front();
    std::pop_heap(open_list_.begin(), open_list_.end(), compare_cost());
    open_list_.pop_back();

    // current node does not exist in closed list
    if (closed_list_.find(current) != closed_list_.end())
      continue;

    closed_list_.insert(current);
    expand.push_back(current);

    // goal found
    if (current == goal)
    {
      path = _convertClosedListToPath(closed_list_, start, goal);
      return true;
    }

    // explore neighbor of current node
    for (const auto& m : motion)
    {
      Node node_new = current + m;  // add the x_, y_, g_

      // current node do not exist in closed list
      if (closed_list_.find(node_new) != closed_list_.end())
        continue;

      // explore a new node
      // path 1
      node_new.h_ = _getDistance(node_new, goal);
      node_new.id_ = grid2Index(node_new.x_, node_new.y_);
      node_new.pid_ = current.id_;

      // next node hit the boundary or obstacle
      if ((node_new.id_ < 0) || (node_new.id_ >= ns_) || (costs_[node_new.id_] >= lethal_cost_ * factor_))
        continue;

      // get the coordinate of parent node
      Node parent;
      parent.id_ = current.pid_;
      index2Grid(parent.id_, parent.x_, parent.y_);

      // update g value
      auto find_parent = closed_list_.find(parent);
      if (find_parent != closed_list_.end())
      {
        parent = *find_parent;
        _updateVertex(parent, node_new);
      }

      // add node to open list
      open_list_.push_back(node_new);
      std::push_heap(open_list_.begin(), open_list_.end(), compare_cost());
    }
  }

  return false;
}

/**
 * @brief update the g value of child node
 * @param parent
 * @param child
 */
void ThetaStar::_updateVertex(const Node& parent, Node& child){
  if (_lineOfSight(parent, child)){
    // path 2
    if (parent.g_ + _getDistance(parent, child) < child.g_){
      child.g_ = parent.g_ + _getDistance(parent, child);
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
bool ThetaStar::_lineOfSight(const Node& parent, const Node& child){
  int s_x = (parent.x_ - child.x_ == 0)? 0: (parent.x_ - child.x_) / std::abs(parent.x_ - child.x_);
  int s_y = (parent.y_ - child.y_ == 0)? 0: (parent.y_ - child.y_) / std::abs(parent.y_ - child.y_);
  int d_x = std::abs(parent.x_ - child.x_);
  int d_y = std::abs(parent.y_ - child.y_);

  // check if any obstacle exists between parent and child
  if (d_x > d_y){
    int tau = d_y - d_x;
    int x = child.x_, y = child.y_;
    int e = 0;
    while (x != parent.x_){
      if (e * 2 > tau){
        x += s_x;
        e -= d_y;
      }else if (e * 2 < tau){
        y += s_y;
        e += d_x;
      }else{
        x += s_x;
        y += s_y;
        e += d_x - d_y;
      }
      if (costs_[grid2Index(x, y)] >= lethal_cost_ * factor_)
        // obstacle detected
        return false;
    }

  }else{
    // similar. swap x and y
    int tau = d_x - d_y;
    int x = child.x_, y = child.y_;
    int e = 0;
    while (y != parent.y_){
      if (e * 2 > tau){
        y += s_y;
        e -= d_x;
      }else if (e * 2 < tau){
        x += s_x;
        e += d_y;
      }else{
        x += s_x;
        y += s_y;
        e += d_y - d_x;
      }
      if (costs_[grid2Index(x, y)] >= lethal_cost_ * factor_)
        // obstacle detected
        return false;
    }
  }
  return true;
}

/**
 * @brief Get the Euclidean distance between two nodes
 * @param node  current node
 * @param goal  goal node
 * @return  Euclidean distance
 */
double ThetaStar::_getDistance(const Node& node, const Node& goal)
{
  return std::hypot(node.x_ - goal.x_, node.y_ - goal.y_);
}
}  // namespace global_planner
