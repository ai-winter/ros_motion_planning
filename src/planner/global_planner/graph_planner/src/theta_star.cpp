/***********************************************************
 *
 * @file: theta_star.cpp
 * @breif: Contains the Theta* planner class
 * @author: Wu Maojia
 * @update: 2023-8-13
 * @version: 1.0
 *
 * Copyright (c) 2023， Wu Maojia
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include "theta_star.h"

#include <unordered_set>

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
  ROS_WARN("ThetaStar::ThetaStar");
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
bool ThetaStar::plan(unsigned char* global_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
                 std::vector<Node>& expand)
{
  ROS_WARN("ThetaStar::plan");
  // initialize and clear vector
  global_costmap_ = global_costmap;
  open_list_.clear();
  path.clear();
  expand.clear();

  // closed list
  std::unordered_set<Node, NodeIdAsHash, compare_coordinates> closed_list;

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
    if (closed_list.find(current) != closed_list.end())
      continue;

    closed_list.insert(current);
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
      ROS_WARN("m.x_ = %d, m.y_ = %d", m.x_, m.y_);
      Node node_new = current + m;

      // current node do not exist in closed list
      if (closed_list.find(node_new) != closed_list.end())
        continue;

      // explore a new node
      node_new.id_ = grid2Index(node_new.x_, node_new.y_);
      node_new.pid_ = current.id_;

      // next node hit the boundary or obstacle
      if ((node_new.id_ < 0) || (node_new.id_ >= ns_) || (global_costmap_[node_new.id_] >= lethal_cost_ * factor_))
        continue;

      // update g value
      Node parent;
      parent.id_ = current.pid_;
      index2Grid(parent.id_, parent.x_, parent.y_);
      auto find_parent = closed_list.find(parent);
      if (find_parent != closed_list.end())
      {
        parent = *find_parent;
        updateVertex(current, parent, node_new);
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
 * @param node
 * @param parent
 * @param child
 */
void ThetaStar::updateVertex(const Node& node, const Node& parent, Node& child){
  ROS_WARN("updateVertex");
  if (lineOfSight(parent, child)){
    // path 2
    if (parent.g_ + getDistance(parent, child) < child.g_){
      child.g_ = parent.g_ + getDistance(parent, child);
      child.pid_ = parent.id_;
      open_list_.push_back(child);
      std::push_heap(open_list_.begin(), open_list_.end(), compare_cost());
    }

  }else{
    // path 1
    if (node.g_ + getDistance(node, child) < child.g_){
      child.g_ = node.g_ + getDistance(node, child);
      child.pid_ = node.id_;
      open_list_.push_back(child);
      std::push_heap(open_list_.begin(), open_list_.end(), compare_cost());
    }
  }
}

/**
 * @brief check if there is any obstacle between parent and child
 * @param parent
 * @param child
 * @return true if no obstacle, else false
 */
bool ThetaStar::lineOfSight(const Node& parent, const Node& child){
  ROS_WARN("lineOfSight");
  int s_x = (parent.x_ - child.x_ == 0)? 0: (parent.x_ - child.x_) / std::abs(parent.x_ - child.x_);
  int s_y = (parent.y_ - child.y_ == 0)? 0: (parent.y_ - child.y_) / std::abs(parent.y_ - child.y_);
  int d_x = std::abs(parent.x_ - child.x_);
  int d_y = std::abs(parent.y_ - child.y_);

  // check if any obstacle exists between parent and child
  if (d_x > d_y){
    double tau = (d_y - d_x) / 2.0;
    int x = child.x_, y = child.y_;
    int e = 0;
    while (x != parent.x_){
      if (e > tau){
        x += s_x;
        e -= d_y;
      }else if (e < tau){
        y += s_y;
        e += d_x;
      }else{
        x += s_x;
        y += s_y;
        e += d_x - d_y;
      }
      if (global_costmap_[grid2Index(x, y)] >= lethal_cost_ * factor_)
        // obstacle detected
        return false;
    }

  }else if (d_x < d_y){
    // similar. swap x and y
    double tau = (d_x - d_y) / 2.0;
    int x = child.x_, y = child.y_;
    int e = 0;
    while (y != parent.y_){
      if (e > tau){
        y += s_y;
        e -= d_x;
      }else if (e < tau){
        x += s_x;
        e += d_y;
      }else{
        x += s_x;
        y += s_y;
        e += d_y - d_x;
      }
      if (global_costmap_[grid2Index(x, y)] >= lethal_cost_ * factor_)
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
double ThetaStar::getDistance(const Node& node, const Node& goal)
{
  return std::hypot(node.x_ - goal.x_, node.y_ - goal.y_);
}
}  // namespace global_planner