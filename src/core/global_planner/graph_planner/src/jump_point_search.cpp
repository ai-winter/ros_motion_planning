/**
 * *********************************************************
 *
 * @file: jump_point_search.cpp
 * @brief: Contains the Jump Point Search(JPS) planner class
 * @author: Yang Haodong
 * @date: 2023-12-14
 * @version: 1.1
 *
 * Copyright (c) 2024, Yang Haodong. 
 * All rights reserved.
 * 
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "jump_point_search.h"

namespace global_planner
{
/**
 * @brief Constructor
 * @param nx         pixel number in costmap x direction
 * @param ny         pixel number in costmap y direction
 * @param resolution costmap resolution
 */
JumpPointSearch::JumpPointSearch(int nx, int ny, double resolution) : GlobalPlanner(nx, ny, resolution)
{
}

/**
 * @brief Jump Point Search(JPS) implementation
 * @param global_costmap costmap
 * @param start          start node
 * @param goal           goal node
 * @param expand         containing the node been search during the process
 * @return tuple contatining a bool as to whether a path was found, and the path
 */
bool JumpPointSearch::plan(const unsigned char* global_costmap, const Node& start, const Node& goal,
                           std::vector<Node>& path, std::vector<Node>& expand)
{
  // copy
  costs_ = global_costmap;
  start_ = start, goal_ = goal;

  // clear vector
  path.clear();
  expand.clear();

  // open list and closed list
  std::priority_queue<Node, std::vector<Node>, Node::compare_cost> open_list;
  std::unordered_map<int, Node> closed_list;

  open_list.push(start);

  // get all possible motions
  std::vector<Node> motions = Node::getMotion();

  // main loop
  while (!open_list.empty())
  {
    // pop current node from open list
    Node current = open_list.top();
    open_list.pop();

    // current node do not exist in closed list
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
    for (const auto& motion : motions)
    {
      Node node_new = jump(current, motion);

      // node_new not exits or in closed list
      if (node_new.id_ == -1 || closed_list.find(node_new.id_) != closed_list.end())
        continue;

      node_new.pid_ = current.id_;
      open_list.push(node_new);
    }
  }

  return false;
}

/**
 * @brief Calculate jump node recursively
 * @param point  current node
 * @param motion the motion that current node executes
 * @return jump node
 */
Node JumpPointSearch::jump(const Node& point, const Node& motion)
{
  Node new_point = point + motion;
  new_point.id_ = grid2Index(new_point.x_, new_point.y_);
  new_point.pid_ = point.id_;
  new_point.h_ = std::sqrt(std::pow(new_point.x_ - goal_.x_, 2) + std::pow(new_point.y_ - goal_.y_, 2));

  // next node hit the boundary or obstacle
  if (new_point.id_ < 0 || new_point.id_ >= ns_ || costs_[new_point.id_] >= lethal_cost_ * factor_)
    return Node(-1, -1, -1, -1, -1, -1);

  // goal found
  if (new_point == goal_)
    return new_point;

  // diagonal
  if (motion.x_ && motion.y_)
  {
    // if exists jump point at horizontal or vertical
    Node x_dir = Node(motion.x_, 0, 1, 0, 0, 0);
    Node y_dir = Node(0, motion.y_, 1, 0, 0, 0);
    if (jump(new_point, x_dir).id_ != -1 || jump(new_point, y_dir).id_ != -1)
      return new_point;
  }

  // exists forced neighbor
  if (detectForceNeighbor(new_point, motion))
    return new_point;
  else
    return jump(new_point, motion);
}

/**
 * @brief Detect whether current node has forced neighbors
 * @param point  current node
 * @param motion the motion that current node executes
 * @return true if current node has forced neighbor else false
 */
bool JumpPointSearch::detectForceNeighbor(const Node& point, const Node& motion)
{
  int x = point.x_;
  int y = point.y_;
  int x_dir = motion.x_;
  int y_dir = motion.y_;

  // horizontal
  if (x_dir && !y_dir)
  {
    if (costs_[grid2Index(x, y + 1)] >= lethal_cost_ * factor_ &&
        costs_[grid2Index(x + x_dir, y + 1)] < lethal_cost_ * factor_)
      return true;
    if (costs_[grid2Index(x, y - 1)] >= lethal_cost_ * factor_ &&
        costs_[grid2Index(x + x_dir, y - 1)] < lethal_cost_ * factor_)
      return true;
  }

  // vertical
  if (!x_dir && y_dir)
  {
    if (costs_[grid2Index(x + 1, y)] >= lethal_cost_ * factor_ &&
        costs_[grid2Index(x + 1, y + y_dir)] < lethal_cost_ * factor_)
      return true;
    if (costs_[grid2Index(x - 1, y)] >= lethal_cost_ * factor_ &&
        costs_[grid2Index(x - 1, y + y_dir)] < lethal_cost_ * factor_)
      return true;
  }

  // diagonal
  if (x_dir && y_dir)
  {
    if (costs_[grid2Index(x - x_dir, y)] >= lethal_cost_ * factor_ &&
        costs_[grid2Index(x - x_dir, y + y_dir)] < lethal_cost_ * factor_)
      return true;
    if (costs_[grid2Index(x, y - y_dir)] >= lethal_cost_ * factor_ &&
        costs_[grid2Index(x + x_dir, y - y_dir)] < lethal_cost_ * factor_)
      return true;
  }

  return false;
}

}  // namespace global_planner