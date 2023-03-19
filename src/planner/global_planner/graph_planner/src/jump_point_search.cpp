/***********************************************************
 *
 * @file: jump_point_search.cpp
 * @breif: Contains the Jump Point Search(JPS) planner class
 * @author: Yang Haodong
 * @update: 2022-10-27
 * @version: 1.0
 *
 * Copyright (c) 2022， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include "jump_point_search.h"

namespace jps_planner
{
/**
 * @brief  Constructor
 * @param   nx          pixel number in costmap x direction
 * @param   ny          pixel number in costmap y direction
 * @param   resolution  costmap resolution
 */
JumpPointSearch::JumpPointSearch(int nx, int ny, double resolution) : GlobalPlanner(nx, ny, resolution)
{
}

/**
 * @brief Jump Point Search(JPS) implementation
 * @param gloal_costmap     costmap
 * @param start     start node
 * @param goal      goal node
 * @param expand    containing the node been search during the process
 * @return tuple contatining a bool as to whether a path was found, and the path
 */
bool JumpPointSearch::plan(const unsigned char* gloal_costmap, const Node& start, const Node& goal,
                           std::vector<Node>& path, std::vector<Node>& expand)
{
  // copy
  costs_ = gloal_costmap;
  start_ = start, goal_ = goal;

  // open list
  std::priority_queue<Node, std::vector<Node>, compare_cost> open_list;
  open_list.push(start);

  // closed list
  std::unordered_set<Node, NodeIdAsHash, compare_coordinates> closed_list;

  // expand list
  expand.clear();
  expand.push_back(start);

  path.clear();

  // get all possible motions
  std::vector<Node> motions = getMotion();

  // main loop
  while (!open_list.empty())
  {
    // pop current node from open list
    Node current = open_list.top();
    open_list.pop();

    // current node do not exist in closed list
    if (closed_list.find(current) != closed_list.end())
      continue;

    // goal found
    if (current == goal)
    {
      closed_list.insert(current);
      path = _convertClosedListToPath(closed_list, start, goal);
      return true;
    }

    // explore neighbor of current node
    std::vector<Node> jp_list;
    for (const auto& motion : motions)
    {
      Node jp = jump(current, motion);

      // exists and not in CLOSED set
      if (jp.id_ != -1 && closed_list.find(jp) == closed_list.end())
      {
        jp.pid_ = current.id_;
        jp.h_ = std::sqrt(std::pow(jp.x_ - goal.x_, 2) + std::pow(jp.y_ - goal.y_, 2));
        jp_list.push_back(jp);
      }
    }

    for (const auto& jp : jp_list)
    {
      open_list.push(jp);
      expand.push_back(jp);

      // goal found
      if (jp == goal)
        break;
    }

    closed_list.insert(current);
  }
  return false;
}

/**
 * @brief detect whether current node has forced neighbor or not
 * @param point     current node
 * @param motion    the motion that current node executes
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

/**
 * @brief calculate jump node recursively
 * @param point     current node
 * @param motion    the motion that current node executes
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
}  // namespace jps_planner