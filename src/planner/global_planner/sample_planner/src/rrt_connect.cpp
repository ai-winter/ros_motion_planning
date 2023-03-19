/***********************************************************
 *
 * @file: rrt_connect.cpp
 * @breif: Contains the RRT Connect planner class
 * @author: Yang Haodong
 * @update: 2023-1-18
 * @version: 1.0
 *
 * Copyright (c) 2022， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include <cmath>

#include "rrt_connect.h"

namespace rrt_planner
{
/**
 * @brief  Constructor
 * @param   nx          pixel number in costmap x direction
 * @param   ny          pixel number in costmap y direction
 * @param   sample_num  andom sample points
 * @param   max_dist    max distance between sample points
 */
RRTConnect::RRTConnect(int nx, int ny, double resolution, int sample_num, double max_dist)
  : RRT(nx, ny, resolution, sample_num, max_dist)
{
}

/**
 * @brief RRT-Connect implementation
 * @param gloal_costmap     costmap
 * @param start     start node
 * @param goal      goal node
 * @param expand    containing the node been search during the process
 * @return tuple contatining a bool as to whether a path was found, and the path
 */
bool RRTConnect::plan(const unsigned char* gloal_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
                      std::vector<Node>& expand)
{
  sample_list_f_.clear();
  sample_list_b_.clear();
  // copy
  start_ = start, goal_ = goal;
  costs_ = gloal_costmap;
  sample_list_f_.insert(start);
  sample_list_b_.insert(goal);
  expand.push_back(start);
  expand.push_back(goal);

  // main loop
  int iteration = 0;
  while (iteration < sample_num_)
  {
    // generate a random node in the map
    Node sample_node = _generateRandomNode();

    // obstacle
    if (gloal_costmap[sample_node.id_] >= lethal_cost_ * factor_)
      continue;

    // visited
    if (sample_list_.find(sample_node) != sample_list_.end())
      continue;

    // regular the sample node
    Node new_node = _findNearestPoint(sample_list_f_, sample_node);
    if (new_node.id_ == -1)
      continue;
    else
    {
      sample_list_f_.insert(new_node);
      expand.push_back(new_node);
      // backward exploring
      Node new_node_b = _findNearestPoint(sample_list_b_, new_node);
      if (new_node_b.id_ != -1)
      {
        sample_list_b_.insert(new_node_b);
        expand.push_back(new_node_b);
        // greedy extending
        while (true)
        {
          double dist = std::min(max_dist_, _dist(new_node, new_node_b));
          double theta = _angle(new_node_b, new_node);
          Node new_node_b2;
          new_node_b2.x_ = new_node_b.x_ + (int)(dist * cos(theta));
          new_node_b2.y_ = new_node_b.y_ + (int)(dist * sin(theta));
          new_node_b2.id_ = grid2Index(new_node_b2.x_, new_node_b2.y_);
          new_node_b2.pid_ = new_node_b.id_;
          new_node_b2.g_ = dist + new_node_b.g_;

          if (!_isAnyObstacleInPath(new_node_b, new_node_b2))
          {
            sample_list_b_.insert(new_node_b2);
            expand.push_back(new_node_b2);
            new_node_b = new_node_b2;
          }
          else
            break;

          // connected -> goal found
          if (new_node_b == new_node)
          {
            path = _convertClosedListToPath(new_node_b);
            return true;
          }
        }
      }
    }

    // swap
    if (sample_list_b_.size() < sample_list_f_.size())
      std::swap(sample_list_f_, sample_list_b_);

    iteration++;
  }
  return false;
}

/**
 * @brief convert closed list to path
 * @param boundary  connected node that the boudary of forward and backward
 * @return ector containing path nodes
 */
std::vector<Node> RRTConnect::_convertClosedListToPath(const Node& boundary)
{
  if (sample_list_f_.find(start_) == sample_list_.end())
    std::swap(sample_list_f_, sample_list_b_);

  std::vector<Node> path;

  // backward
  std::vector<Node> path_b;
  auto current = *sample_list_b_.find(boundary);
  while (current != goal_)
  {
    path_b.push_back(current);
    auto it = sample_list_b_.find(Node(current.pid_ % nx_, current.pid_ / nx_, 0, 0, current.pid_));
    if (it != sample_list_b_.end())
    {
      current = *it;
    }
    else
      return {};
  }
  path_b.push_back(goal_);

  // forward
  for (auto rit = path_b.rbegin(); rit != path_b.rend(); rit++)
    path.push_back(*rit);

  current = *sample_list_f_.find(boundary);
  while (current != start_)
  {
    auto it = sample_list_f_.find(Node(current.pid_ % nx_, current.pid_ / nx_, 0, 0, current.pid_));
    if (it != sample_list_f_.end())
    {
      current = *it;
    }
    else
      return {};
    path.push_back(current);
  }

  return path;
}
}  // namespace rrt_planner