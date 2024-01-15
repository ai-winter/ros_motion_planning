/**
 * *********************************************************
 *
 * @file: rrt_connect.cpp
 * @brief: Contains the RRT Connect planner class
 * @author: Yang Haodong
 * @date: 2023-01-18
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong. 
 * All rights reserved.
 * 
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <cmath>

#include "rrt_connect.h"

namespace global_planner
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
 * @param global_costmap     costmap
 * @param start     start node
 * @param goal      goal node
 * @param expand    containing the node been search during the process
 * @return tuple contatining a bool as to whether a path was found, and the path
 */
bool RRTConnect::plan(const unsigned char* global_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
                      std::vector<Node>& expand)
{
  sample_list_f_.clear();
  sample_list_b_.clear();
  // copy
  start_ = start, goal_ = goal;
  costs_ = global_costmap;
  sample_list_f_.insert(std::make_pair(start.id_, start));
  sample_list_b_.insert(std::make_pair(goal.id_, goal));
  expand.push_back(start);
  expand.push_back(goal);

  // main loop
  int iteration = 0;
  while (iteration < sample_num_)
  {
    // generate a random node in the map
    Node sample_node = _generateRandomNode();

    // obstacle
    if (global_costmap[sample_node.id_] >= lethal_cost_ * factor_)
      continue;

    // visited
    if (sample_list_.find(sample_node.id_) != sample_list_.end())
      continue;

    // regular the sample node
    Node new_node = _findNearestPoint(sample_list_f_, sample_node);
    if (new_node.id_ == -1)
      continue;
    else
    {
      sample_list_f_.insert(std::make_pair(new_node.id_, new_node));
      expand.push_back(new_node);
      // backward exploring
      Node new_node_b = _findNearestPoint(sample_list_b_, new_node);
      if (new_node_b.id_ != -1)
      {
        sample_list_b_.insert(std::make_pair(new_node_b.id_, new_node_b));
        expand.push_back(new_node_b);
        // greedy extending
        while (true)
        {
          double dist_ = std::min(max_dist_, helper::dist(new_node, new_node_b));
          double theta = helper::angle(new_node_b, new_node);
          Node new_node_b2;
          new_node_b2.x_ = new_node_b.x_ + (int)(dist_ * cos(theta));
          new_node_b2.y_ = new_node_b.y_ + (int)(dist_ * sin(theta));
          new_node_b2.id_ = grid2Index(new_node_b2.x_, new_node_b2.y_);
          new_node_b2.pid_ = new_node_b.id_;
          new_node_b2.g_ = dist_ + new_node_b.g_;

          if (!_isAnyObstacleInPath(new_node_b, new_node_b2))
          {
            sample_list_b_.insert(std::make_pair(new_node_b2.id_, new_node_b2));
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
  if (sample_list_f_.find(start_.id_) == sample_list_.end())
    std::swap(sample_list_f_, sample_list_b_);

  std::vector<Node> path;

  // backward
  std::vector<Node> path_b;
  auto current = sample_list_b_.find(boundary.id_);
  while (current->second != goal_)
  {
    path_b.push_back(current->second);
    auto it = sample_list_b_.find(current->second.pid_);
    if (it != sample_list_b_.end())
      current = it;
    else
      return {};
  }
  path_b.push_back(goal_);

  // forward
  for (auto rit = path_b.rbegin(); rit != path_b.rend(); rit++)
    path.push_back(*rit);

  current = sample_list_f_.find(boundary.id_);
  while (current->second != start_)
  {
    auto it = sample_list_f_.find(current->second.pid_);
    if (it != sample_list_f_.end())
    {
      current = it;
    }
    else
      return {};
    path.push_back(current->second);
  }

  return path;
}
}  // namespace global_planner