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
#include "rrt_connect.h"

namespace global_planner
{
/**
 * @brief Construct a new RRTConnect object
 * @param costmap    the environment for path planning
 * @param sample_num andom sample points
 * @param max_dist   max distance between sample points
 */
RRTConnect::RRTConnect(costmap_2d::Costmap2D* costmap, int sample_num, double max_dist)
  : RRT(costmap, sample_num, max_dist)
{
}

/**
 * @brief RRT connect implementation
 * @param start  start node
 * @param goal   goal node
 * @param expand containing the node been search during the process
 * @return  true if path found, else false
 */
bool RRTConnect::plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand)
{
  // clear vector
  path.clear();
  expand.clear();
  sample_list_f_.clear();
  sample_list_b_.clear();

  // copy
  start_ = start, goal_ = goal;
  sample_list_f_.insert(std::make_pair(start.id(), start));
  sample_list_b_.insert(std::make_pair(goal.id(), goal));
  expand.push_back(start);
  expand.push_back(goal);

  // main loop
  int iteration = 0;
  while (iteration < sample_num_)
  {
    // generate a random node in the map
    Node sample_node = _generateRandomNode();

    // regular the sample node
    Node new_node_f = _findNearestPoint(sample_list_f_, sample_node);
    if (new_node_f.id() == -1)
      continue;
    else
    {
      sample_list_f_.insert(std::make_pair(new_node_f.id(), new_node_f));
      expand.push_back(new_node_f);

      // backward exploring
      Node new_node_b = _findNearestPoint(sample_list_b_, new_node_f);
      if (new_node_b.id() == -1)
        continue;

      sample_list_b_.insert(std::make_pair(new_node_b.id(), new_node_b));
      expand.push_back(new_node_b);

      // greedy extending
      while (true)
      {
        double dist = std::min(max_dist_, helper::dist(new_node_f, new_node_b));
        double theta = helper::angle(new_node_b, new_node_f);
        Node new_node_b2;
        new_node_b2.set_x(new_node_b.x() + static_cast<int>(dist * cos(theta)));
        new_node_b2.set_y(new_node_b.y() + static_cast<int>(dist * sin(theta)));
        new_node_b2.set_id(grid2Index(new_node_b2.x(), new_node_b2.y()));
        new_node_b2.set_pid(new_node_b.id());
        new_node_b2.set_g(dist + new_node_b.g());

        if (_isAnyObstacleInPath(new_node_b, new_node_b2))
          break;

        sample_list_b_.insert(std::make_pair(new_node_b2.id(), new_node_b2));
        expand.push_back(new_node_b2);
        new_node_b = new_node_b2;

        // connected -> goal found
        if (new_node_b == new_node_f)
        {
          path = _convertClosedListToPath(new_node_b);
          return true;
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
 * @brief Convert closed list to path
 * @param boundary connected node that the boudary of forward and backward
 * @return vector containing path nodes
 */
std::vector<Node> RRTConnect::_convertClosedListToPath(const Node& boundary)
{
  if (!sample_list_f_.count(start_.id()))
    std::swap(sample_list_f_, sample_list_b_);

  std::vector<Node> path;

  auto current = sample_list_b_.find(boundary.id());
  while (current->second != goal_)
  {
    path.push_back(current->second);

    auto it = sample_list_b_.find(current->second.pid());
    if (it != sample_list_b_.end())
      current = it;
    else
      return {};
  }
  path.push_back(goal_);

  std::reverse(path.begin(), path.end());

  current = sample_list_f_.find(boundary.id());
  while (current->second != start_)
  {
    path.push_back(current->second);

    auto it = sample_list_f_.find(current->second.pid());
    if (it != sample_list_f_.end())
      current = it;
    else
      return {};
  }

  path.push_back(start_);

  return path;
}
}  // namespace global_planner