/**
 * *********************************************************
 *
 * @file: rrt_star.cpp
 * @brief: Contains the Rapidly-Exploring Random Tree Star(RRT*) planner class
 * @author: Yang Haodong
 * @date: 2022-10-29
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
#include <random>

#include "rrt_star.h"

namespace global_planner
{
/**
 * @brief  Constructor
 * @param   nx          pixel number in costmap x direction
 * @param   ny          pixel number in costmap y direction
 * @param   resolution  costmap resolution
 * @param   sample_num  andom sample points
 * @param   max_dist    max distance between sample points
 * @param   r           optimization radius
 */
RRTStar::RRTStar(int nx, int ny, double resolution, int sample_num, double max_dist, double r)
  : RRT(nx, ny, resolution, sample_num, max_dist), r_(r)
{
}
/**
 * @brief RRT implementation
 * @param global_costmap     costmap
 * @param start     start node
 * @param goal      goal node
 * @param expand    containing the node been search during the process
 * @return tuple contatining a bool as to whether a path was found, and the path
 */
bool RRTStar::plan(const unsigned char* global_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
                   std::vector<Node>& expand)
{
  sample_list_.clear();
  // copy
  start_ = start, goal_ = goal;
  costs_ = global_costmap;
  sample_list_.insert(std::make_pair(start.id_, start));
  expand.push_back(start);

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
    Node new_node = _findNearestPoint(sample_list_, sample_node);
    if (new_node.id_ == -1)
      continue;
    else
    {
      sample_list_.insert(std::make_pair(new_node.id_, new_node));
      expand.push_back(new_node);
    }

    // goal found
    if (_checkGoal(new_node))
    {
      path = _convertClosedListToPath(sample_list_, start, goal);
      return true;
    }

    iteration++;
  }
  return false;
}

/**
 * @brief Regular the new node by the nearest node in the sample list
 * @param list     sample list
 * @param node     sample node
 * @return nearest node
 */
Node RRTStar::_findNearestPoint(std::unordered_map<int, Node> list, Node& node)
{
  Node nearest_node, new_node(node);
  double min_dist = std::numeric_limits<double>::max();

  for (const auto p : list)
  {
    // calculate distance
    double new_dist = helper::dist(p.second, new_node);

    // update nearest node
    if (new_dist < min_dist)
    {
      nearest_node = p.second;
      new_node.pid_ = nearest_node.id_;
      new_node.g_ = new_dist + p.second.g_;
      min_dist = new_dist;
    }
  }

  // distance longer than the threshold
  if (min_dist > max_dist_)
  {
    // connect sample node and nearest node, then move the nearest node
    // forward to sample node with `max_distance` as result
    double theta = helper::angle(nearest_node, new_node);
    new_node.x_ = nearest_node.x_ + (int)(max_dist_ * cos(theta));
    new_node.y_ = nearest_node.y_ + (int)(max_dist_ * sin(theta));
    new_node.id_ = grid2Index(new_node.x_, new_node.y_);
    new_node.g_ = max_dist_ + nearest_node.g_;
  }

  // obstacle check
  if (!_isAnyObstacleInPath(new_node, nearest_node))
  {
    // rewire optimization
    for (auto p : sample_list_)
    {
      // inside the optimization circle
      double new_dist = helper::dist(p.second, new_node);
      if (new_dist < r_)
      {
        double cost = p.second.g_ + new_dist;
        // update new sample node's cost and parent
        if (new_node.g_ > cost)
        {
          if (!_isAnyObstacleInPath(new_node, p.second))
          {
            new_node.pid_ = p.second.id_;
            new_node.g_ = cost;
          }
        }
        else
        {
          // update nodes' cost inside the radius
          cost = new_node.g_ + new_dist;
          if (cost < p.second.g_)
          {
            if (!_isAnyObstacleInPath(new_node, p.second))
            {
              p.second.pid_ = new_node.id_;
              p.second.g_ = cost;
            }
          }
        }
      }
      else
        continue;
    }
  }
  else
    new_node.id_ = -1;
  return new_node;
}
}  // namespace global_planner