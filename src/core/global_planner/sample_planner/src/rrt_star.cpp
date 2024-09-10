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
#include "rrt_star.h"

namespace global_planner
{
/**
 * @brief Construct a new RRTStar object
 * @param costmap    the environment for path planning
 * @param sample_num andom sample points
 * @param max_dist   max distance between sample points
 * @param r          optimization radius
 */
RRTStar::RRTStar(costmap_2d::Costmap2D* costmap, int sample_num, double max_dist, double r)
  : RRT(costmap, sample_num, max_dist), r_(r)
{
}

/**
 * @brief RRT star implementation
 * @param start  start node
 * @param goal   goal node
 * @param expand containing the node been search during the process
 * @return  true if path found, else false
 */
bool RRTStar::plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand)
{
  // clear vector
  path.clear();
  expand.clear();
  sample_list_.clear();

  // copy
  start_ = start, goal_ = goal;
  sample_list_.insert(std::make_pair(start.id(), start));
  expand.push_back(start);

  // main loop
  int iteration = 0;
  bool optimized = false;
  while (iteration < sample_num_)
  {
    // generate a random node in the map
    Node sample_node = _generateRandomNode();

    // regular the sample node
    Node new_node = _findNearestPoint(sample_list_, sample_node);
    if (new_node.id() == -1)
      continue;
    else
    {
      sample_list_.insert(std::make_pair(new_node.id(), new_node));
      expand.push_back(new_node);
    }

    // goal found
    if (_checkGoal(new_node))
    {
      path = _convertClosedListToPath(sample_list_, start, goal);
      optimized = true;
    }

    iteration++;
  }

  return optimized;
}

/**
 * @brief Regular the new node by the nearest node in the sample list
 * @param list sample list
 * @param node sample node
 * @return nearest node
 */
Node RRTStar::_findNearestPoint(std::unordered_map<int, Node>& list, Node& node)
{
  Node nearest_node, new_node(node);
  double min_dist = std::numeric_limits<double>::max();

  for (const auto& p : list)
  {
    // calculate distance
    double new_dist = helper::dist(p.second, new_node);

    // update nearest node
    if (new_dist < min_dist)
    {
      nearest_node = p.second;
      new_node.set_pid(nearest_node.id());
      new_node.set_g(new_dist + p.second.g());
      min_dist = new_dist;
    }
  }

  // distance longer than the threshold
  if (min_dist > max_dist_)
  {
    // connect sample node and nearest node, then move the nearest node
    // forward to sample node with `max_distance` as result
    double theta = helper::angle(nearest_node, new_node);
    new_node.set_x(nearest_node.x() + static_cast<int>(max_dist_ * cos(theta)));
    new_node.set_y(nearest_node.y() + static_cast<int>(max_dist_ * sin(theta)));
    new_node.set_id(grid2Index(new_node.x(), new_node.y()));
    new_node.set_g(max_dist_ + nearest_node.g());
  }

  // already in tree or collide
  if (list.count(new_node.id()) || _isAnyObstacleInPath(new_node, nearest_node))
    new_node.set_id(-1);
  else
  {
    // rewire optimization
    for (auto& p : sample_list_)
    {
      double new_dist = helper::dist(p.second, new_node);

      // outside the optimization circle or collide
      if (new_dist >= r_ || _isAnyObstacleInPath(new_node, p.second))
        continue;

      double cost;

      // update new sample node's cost and parent
      cost = p.second.g() + new_dist;
      if (cost < new_node.g())
      {
        new_node.set_pid(p.second.id());
        new_node.set_g(cost);
      }

      // update nodes' cost inside the radius
      cost = new_node.g() + new_dist;
      if (cost < p.second.g())
      {
        p.second.set_pid(new_node.id());
        p.second.set_g(cost);
      }
    }
  }

  return new_node;
}
}  // namespace global_planner