/**
 * *********************************************************
 *
 * @file: rrt_connect_planner.cpp
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

#include "path_planner/sample_planner/rrt_connect_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Construct a new RRTConnect object
 * @param costmap    the environment for path planning
 * @param sample_num andom sample points
 * @param max_dist   max distance between sample points
 */
RRTConnectPathPlanner::RRTConnectPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, int sample_num, double max_dist)
  : RRTPathPlanner(costmap_ros, sample_num, max_dist)
{
}

/**
 * @brief RRT connect implementation
 * @param start  start node
 * @param goal   goal node
 * @param expand containing the node been search during the process
 * @return  true if path found, else false
 */
bool RRTConnectPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  path.clear();
  expand.clear();
  sample_list_f_.clear();
  sample_list_b_.clear();

  // copy
  start_.set_x(start.x());
  start_.set_y(start.y());
  start_.set_id(grid2Index(start_.x(), start_.y()));
  goal_.set_x(goal.x());
  goal_.set_y(goal.y());
  goal_.set_id(grid2Index(goal_.x(), goal_.y()));
  sample_list_f_.insert(std::make_pair(start_.id(), start_));
  sample_list_b_.insert(std::make_pair(goal_.id(), goal_));
  expand.emplace_back(start.x(), start.y(), 0);
  expand.emplace_back(goal.x(), goal.y(), 0);

  // main loop
  int iteration = 0;
  while (iteration < sample_num_)
  {
    // generate a random node in the map
    Node sample_node = _generateRandomNode();

    // regular the sample node
    Node new_node = _findNearestPoint(sample_list_f_, sample_node);
    if (new_node.id() == -1)
    {
      continue;
    }
    else
    {
      sample_list_f_.insert(std::make_pair(new_node.id(), new_node));
      expand.emplace_back(new_node.x(), new_node.y(), new_node.pid());
      // backward exploring
      Node new_node_b = _findNearestPoint(sample_list_b_, new_node_f);
      if (new_node_b.id() == -1)
        continue;

      sample_list_b_.insert(std::make_pair(new_node_b.id(), new_node_b));
      expand.push_back(new_node_b);

      // greedy extending
      while (true)
      {
        sample_list_b_.insert(std::make_pair(new_node_b.id(), new_node_b));
        expand.emplace_back(new_node_b.x(), new_node_b.y(), new_node_b.pid());
        // greedy extending
        while (true)
        {
          double dist_ = std::min(max_dist_, std::hypot(new_node.x() - new_node_b.x(), new_node.y() - new_node_b.y()));
          double theta = std::atan2(new_node.y() - new_node_b.y(), new_node.x() - new_node_b.x());
          Node new_node_b2;
          new_node_b2.set_x(new_node_b.x() + static_cast<int>(dist_ * cos(theta)));
          new_node_b2.set_y(new_node_b.y() + static_cast<int>(dist_ * sin(theta)));
          new_node_b2.set_id(grid2Index(new_node_b2.x(), new_node_b2.y()));
          new_node_b2.set_pid(new_node_b.id());
          new_node_b2.set_g(dist_ + new_node_b.g());

          if (!_isAnyObstacleInPath(new_node_b, new_node_b2))
          {
            sample_list_b_.insert(std::make_pair(new_node_b2.id(), new_node_b2));
            expand.emplace_back(new_node_b2.x(), new_node_b2.y(), new_node_b2.pid());
            new_node_b = new_node_b2;
          }
          else
            break;

          // connected -> goal found
          if (new_node_b == new_node)
          {
            const auto& backtrace = _convertClosedListToPath(new_node_b);
            for (auto iter = backtrace.rbegin(); iter != backtrace.rend(); ++iter)
            {
              path.emplace_back(iter->x(), iter->y());
            }
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
 * @brief Convert closed list to path
 * @param boundary connected node that the boudary of forward and backward
 * @return vector containing path nodes
 */
std::vector<RRTConnectPathPlanner::Node> RRTConnectPathPlanner::_convertClosedListToPath(const Node& boundary)
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
    {
      return {};
    }
    path.push_back(current->second);
  }

  path.push_back(start_);

  return path;
}
}  // namespace path_planner
}  // namespace rmp