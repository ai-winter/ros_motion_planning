/**
 * *********************************************************
 *
 * @file: rrt_planner.cpp
 * @brief: Contains the Rapidly-Exploring Random Tree (RRT) planner class
 * @author: Yang Haodong
 * @date: 2024-9-24
 * @version: 2.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <random>

#include "path_planner/sample_planner/rrt_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Construct a new RRT object
 * @param costmap    the environment for path planning
 * @param sample_num andom sample points
 * @param max_dist   max distance between sample points
 */
RRTPathPlanner::RRTPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, int sample_num, double max_dist)
  : PathPlanner(costmap_ros), sample_num_(sample_num), max_dist_(max_dist)
{
}

/**
 * @brief RRT implementation
 * @param start  start node
 * @param goal   goal node
 * @param path   optimal path consists of Node
 * @param expand containing the node been search during the process
 * @return  true if path found, else false
 */
bool RRTPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  // clear vector
  path.clear();
  expand.clear();
  sample_list_.clear();

  // copy
  start_.set_x(start.x());
  start_.set_y(start.y());
  start_.set_id(grid2Index(start_.x(), start_.y()));
  goal_.set_x(goal.x());
  goal_.set_y(goal.y());
  goal_.set_id(grid2Index(goal_.x(), goal_.y()));
  sample_list_.insert(std::make_pair(start_.id(), start_));
  expand.emplace_back(start.x(), start.y(), 0);

  // main loop
  int iteration = 0;
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
      expand.emplace_back(new_node.x(), new_node.y(), new_node.pid());
    }

    // goal found
    if (_checkGoal(new_node))
    {
      const auto& backtrace = _convertClosedListToPath<int>(sample_list_, start_, goal_);
      for (auto iter = backtrace.rbegin(); iter != backtrace.rend(); ++iter)
      {
        path.emplace_back(iter->x(), iter->y());
      }
      return true;
    }
    iteration++;
  }

  return false;
}

/**
 * @brief Generates a random node
 * @return generated node
 */
RRTPathPlanner::Node RRTPathPlanner::_generateRandomNode()
{
  // obtain a random number from hardware
  std::random_device rd;

  // seed the generator
  std::mt19937 eng(rd());

  // define the range
  std::uniform_real_distribution<float> p(0, 1);

  // heuristic
  if (p(eng) > opti_sample_p_)
  {
    // generate node
    std::uniform_int_distribution<int> distr(0, map_size_ - 1);
    const int id = distr(eng);
    int x, y;
    index2Grid(id, x, y);
    return Node(x, y, 0, 0, id, -1);
  }
  else
    return Node(goal_.x(), goal_.y(), 0, 0, goal_.id(), -1);
}

/**
 * @brief Regular the new node by the nearest node in the sample list
 * @param list sample list
 * @param node sample node
 * @return nearest node
 */
RRTPathPlanner::Node RRTPathPlanner::_findNearestPoint(std::unordered_map<int, Node>& list, const Node& node)
{
  Node nearest_node, new_node(node);
  double min_dist = std::numeric_limits<double>::max();

  for (const auto& p : list)
  {
    // calculate distance
    double new_dist = std::hypot(p.second.x() - new_node.x(), p.second.y() - new_node.y());

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
    double theta = std::atan2(new_node.y() - nearest_node.y(), new_node.x() - nearest_node.x());
    new_node.set_x(nearest_node.x() + static_cast<int>(max_dist_ * cos(theta)));
    new_node.set_y(nearest_node.y() + static_cast<int>(max_dist_ * sin(theta)));
    new_node.set_id(grid2Index(new_node.x(), new_node.y()));
    new_node.set_g(max_dist_ + nearest_node.g());
  }

  // already in tree or collide
  if (list.count(new_node.id()) || _isAnyObstacleInPath(new_node, nearest_node))
    new_node.set_id(-1);

  return new_node;
}

/**
 * @brief Check if there is any obstacle between the 2 nodes.
 * @param n1 Node 1
 * @param n2 Node 2
 * @return bool value of whether obstacle exists between nodes
 */
bool RRTPathPlanner::_isAnyObstacleInPath(const Node& n1, const Node& n2)
{
  double theta = std::atan2(n2.y() - n1.y(), n2.x() - n1.x());
  double dist_ = std::hypot(n1.x() - n2.x(), n1.y() - n2.y());

  // distance longer than the threshold
  if (dist > max_dist_)
    return true;

  // sample the line between two nodes and check obstacle
  float resolution = costmap_->getResolution();
  int n_step = static_cast<int>(dist / resolution);
  for (int i = 0; i < n_step; i++)
  {
    float line_x = static_cast<float>(n1.x() + i * resolution * cos(theta));
    float line_y = static_cast<float>(n1.y() + i * resolution * sin(theta));
    if (costmap_->getCharMap()[grid2Index(static_cast<int>(line_x), static_cast<int>(line_y))] >=
        costmap_2d::LETHAL_OBSTACLE * factor_)
      return true;
  }

  return false;
}

/**
 * @brief Check if goal is reachable from current node
 * @param new_node current node
 * @return bool value of whether goal is reachable from current node
 */
bool RRTPathPlanner::_checkGoal(const Node& new_node)
{
  auto dist_ = std::hypot(new_node.x() - goal_.x(), new_node.y() - goal_.y());
  if (dist_ > max_dist_)
    return false;

  if (!_isAnyObstacleInPath(new_node, goal_))
  {
    Node goal(goal_.x(), goal_.y(), dist + new_node.g(), 0, grid2Index(goal_.x(), goal_.y()), new_node.id());
    sample_list_.insert(std::make_pair(goal.id(), goal));
    return true;
  }

  return false;
}
}  // namespace path_planner
}  // namespace rmp
