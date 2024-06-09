/**
 * *********************************************************
 *
 * @file: rrt.cpp
 * @brief: Contains the Rapidly-Exploring Random Tree (RRT) planner class
 * @author: Yang Haodong
 * @date: 2022-10-27
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

#include "rrt.h"

namespace global_planner
{
/**
 * @brief  Constructor
 * @param   costmap   the environment for path planning
 * @param   sample_num  andom sample points
 * @param   max_dist    max distance between sample points
 */
RRT::RRT(costmap_2d::Costmap2D* costmap, int sample_num, double max_dist)
  : GlobalPlanner(costmap), sample_num_(sample_num), max_dist_(max_dist)
{
}

/**
 * @brief RRT implementation
 * @param start         start node
 * @param goal          goal node
 * @param path          optimal path consists of Node
 * @param expand        containing the node been search during the process
 * @return  true if path found, else false
 */
bool RRT::plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand)
{
  path.clear();
  expand.clear();

  sample_list_.clear();
  // copy
  start_ = start, goal_ = goal;
  sample_list_.insert(std::make_pair(start.id_, start));
  expand.push_back(start);

  // main loop
  int iteration = 0;
  while (iteration < sample_num_)
  {
    // generate a random node in the map
    Node sample_node = _generateRandomNode();

    // obstacle
    if (costmap_->getCharMap()[sample_node.id_] >= costmap_2d::LETHAL_OBSTACLE * factor_)
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
 * @brief Generates a random node
 * @return Generated node
 */
Node RRT::_generateRandomNode()
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
    return Node(x, y, 0, 0, id, 0);
  }
  else
    return Node(goal_.x_, goal_.y_, 0, 0, goal_.id_, 0);
}

/**
 * @brief Regular the sample node by the nearest node in the sample list
 * @param list  samplee list
 * @param node  sample node
 * @return nearest node
 */
Node RRT::_findNearestPoint(std::unordered_map<int, Node> list, const Node& node)
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
    new_node.x_ = nearest_node.x_ + static_cast<int>(max_dist_ * cos(theta));
    new_node.y_ = nearest_node.y_ + static_cast<int>(max_dist_ * sin(theta));
    new_node.id_ = grid2Index(new_node.x_, new_node.y_);
    new_node.g_ = max_dist_ + nearest_node.g_;
  }

  // obstacle check
  if (_isAnyObstacleInPath(new_node, nearest_node))
    new_node.id_ = -1;

  return new_node;
}

/**
 * @brief Check if there is any obstacle between the 2 nodes.
 * @param n1        Node 1
 * @param n2        Node 2
 * @return bool value of whether obstacle exists between nodes
 */
bool RRT::_isAnyObstacleInPath(const Node& n1, const Node& n2)
{
  double theta = helper::angle(n1, n2);
  double dist_ = helper::dist(n1, n2);

  // distance longer than the threshold
  if (dist_ > max_dist_)
    return true;

  // sample the line between two nodes and check obstacle
  float resolution = costmap_->getResolution();
  int n_step = static_cast<int>(dist_ / resolution);
  for (int i = 0; i < n_step; i++)
  {
    float line_x = (float)n1.x_ + (float)(i * resolution * cos(theta));
    float line_y = (float)n1.y_ + (float)(i * resolution * sin(theta));
    if (costmap_->getCharMap()[grid2Index(static_cast<int>(line_x), static_cast<int>(line_y))] >=
        costmap_2d::LETHAL_OBSTACLE * factor_)
      return true;
  }
  return false;
}

/**
 * @brief Check if goal is reachable from current node
 * @param new_node Current node
 * @return bool value of whether goal is reachable from current node
 */
bool RRT::_checkGoal(const Node& new_node)
{
  auto dist_ = helper::dist(new_node, goal_);
  if (dist_ > max_dist_)
    return false;

  if (!_isAnyObstacleInPath(new_node, goal_))
  {
    Node goal(goal_.x_, goal_.y_, dist_ + new_node.g_, 0, grid2Index(goal_.x_, goal_.y_), new_node.id_);
    sample_list_.insert(std::make_pair(goal.id_, goal));
    return true;
  }
  return false;
}
}  // namespace global_planner
