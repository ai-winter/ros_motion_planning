/***********************************************************
 *
 * @file: informed_rrt.cpp
 * @breif: Contains the informed RRT* planner class
 * @author: Yang Haodong
 * @update: 2023-1-19
 * @version: 1.0
 *
 * Copyright (c) 2023， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include <cmath>
#include <random>

#include "informed_rrt.h"

namespace rrt_planner
{
/**
 * @brief  Constructor
 * @param   nx          pixel number in costmap x direction
 * @param   ny          pixel number in costmap y direction
 * @param   sample_num  andom sample points
 * @param   max_dist    max distance between sample points
 */
InformedRRT::InformedRRT(int nx, int ny, double resolution, int sample_num, double max_dist, double r)
  : RRTStar(nx, ny, resolution, sample_num, max_dist, r)
{
}

/**
 * @brief Informed RRT* implementation
 * @param gloal_costmap     costmap
 * @param start     start node
 * @param goal      goal node
 * @param expand    containing the node been search during the process
 * @return tuple contatining a bool as to whether a path was found, and the path
 */
bool InformedRRT::plan(const unsigned char* gloal_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
                       std::vector<Node>& expand)
{
  // initialization
  c_best_ = std::numeric_limits<double>::max();
  c_min_ = _dist(start, goal);
  int best_parent = -1;
  sample_list_.clear();

  // copy
  start_ = start, goal_ = goal;
  costs_ = gloal_costmap;
  sample_list_.insert(start);
  expand.push_back(start);

  // main loop
  int iteration = 0;
  while (iteration < sample_num_)
  {
    iteration++;

    // generate a random node in the map
    Node sample_node = _generateRandomNode();

    // obstacle
    if (gloal_costmap[sample_node.id_] >= lethal_cost_ * factor_)
      continue;

    // visited
    if (sample_list_.find(sample_node) != sample_list_.end())
      continue;

    // regular the sample node
    Node new_node = _findNearestPoint(sample_list_, sample_node);
    if (new_node.id_ == -1)
      continue;
    else
    {
      sample_list_.insert(new_node);
      expand.push_back(new_node);
    }

    // goal found
    auto dist = _dist(new_node, goal_);
    if (dist <= max_dist_ && !_isAnyObstacleInPath(new_node, goal_))
    {
      double cost = dist + new_node.g_;
      if (cost < c_best_)
      {
        best_parent = new_node.id_;
        c_best_ = cost;
      }
    }
  }

  if (best_parent != -1)
  {
    Node goal_(goal_.x_, goal_.y_, c_best_, 0, grid2Index(goal_.x_, goal_.y_),
               best_parent);
    sample_list_.insert(goal_);
    path = _convertClosedListToPath(sample_list_, start, goal);
    return true;
  }
  return false;
}

/**
 * @brief Generates a random node
 * @return Generated node
 */
Node InformedRRT::_generateRandomNode()
{
  // ellipse sample
  if (c_best_ < std::numeric_limits<double>::max())
  {
    while (true)
    {
      // unit ball sample
      double x, y;
      std::random_device rd;
      std::mt19937 eng(rd());
      std::uniform_real_distribution<float> p(-1, 1);
      while (true)
      {
        x = p(eng);
        y = p(eng);
        if (x * x + y * y < 1)
          break;
      }
      // transform to ellipse
      Node temp = _transform(x, y);
      if (temp.id_ < ns_ - 1)
        return temp;
    }
  }
  else
    return RRTStar::_generateRandomNode();
}

/**
 * @brief Sample in ellipse
 * @param   x   random sampling x
 * @param   y   random sampling y
 * @return ellipse node
 */
Node InformedRRT::_transform(double x, double y)
{
  // center
  double center_x = (start_.x_ + goal_.x_) / 2;
  double center_y = (start_.y_ + goal_.y_) / 2;

  // rotation
  double theta = -_angle(start_, goal_);

  // ellipse
  double a = c_best_ / 2.0;
  double c = c_min_ / 2.0;
  double b = std::sqrt(a * a - c * c);

  // transform
  int tx = (int)(a * cos(theta) * x + b * sin(theta) * y + center_x);
  int ty = (int)(-a * sin(theta) * x + b * cos(theta) * y + center_y);
  int id = grid2Index(tx, ty);
  return Node(tx, ty, 0, 0, id, 0);
}
}  // namespace rrt_planner