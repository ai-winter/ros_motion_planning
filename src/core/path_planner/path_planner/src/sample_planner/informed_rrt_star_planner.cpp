/**
 * *********************************************************
 *
 * @file: informed_rrt_star_planner.cpp
 * @brief: Contains the informed RRT* planner class
 * @author: Yang Haodong
 * @date: 2024-09-24
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

#include "common/geometry/collision_checker.h"
#include "path_planner/sample_planner/informed_rrt_star_planner.h"

namespace rmp
{
namespace path_planner
{
namespace
{
using CollisionChecker = rmp::common::geometry::CollisionChecker;
}

/**
 * @brief  Constructor
 * @param   costmap   the environment for path planning
 * @param   obstacle_factor obstacle factor(greater means obstacles)
 * @param   max_dist    max distance between sample points
 */
InformedRRTStarPathPlanner::InformedRRTStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor,
                                                       int sample_num, double max_dist, double r)
  : RRTStarPathPlanner(costmap_ros, obstacle_factor, sample_num, max_dist, r)
{
}

/**
 * @brief Informed RRT* implementation
 * @param start     start node
 * @param goal      goal node
 * @param expand    containing the node been search during the process
 * @return tuple contatining a bool as to whether a path was found, and the path
 */
bool InformedRRTStarPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  double m_start_x, m_start_y, m_goal_x, m_goal_y;
  if ((!validityCheck(start.x(), start.y(), m_start_x, m_start_y)) ||
      (!validityCheck(goal.x(), goal.y(), m_goal_x, m_goal_y)))
  {
    return false;
  }

  // initialization
  c_best_ = std::numeric_limits<double>::max();
  c_min_ = std::hypot(m_start_x - m_goal_x, m_start_y - m_goal_y);
  int best_parent = -1;
  path.clear();
  expand.clear();
  sample_list_.clear();

  // copy
  start_.set_x(m_start_x);
  start_.set_y(m_start_y);
  start_.set_id(grid2Index(start_.x(), start_.y()));
  goal_.set_x(m_goal_x);
  goal_.set_y(m_goal_y);
  goal_.set_id(grid2Index(goal_.x(), goal_.y()));
  sample_list_.insert(std::make_pair(start_.id(), start_));
  expand.emplace_back(m_start_x, m_start_y, 0);

  // main loop
  int iteration = 0;
  while (iteration < sample_num_)
  {
    iteration++;

    // generate a random node in the map
    Node sample_node = _generateRandomNode();
    // obstacle
    if (costmap_->getCharMap()[sample_node.id()] >= costmap_2d::LETHAL_OBSTACLE * obstacle_factor_)
      continue;

    // visited
    if (sample_list_.find(sample_node.id()) != sample_list_.end())
      continue;

    // regular the sample node
    Node new_node = _findNearestPoint(sample_list_, sample_node);
    if (new_node.id() == -1)
    {
      continue;
    }
    else
    {
      sample_list_.insert(std::make_pair(new_node.id(), new_node));
      expand.emplace_back(new_node.x(), new_node.y(), new_node.pid());
    }

    // goal found
    auto isCollision = [&](const Node& node1, const Node& node2) {
      return CollisionChecker::BresenhamCollisionDetection(node1, node2, [&](const Node& node) {
        return costmap_->getCharMap()[grid2Index(node.x(), node.y())] >= costmap_2d::LETHAL_OBSTACLE * obstacle_factor_;
      });
    };

    auto dist_ = std::hypot(new_node.x() - goal_.x(), new_node.y() - goal_.y());
    if (dist_ <= max_dist_ && !isCollision(new_node, goal_))
    {
      double cost = dist_ + new_node.g();
      if (cost < c_best_)
      {
        best_parent = new_node.id();
        c_best_ = cost;
      }
    }
  }

  if (best_parent != -1)
  {
    Node goal_star(goal_.x(), goal_.y(), c_best_, 0, grid2Index(goal_.x(), goal_.y()), best_parent);
    sample_list_.insert(std::make_pair(goal_star.id(), goal_star));

    const auto& backtrace = _convertClosedListToPath<Node>(sample_list_, start_, goal_);
    for (auto iter = backtrace.rbegin(); iter != backtrace.rend(); ++iter)
    {
      // convert to world frame
      double wx, wy;
      costmap_->mapToWorld(iter->x(), iter->y(), wx, wy);
      path.emplace_back(wx, wy);
    }
    return true;
  }

  return false;
}

/**
 * @brief Generates a random node
 * @return Generated node
 */
InformedRRTStarPathPlanner::Node InformedRRTStarPathPlanner::_generateRandomNode()
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
      if (temp.id() < map_size_ - 1)
      {
        return temp;
      }
    }
  }
  else
  {
    return RRTStarPathPlanner::_generateRandomNode();
  }
}

/**
 * @brief Sample in ellipse
 * @param   x   random sampling x
 * @param   y   random sampling y
 * @return ellipse node
 */
InformedRRTStarPathPlanner::Node InformedRRTStarPathPlanner::_transform(double x, double y)
{
  // center
  double center_x = (start_.x() + goal_.x()) / 2;
  double center_y = (start_.y() + goal_.y()) / 2;

  // rotation
  double theta = -std::atan2(goal_.y() - start_.y(), goal_.x() - start_.x());

  // ellipse
  double a = c_best_ / 2.0;
  double c = c_min_ / 2.0;
  double b = std::sqrt(std::max(0.0, a * a - c * c));

  // transform
  int tx = static_cast<int>(a * cos(theta) * x + b * sin(theta) * y + center_x);
  int ty = static_cast<int>(-a * sin(theta) * x + b * cos(theta) * y + center_y);
  int id = grid2Index(tx, ty);
  return Node(tx, ty, 0, 0, id, 0);
}
}  // namespace path_planner
}  // namespace rmp