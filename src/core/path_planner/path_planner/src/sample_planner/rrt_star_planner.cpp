/**
 * *********************************************************
 *
 * @file: rrt_star_planner.cpp
 * @brief: Contains the Rapidly-Exploring Random Tree Star(RRT*) planner class
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

#include "common/geometry/collision_checker.h"
#include "path_planner/sample_planner/rrt_star_planner.h"

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
 * @param   sample_num  andom sample points
 * @param   max_dist    max distance between sample points
 * @param   r           optimization radius
 */
RRTStarPathPlanner::RRTStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor, int sample_num,
                                       double max_dist, double r)
  : RRTPathPlanner(costmap_ros, obstacle_factor, sample_num, max_dist), r_(r)
{
}
/**
 * @brief RRT implementation
 * @param start     start node
 * @param goal      goal node
 * @param expand    containing the node been search during the process
 * @return tuple contatining a bool as to whether a path was found, and the path
 */
bool RRTStarPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  double m_start_x, m_start_y, m_goal_x, m_goal_y;
  if ((!validityCheck(start.x(), start.y(), m_start_x, m_start_y)) ||
      (!validityCheck(goal.x(), goal.y(), m_goal_x, m_goal_y)))
  {
    return false;
  }

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
  bool optimized = false;
  while (iteration < sample_num_)
  {
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
      continue;
    else
    {
      sample_list_.insert(std::make_pair(new_node.id(), new_node));
      expand.emplace_back(new_node.x(), new_node.y(), new_node.pid());
    }

    // goal found
    if (_checkGoal(new_node))
    {
      path.clear();
      const auto& backtrace = _convertClosedListToPath<Node>(sample_list_, start_, goal_);
      for (auto iter = backtrace.rbegin(); iter != backtrace.rend(); ++iter)
      {
        // convert to world frame
        double wx, wy;
        costmap_->mapToWorld(iter->x(), iter->y(), wx, wy);
        path.emplace_back(wx, wy);
      }
      optimized = true;
    }
    iteration++;
  }

  return optimized;
}

/**
 * @brief Regular the new node by the nearest node in the sample list
 * @param list     sample list
 * @param node     sample node
 * @return nearest node
 */
RRTStarPathPlanner::Node RRTStarPathPlanner::_findNearestPoint(std::unordered_map<int, Node>& list, Node& node)
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

  // obstacle check
  auto isCollision = [&](const Node& node1, const Node& node2) {
    return CollisionChecker::BresenhamCollisionDetection(node1, node2, [&](const Node& node) {
      return costmap_->getCharMap()[grid2Index(node.x(), node.y())] >= costmap_2d::LETHAL_OBSTACLE * obstacle_factor_;
    });
  };

  if (!isCollision(new_node, nearest_node))
  {
    // rewire optimization
    for (auto& p : sample_list_)
    {
      // inside the optimization circle
      double new_dist = std::hypot(p.second.x() - new_node.x(), p.second.y() - new_node.y());
      if (new_dist < r_)
      {
        double cost = p.second.g() + new_dist;
        // update new sample node's cost and parent
        if (new_node.g() > cost)
        {
          if (!isCollision(new_node, p.second))
          {
            new_node.set_pid(p.second.id());
            new_node.set_g(cost);
          }
        }
        else
        {
          // update nodes' cost inside the radius
          cost = new_node.g() + new_dist;
          if (cost < p.second.g())
          {
            if (!isCollision(new_node, p.second))
            {
              p.second.set_pid(new_node.id());
              p.second.set_g(cost);
            }
          }
        }
      }
      else
        continue;
    }
  }
  else
    new_node.set_id(-1);
  return new_node;
}
}  // namespace path_planner
}  // namespace rmp