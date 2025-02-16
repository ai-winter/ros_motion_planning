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
#include "common/geometry/collision_checker.h"
#include "path_planner/sample_planner/rrt_connect_planner.h"

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
 */
RRTConnectPathPlanner::RRTConnectPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor,
                                             int sample_num, double max_dist)
  : RRTPathPlanner(costmap_ros, obstacle_factor, sample_num, max_dist)
{
}

/**
 * @brief RRT-Connect implementation
 * @param start     start node
 * @param goal      goal node
 * @param expand    containing the node been search during the process
 * @return tuple contatining a bool as to whether a path was found, and the path
 */
bool RRTConnectPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  double m_start_x, m_start_y, m_goal_x, m_goal_y;
  if ((!validityCheck(start.x(), start.y(), m_start_x, m_start_y)) ||
      (!validityCheck(goal.x(), goal.y(), m_goal_x, m_goal_y)))
  {
    return false;
  }

  path.clear();
  expand.clear();
  sample_list_f_.clear();
  sample_list_b_.clear();

  // copy
  start_.set_x(m_start_x);
  start_.set_y(m_start_y);
  start_.set_id(grid2Index(start_.x(), start_.y()));
  goal_.set_x(m_goal_x);
  goal_.set_y(m_goal_y);
  goal_.set_id(grid2Index(goal_.x(), goal_.y()));
  sample_list_f_.insert(std::make_pair(start_.id(), start_));
  sample_list_b_.insert(std::make_pair(goal_.id(), goal_));
  expand.emplace_back(m_start_x, m_start_y, 0);
  expand.emplace_back(m_goal_x, m_goal_y, 0);

  // main loop
  int iteration = 0;
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
      Node new_node_b = _findNearestPoint(sample_list_b_, new_node);
      if (new_node_b.id() != -1)
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

          auto isCollision = [&](const Node& node1, const Node& node2) {
            return CollisionChecker::BresenhamCollisionDetection(node1, node2, [&](const Node& node) {
              return costmap_->getCharMap()[grid2Index(node.x(), node.y())] >=
                     costmap_2d::LETHAL_OBSTACLE * obstacle_factor_;
            });
          };

          if (!isCollision(new_node_b, new_node_b2))
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
            const auto& backtrace =
                _convertBiClosedListToPath<Node>(sample_list_f_, sample_list_b_, start_, goal_, new_node_b);
            for (auto iter = backtrace.rbegin(); iter != backtrace.rend(); ++iter)
            {
              // convert to world frame
              double wx, wy;
              costmap_->mapToWorld(iter->x(), iter->y(), wx, wy);
              path.emplace_back(wx, wy);
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
}  // namespace path_planner
}  // namespace rmp