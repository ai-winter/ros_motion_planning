/**
 * *********************************************************
 *
 * @file: bi_jps_planner.cpp
 * @brief: Contains the Bi-Jump Point Search(bi-JPS) planner class
 * @author: Yang Haodong
 * @date: 2024-09-30
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <costmap_2d/cost_values.h>

#include "path_planner/graph_planner/bi_jps_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Constructor
 * @param costmap   the environment for path planning
 * @param obstacle_factor obstacle factor(greater means obstacles)
 */
BiJPSPathPlanner::BiJPSPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor)
  : JPSPathPlanner(costmap_ros, obstacle_factor)
{
}

/**
 * @brief Bi-Jump Point Search(bi-JPS) implementation
 * @param start  start node
 * @param goal   goal node
 * @param expand containing the node been search during the process
 * @return tuple contatining a bool as to whether a path was found, and the path
 */
bool BiJPSPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  double m_start_x, m_start_y, m_goal_x, m_goal_y;
  if ((!validityCheck(start.x(), start.y(), m_start_x, m_start_y)) ||
      (!validityCheck(goal.x(), goal.y(), m_goal_x, m_goal_y)))
  {
    return false;
  }

  start_.set_x(m_start_x);
  start_.set_y(m_start_y);
  start_.set_id(grid2Index(start_.x(), start_.y()));
  start_.set_h(std::hypot(m_start_x - m_goal_x, m_start_y - m_goal_y));
  goal_.set_x(m_goal_x);
  goal_.set_y(m_goal_y);
  goal_.set_id(grid2Index(goal_.x(), goal_.y()));
  goal_.set_h(std::hypot(m_start_x - m_goal_x, m_start_y - m_goal_y));

  // clear vector
  path.clear();
  expand.clear();

  // open list and closed list
  f_open_list_ = decltype(f_open_list_)();
  f_closed_list_ = decltype(f_closed_list_)();
  b_open_list_ = decltype(b_open_list_)();
  b_closed_list_ = decltype(b_closed_list_)();

  // initialization
  for (int i = 4; i < 6; i++)
  {  // explore left-top and right-bottom from current
    _checkSlashLine(dirs_[i], start_, f_open_list_);
    _checkSlashLine(dirs_[i], goal_, b_open_list_);
  }
  for (int i = 6; i < 8; i++)
  {  // explore left-bottom and right-top from next
    _checkSlashLine(dirs_[i], start_, f_open_list_, false);
    _checkSlashLine(dirs_[i], goal_, b_open_list_, false);
  }
  f_closed_list_.insert(std::make_pair(start_.id(), start_));
  b_closed_list_.insert(std::make_pair(goal_.id(), goal_));

  // main loop
  while (!f_open_list_.empty() && !b_open_list_.empty())
  {
    // pop current node from forward / backward open list
    auto f_current = f_open_list_.top();
    auto b_current = b_open_list_.top();
    f_open_list_.pop();
    b_open_list_.pop();
    expand.emplace_back(f_current.x(), f_current.y());
    expand.emplace_back(b_current.x(), b_current.y());

    // current node do not exist in closed list
    if (f_closed_list_.find(f_current.id()) == f_closed_list_.end())
    {
      f_closed_list_.insert(std::make_pair(f_current.id(), f_current));
    }
    if (b_closed_list_.find(b_current.id()) == b_closed_list_.end())
    {
      b_closed_list_.insert(std::make_pair(b_current.id(), b_current));
    }

    // find forward point in backward closed set or find backward point in forward closed set
    if ((b_closed_list_.find(f_current.id()) != b_closed_list_.end()) ||
        (f_closed_list_.find(b_current.id()) != f_closed_list_.end()))
    {
      double len = 0.0;
      auto boundary = b_closed_list_.find(f_current.id()) != b_closed_list_.end() ? f_current : b_current;
      const auto& backtrace =
          _convertBiClosedListToPath<JNode>(f_closed_list_, b_closed_list_, start_, goal_, boundary);
      for (auto iter = backtrace.rbegin(); iter != backtrace.rend(); ++iter)
      {
        if (!path.empty())
        {
          len += std::hypot(path.back().x() - iter->x(), path.back().y() - iter->y());
        }
        // convert to world frame
        double wx, wy;
        costmap_->mapToWorld(iter->x(), iter->y(), wx, wy);
        path.emplace_back(wx, wy);
      }

      if (last_path_.empty() || last_len_ > len)
      {
        last_len_ = len;
        last_path_ = path;
      }
      else
      {
        path = last_path_;
      }

      return true;
    }

    _jump(f_current, f_open_list_);
    _jump(b_current, b_open_list_);
  }

  return false;
}

}  // namespace path_planner
}  // namespace rmp