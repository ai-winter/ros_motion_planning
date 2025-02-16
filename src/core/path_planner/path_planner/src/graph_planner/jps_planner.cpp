/**
 * *********************************************************
 *
 * @file: jps_planner.cpp
 * @brief: Contains the Jump Point Search(JPS) planner class
 * @author: Yang Haodong
 * @date: 2024-09-30
 * @version: 2.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <costmap_2d/cost_values.h>

#include "common/math/math_helper.h"
#include "path_planner/graph_planner/jps_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Constructor
 * @param costmap   the environment for path planning
 * @param obstacle_factor obstacle factor(greater means obstacles)
 */
JPSPathPlanner::JPSPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor)
  : PathPlanner(costmap_ros, obstacle_factor)
{
}

/**
 * @brief Jump Point Search(JPS) implementation
 * @param start          start node
 * @param goal           goal node
 * @param expand         containing the node been search during the process
 * @return tuple contatining a bool as to whether a path was found, and the path
 */
bool JPSPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
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
  goal_.set_x(m_goal_x);
  goal_.set_y(m_goal_y);
  goal_.set_id(grid2Index(goal_.x(), goal_.y()));

  // clear vector
  path.clear();
  expand.clear();

  // open list and closed list
  open_list_ = decltype(open_list_)();
  closed_list_ = decltype(closed_list_)();

  // initialization
  for (int i = 4; i < 6; i++)
  {  // explore left-top and right-bottom from current
    _checkSlashLine(dirs_[i], start_, open_list_);
  }
  for (int i = 6; i < 8; i++)
  {  // explore left-bottom and right-top from next
    _checkSlashLine(dirs_[i], start_, open_list_, false);
  }
  closed_list_.insert(std::make_pair(start_.id(), start_));

  // main loop
  while (!open_list_.empty())
  {
    // pop current node from open list
    auto current = open_list_.top();
    open_list_.pop();

    // current node do not exist in closed list
    if (closed_list_.find(current.id()) != closed_list_.end())
      continue;

    closed_list_.insert(std::make_pair(current.id(), current));
    expand.emplace_back(current.x(), current.y());

    // goal found
    if (current == goal_)
    {
      while (!open_list_.empty())
      {
        auto n = open_list_.top();
        open_list_.pop();
        closed_list_.insert(std::make_pair(n.id(), n));
        expand.emplace_back(n.x(), n.y());
      }

      const auto& backtrace = _convertClosedListToPath<JNode>(closed_list_, start_, goal_);
      for (auto iter = backtrace.rbegin(); iter != backtrace.rend(); ++iter)
      {
        // convert to world frame
        double wx, wy;
        costmap_->mapToWorld(iter->x(), iter->y(), wx, wy);
        path.emplace_back(wx, wy);
      }
      return true;
    }

    _jump(current, open_list_);
  }

  return false;
}

/**
 *  @brief jump point detection from current node (including slash and straight direction)
 *  @param  node        current node
 *  @param  open_list   open list
 */
void JPSPathPlanner::_jump(const JNode& node, OpenList& open_list)
{
  auto calDir = [&](int cur_id, int parent_id) {
    int cur_x, cur_y, parent_x, parent_y;
    index2Grid(cur_id, cur_x, cur_y);
    index2Grid(parent_id, parent_x, parent_y);
    int dx = cur_x - parent_x, dy = cur_y - parent_y;
    dx = std::abs(dx) > rmp::common::math::kMathEpsilon ? dx / std::abs(dx) : 0;
    dy = std::abs(dy) > rmp::common::math::kMathEpsilon ? dy / std::abs(dy) : 0;
    return grid2Index(dx, dy);
  };

  int dir = calDir(node.id(), node.pid());
  if (dir == 1 || dir == -1 || dir == nx_ || dir == -nx_)
  {
    _checkStraightLine(dir, node, open_list);
  }
  else
  {
    _checkSlashLine(dir, node, open_list);
  }

  if (node.fid() != -1)
  {
    int f_dir = calDir(node.fid(), node.id());
    _checkSlashLine(f_dir, node, open_list, false);
  }
}

/**
 *  @brief force neighbor detection
 *  @param  dir             direction index
 *  @param  cur_id          current node index
 *  @param  fn_id           the set of force neighbors
 *  @return bool            whether there exists force neighbor of current node
 */
bool JPSPathPlanner::_forceNeighborDetect(int dir, int cur_id, std::vector<int>& fn_id)
{
  fn_id.clear();
  std::array<int, 2> delta_obs = { dirs_[dir_to_obs_id_.at(dir).first], dirs_[dir_to_obs_id_.at(dir).second] };

  // horizon or vertical
  if (dir == 1 || dir == -1 || dir == nx_ || dir == -nx_)
  {
    for (int i = 0; i < 2; i++)
    {
      if (costmap_->getCharMap()[cur_id + delta_obs[i]] >= costmap_2d::LETHAL_OBSTACLE * obstacle_factor_ &&
          costmap_->getCharMap()[cur_id + delta_obs[i] + dir] < costmap_2d::LETHAL_OBSTACLE * obstacle_factor_)
      {
        fn_id.push_back(cur_id + delta_obs[i] + dir);
      }
    }
  }
  // slash
  else
  {
    for (int i = 0; i < 2; i++)
    {
      if (costmap_->getCharMap()[cur_id + delta_obs[i]] >= costmap_2d::LETHAL_OBSTACLE * obstacle_factor_ &&
          costmap_->getCharMap()[cur_id + 2 * delta_obs[i] + dir] < costmap_2d::LETHAL_OBSTACLE * obstacle_factor_)
      {
        fn_id.push_back(cur_id + 2 * delta_obs[i] + dir);
      }
    }
  }

  return !fn_id.empty();
}

/**
 *  @brief jump point detection in straight direction (only for left, right, top, bottom)
 *  @param  dir             direction index
 *  @param  node            current node
 *  @param  open_list       open list
 *  @return bool            whether there exists jump point of current node in given direction
 */
bool JPSPathPlanner::_checkStraightLine(int dir, const JNode& node, OpenList& open_list)
{
  int pt = node.id();
  std::vector<int> force_neighbors;

  while (true)
  {
    pt += dir;
    int pt_x, pt_y;
    index2Grid(pt, pt_x, pt_y);

    if (costmap_->getCharMap()[pt] >= costmap_2d::LETHAL_OBSTACLE * obstacle_factor_)
    {
      return false;
    }

    // goal is jump point
    if (pt == goal_.id())
    {
      open_list.emplace(goal_.x(), goal_.y(), node.g() + std::hypot(node.x() - goal_.x(), node.y() - goal_.y()), 0.0,
                        pt, node.id(), -1);
      return true;
    }

    // point with forced neighbor is jump point
    if (_forceNeighborDetect(dir, pt, force_neighbors))
    {
      for (int fn_id : force_neighbors)
      {
        // next node hit the boundary or obstacle
        if (fn_id < 0 || fn_id >= map_size_ || closed_list_.find(fn_id) != closed_list_.end())
        {
          continue;
        }
        open_list.emplace(pt_x, pt_y, node.g() + std::hypot(node.x() - pt_x, node.y() - pt_y),
                          std::hypot(pt_x - goal_.x(), pt_y - goal_.y()), pt, node.id(), fn_id);
      }
      return true;
    }
  }
}

/**
 *  @brief jump point detection in slash direction (only for left-top, left-bottom, right-top, right-bottom)
 *  @param  dir             direction index
 *  @param  node            current node
 *  @param  open_list       open list
 *  @param  from_cur        detection from current if true, else from the next node
 *  @return bool            whether there exists jump point of current node in given direction
 */
bool JPSPathPlanner::_checkSlashLine(int dir, const JNode& node, OpenList& open_list, bool from_cur)
{
  auto dirDecompose = [&](int dir, int& x_dir, int& y_dir) {
    if (std::abs(dir) == 1)
    {
      x_dir = dir;
      y_dir = 0;
    }
    else if (std::abs(dir) == nx_)
    {
      x_dir = 0;
      y_dir = dir;
    }
    else if (dir > 0)
    {
      x_dir = dir > nx_ ? 1 : -1;
      y_dir = nx_;
    }
    else
    {
      x_dir = std::abs(dir) > nx_ ? -1 : 1;
      y_dir = -nx_;
    }
  };

  int x_dir, y_dir;
  bool find_jp = false;
  dirDecompose(dir, x_dir, y_dir);

  // detection from current point
  if (from_cur)
  {
    // horizon and vertical
    if (_checkStraightLine(x_dir, node, open_list) || _checkStraightLine(y_dir, node, open_list))
    {
      find_jp = true;
    }
  }

  // detection from next point
  int pt = node.id();
  std::vector<int> force_neighbors;
  while (true)
  {
    pt += dir;
    int pt_x, pt_y;
    index2Grid(pt, pt_x, pt_y);

    if (costmap_->getCharMap()[pt] >= costmap_2d::LETHAL_OBSTACLE * obstacle_factor_)
    {
      return find_jp;
    }

    // goal is jump point
    if (pt == goal_.id())
    {
      open_list.emplace(goal_.x(), goal_.y(), node.g() + std::hypot(node.x() - goal_.x(), node.y() - goal_.y()), 0.0,
                        pt, node.id(), -1);
      return true;
    }

    // point with forced neighbor is jump point
    if (_forceNeighborDetect(dir, pt, force_neighbors))
    {
      for (int fn_id : force_neighbors)
      {
        // next node hit the boundary or obstacle
        if (fn_id < 0 || fn_id >= map_size_ || closed_list_.find(fn_id) != closed_list_.end())
        {
          continue;
        }
        open_list.emplace(pt_x, pt_y, node.g() + std::hypot(node.x() - pt_x, node.y() - pt_y),
                          std::hypot(pt_x - goal_.x(), pt_y - goal_.y()), pt, node.id(), fn_id);
      }
      return true;
    }

    // point with horizon or vertical jump point is jump point
    JNode temp(pt_x, pt_y, node.g() + std::hypot(node.x() - pt_x, node.y() - pt_y),
               std::hypot(pt_x - goal_.x(), pt_y - goal_.y()), pt, node.id(), -1);
    if (_checkStraightLine(x_dir, temp, open_list) || _checkStraightLine(y_dir, temp, open_list))
    {
      if (!(temp.id() < 0 || temp.id() >= map_size_ || closed_list_.find(temp.id()) != closed_list_.end()))
      {
        open_list.emplace(temp);
      }
      return true;
    }
  }
}

}  // namespace path_planner
}  // namespace rmp