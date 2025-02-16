/**
 * *********************************************************
 *
 * @file: lazy_theta_star_planner.cpp
 * @brief: Contains the lazy Theta* planner class
 * @author: Wu Maojia, Yang Haodong
 * @date: 2023-10-01
 * @version: 1.3
 *
 * Copyright (c) 2024, Wu Maojia, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <queue>

#include <costmap_2d/cost_values.h>

#include "common/geometry/collision_checker.h"
#include "path_planner/graph_planner/lazy_theta_star_planner.h"

namespace rmp
{
namespace path_planner
{
namespace
{
using CollisionChecker = rmp::common::geometry::CollisionChecker;
}

/**
 * @brief Construct a new LazyThetaStar object
 * @param costmap   the environment for path planning
 * @param obstacle_factor obstacle factor(greater means obstacles)
 */
LazyThetaStarPathPlanner::LazyThetaStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor)
  : ThetaStarPathPlanner(costmap_ros, obstacle_factor){};

/**
 * @brief Lazy Theta* implementation
 * @param start         start node
 * @param goal          goal node
 * @param path          optimal path consists of Node
 * @param expand        containing the node been search during the process
 * @return  true if path found, else false
 */
bool LazyThetaStarPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  double m_start_x, m_start_y, m_goal_x, m_goal_y;
  if ((!validityCheck(start.x(), start.y(), m_start_x, m_start_y)) ||
      (!validityCheck(goal.x(), goal.y(), m_goal_x, m_goal_y)))
  {
    return false;
  }

  Node start_node(m_start_x, m_start_y);
  Node goal_node(m_goal_x, m_goal_y);
  start_node.set_id(grid2Index(start_node.x(), start_node.y()));
  goal_node.set_id(grid2Index(goal_node.x(), goal_node.y()));
  // initialize
  closed_list_.clear();
  path.clear();
  expand.clear();

  // push the start node into open list
  std::priority_queue<Node, std::vector<Node>, Node::compare_cost> open_list;
  open_list.push(start_node);

  // main process
  while (!open_list.empty())
  {
    // pop current node from open list
    auto current = open_list.top();
    open_list.pop();

    _setVertex(current);

    if (current.g() >= std::numeric_limits<double>::max())
      continue;

    // current node does not exist in closed list
    if (closed_list_.find(current.id()) != closed_list_.end())
      continue;

    closed_list_.insert(std::make_pair(current.id(), current));
    expand.emplace_back(current.x(), current.y());

    // goal found
    if (current == goal_node)
    {
      const auto& backtrace = _convertClosedListToPath<Node>(closed_list_, start_node, goal_node);
      for (auto iter = backtrace.rbegin(); iter != backtrace.rend(); ++iter)
      {
        // convert to world frame
        double wx, wy;
        costmap_->mapToWorld(iter->x(), iter->y(), wx, wy);
        path.emplace_back(wx, wy);
      }
      return true;
    }

    // explore neighbor of current node
    for (const auto& m : motions)
    {
      // explore a new node
      // path 1
      auto node_new = current + m;  // add the .x(), .y(), g_
      node_new.set_g(current.g() + m.g());
      node_new.set_h(std::hypot(node_new.x() - goal_node.x(), node_new.y() - goal_node.y()));
      node_new.set_id(grid2Index(node_new.x(), node_new.y()));
      node_new.set_pid(current.id());

      // current node do not exist in closed list
      if (closed_list_.find(node_new.id()) != closed_list_.end())
        continue;

      // next node hit the boundary or obstacle
      if ((node_new.id() < 0) || (node_new.id() >= map_size_) ||
          (costmap_->getCharMap()[node_new.id()] >= costmap_2d::LETHAL_OBSTACLE * obstacle_factor_ &&
           costmap_->getCharMap()[node_new.id()] >= costmap_->getCharMap()[current.id()]))
        continue;

      // get parent node
      Node parent;
      parent.set_id(current.pid());
      int tmp_x, tmp_y;
      index2Grid(parent.id(), tmp_x, tmp_y);
      parent.set_x(tmp_x);
      parent.set_y(tmp_y);
      auto find_parent = closed_list_.find(parent.id());
      if (find_parent != closed_list_.end())
      {
        parent = find_parent->second;
        // path 2
        _updateVertex(parent, node_new);
      }

      open_list.push(node_new);
    }
  }

  return false;
}

/**
 * @brief update the g value of child node
 * @param parent
 * @param child
 */
void LazyThetaStarPathPlanner::_updateVertex(const Node& parent, Node& child)
{
  // path 2
  const double dist = std::hypot(parent.x() - child.x(), parent.y() - child.y());
  if (parent.g() + dist < child.g())
  {
    child.set_g(parent.g() + dist);
    child.set_pid(parent.id());
  }
}

/**
 * @brief check if the parent of vertex need to be updated. if so, update it
 * @param node
 */
void LazyThetaStarPathPlanner::_setVertex(Node& node)
{
  // get the coordinate of parent node
  Node parent;
  parent.set_id(node.pid());
  int tmp_x, tmp_y;
  index2Grid(parent.id(), tmp_x, tmp_y);
  parent.set_x(tmp_x);
  parent.set_y(tmp_y);

  // if no parent, no need to check the line of sight
  auto find_parent = closed_list_.find(parent.id());
  if (find_parent == closed_list_.end())
    return;
  parent = find_parent->second;

  auto isCollision = [&](const Node& node1, const Node& node2) {
    return CollisionChecker::BresenhamCollisionDetection(node1, node2, [&](const Node& node) {
      return costmap_->getCharMap()[grid2Index(node.x(), node.y())] >= costmap_2d::LETHAL_OBSTACLE * obstacle_factor_;
    });
  };

  if (isCollision(parent, node))
  {
    // path 1
    node.set_g(std::numeric_limits<double>::max());
    for (const auto& m : motions)
    {
      auto parent_new = node + m;
      parent_new.set_id(grid2Index(parent_new.x(), parent_new.y()));
      auto find_parent_new = closed_list_.find(parent_new.id());

      if (find_parent_new != closed_list_.end())
      {
        // parent_new exists in closed list
        parent_new = find_parent_new->second;
        const double dist = std::hypot(parent_new.x() - node.x(), parent_new.y() - node.y());
        if (parent_new.g() + dist < node.g())
        {
          node.set_g(parent_new.g() + dist);
          node.set_pid(parent_new.id());
        }
      }
    }
  }
}
}  // namespace path_planner
}  // namespace rmp
