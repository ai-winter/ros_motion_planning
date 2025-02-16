/**
 * *********************************************************
 *
 * @file: astar_planner.cpp
 * @brief: Contains the A* (dijkstra and GBFS) planner class
 * @author: Yang Haodong
 * @date: 2023-12-12
 * @version: 1.2
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <queue>
#include <vector>
#include <unordered_set>

#include <costmap_2d/cost_values.h>

#include "common/structure/node.h"
#include "path_planner/graph_planner/astar_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Construct a new AStar object
 * @param costmap   the environment for path planning
 * @param dijkstra   using diksktra implementation
 * @param gbfs       using gbfs implementation
 */
AStarPathPlanner::AStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor, bool dijkstra,
                                   bool gbfs)
  : PathPlanner(costmap_ros, obstacle_factor)
{
  // can not using both dijkstra and GBFS at the same time
  if (!(dijkstra && gbfs))
  {
    is_dijkstra_ = dijkstra;
    is_gbfs_ = gbfs;
  }
  else
  {
    is_dijkstra_ = false;
    is_gbfs_ = false;
  }
};

/**
 * @brief A* implementation
 * @param start          start node
 * @param goal           goal node
 * @param path           optimal path consists of Node
 * @param expand         containing the node been search during the process
 * @return true if path found, else false
 */
bool AStarPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
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

  // clear vector
  path.clear();
  expand.clear();

  // open list and closed list
  std::priority_queue<Node, std::vector<Node>, Node::compare_cost> open_list;
  std::unordered_map<int, Node> closed_list;

  open_list.push(start_node);

  // main process
  while (!open_list.empty())
  {
    // pop current node from open list
    auto current = open_list.top();
    open_list.pop();

    // current node does not exist in closed list
    if (closed_list.find(current.id()) != closed_list.end())
      continue;

    closed_list.insert(std::make_pair(current.id(), current));
    expand.emplace_back(current.x(), current.y());

    // goal found
    if (current == goal_node)
    {
      const auto& backtrace = _convertClosedListToPath<Node>(closed_list, start_node, goal_node);
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
    for (const auto& motion : motions)
    {
      // explore a new node
      auto node_new = current + motion;
      node_new.set_g(current.g() + motion.g());
      node_new.set_id(grid2Index(node_new.x(), node_new.y()));

      // node_new in closed list
      if (closed_list.find(node_new.id()) != closed_list.end())
        continue;

      node_new.set_pid(current.id());

      // next node hit the boundary or obstacle
      // prevent planning failed when the current within inflation
      if ((node_new.id() < 0) || (node_new.id() >= map_size_) ||
          (costmap_->getCharMap()[node_new.id()] >= costmap_2d::LETHAL_OBSTACLE * obstacle_factor_ &&
           costmap_->getCharMap()[node_new.id()] >= costmap_->getCharMap()[current.id()]))
        continue;

      // if using dijkstra implementation, do not consider heuristics cost
      if (!is_dijkstra_)
        node_new.set_h(std::hypot(node_new.x() - goal_node.x(), node_new.y() - goal_node.y()));

      // if using GBFS implementation, only consider heuristics cost
      if (is_gbfs_)
        node_new.set_g(0.0);
      // else, g will be calculate through node_new = current + m

      open_list.push(node_new);
    }
  }

  return false;
}
}  // namespace path_planner
}  // namespace rmp
