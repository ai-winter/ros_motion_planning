/***********************************************************
 *
 * @file: a_star.cpp
 * @breif: Contains the A* (dijkstra and GBFS) planner class
 * @author: Yang Haodong
 * @update: 2022-10-27
 * @version: 1.1
 *
 * Copyright (c) 2022， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include <cmath>
#include <queue>
#include <unordered_set>
#include <vector>

#include "a_star.h"

namespace a_star_planner
{
/**
 * @brief Construct a new AStar object
 * @param nx          pixel number in costmap x direction
 * @param ny          pixel number in costmap y direction
 * @param resolution  costmap resolution
 * @param dijkstra    using diksktra implementation
 * @param gbfs        using gbfs implementation
 */
AStar::AStar(int nx, int ny, double resolution, bool dijkstra, bool gbfs) : GlobalPlanner(nx, ny, resolution)
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
 * @param gloal_costmap global costmap
 * @param start         start node
 * @param goal          goal node
 * @param path          optimal path consists of Node
 * @param expand        containing the node been search during the process
 * @return  true if path found, else false
 */
bool AStar::plan(const unsigned char* gloal_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
                 std::vector<Node>& expand)
{
  // clear vector
  path.clear();
  expand.clear();

  // open list and closed list
  std::priority_queue<Node, std::vector<Node>, compare_cost> open_list;
  std::unordered_set<Node, NodeIdAsHash, compare_coordinates> closed_list;

  open_list.push(start);

  // get all possible motions
  const std::vector<Node> motion = getMotion();

  // main process
  while (1)
  {
    if (open_list.empty())
      break;

    // pop current node from open list
    Node current = open_list.top();
    open_list.pop();

    // current node do not exist in closed list
    if (closed_list.find(current) != closed_list.end())
      continue;

    closed_list.insert(current);
    expand.push_back(current);

    // goal found
    if (current == goal)
    {
      path = _convertClosedListToPath(closed_list, start, goal);
      return true;
    }

    // explore neighbor of current node
    for (const auto& m : motion)
    {
      Node new_point = current + m;

      // current node do not exist in closed list
      if (closed_list.find(new_point) != closed_list.end())
        continue;

      // explore a new node
      new_point.id_ = grid2Index(new_point.x_, new_point.y_);
      new_point.pid_ = current.id_;

      // next node hit the boundary or obstacle
      if ((new_point.id_ < 0) || (new_point.id_ >= ns_) || (gloal_costmap[new_point.id_] >= lethal_cost_ * factor_))
        continue;

      // if using dijkstra implementation, do not consider heuristics cost
      if (!is_dijkstra_)
        new_point.h_ = std::hypot(new_point.x_ - goal.x_, new_point.y_ - goal.y_);

      // if using GBFS implementation, only consider heuristics cost
      if (is_gbfs_)
        new_point.g_ = 0.0;
      // else, g will be calculate through new_point = current + m

      open_list.push(new_point);
    }
  }

  return false;
}
}  // namespace a_star_planner
