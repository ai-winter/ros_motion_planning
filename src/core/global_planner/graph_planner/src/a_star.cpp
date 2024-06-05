/**
 * *********************************************************
 *
 * @file: a_star.cpp
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
#include "a_star.h"

#include <queue>
#include <unordered_set>
#include <vector>

namespace global_planner
{
/**
 * @brief Construct a new AStar object
 * @param costmap   the environment for path planning
 * @param dijkstra   using diksktra implementation
 * @param gbfs       using gbfs implementation
 */
AStar::AStar(costmap_2d::Costmap2D* costmap, bool dijkstra, bool gbfs) : GlobalPlanner(costmap)
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
bool AStar::plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand)
{
  // clear vector
  path.clear();
  expand.clear();

  // open list and closed list
  std::priority_queue<Node, std::vector<Node>, Node::compare_cost> open_list;
  std::unordered_map<int, Node> closed_list;

  open_list.push(start);

  // get all possible motions
  const std::vector<Node> motions = Node::getMotion();

  // main process
  while (!open_list.empty())
  {
    // pop current node from open list
    Node current = open_list.top();
    open_list.pop();

    // current node does not exist in closed list
    if (closed_list.find(current.id_) != closed_list.end())
      continue;

    closed_list.insert(std::make_pair(current.id_, current));
    expand.push_back(current);

    // goal found
    if (current == goal)
    {
      path = _convertClosedListToPath(closed_list, start, goal);
      return true;
    }

    // explore neighbor of current node
    for (const auto& motion : motions)
    {
      // explore a new node
      Node node_new = current + motion;
      node_new.id_ = grid2Index(node_new.x_, node_new.y_);

      // node_new in closed list
      if (closed_list.find(node_new.id_) != closed_list.end())
        continue;

      node_new.pid_ = current.id_;

      // next node hit the boundary or obstacle
      // prevent planning failed when the current within inflation
      if ((node_new.id_ < 0) || (node_new.id_ >= map_size_) ||
          (costmap_->getCharMap()[node_new.id_] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
           costmap_->getCharMap()[node_new.id_] >= costmap_->getCharMap()[current.id_]))
        continue;

      // if using dijkstra implementation, do not consider heuristics cost
      if (!is_dijkstra_)
        node_new.h_ = helper::dist(node_new, goal);

      // if using GBFS implementation, only consider heuristics cost
      if (is_gbfs_)
        node_new.g_ = 0.0;
      // else, g will be calculate through node_new = current + m

      open_list.push(node_new);
    }
  }

  return false;
}
}  // namespace global_planner
