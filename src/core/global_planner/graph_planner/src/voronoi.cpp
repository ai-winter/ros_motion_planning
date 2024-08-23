/**
 * *********************************************************
 *
 * @file: voronoi.cpp
 * @brief: Contains the Voronoi-based planner class
 * @author: Yang Haodong
 * @date: 2023-07-21
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <algorithm>
#include <queue>
#include <cmath>
#include <unordered_set>

#include "voronoi.h"

namespace global_planner
{
/**
 * @brief Construct a new Voronoi-based planning object
 * @param costmap   the environment for path planning
 * @param circumscribed_radius  the circumscribed radius of robot
 */
VoronoiPlanner::VoronoiPlanner(costmap_2d::Costmap2D* costmap, double circumscribed_radius)
  : GlobalPlanner(costmap), circumscribed_radius_(circumscribed_radius)
{
  voronoi_diagram_ = new VoronoiData*[costmap_->getSizeInCellsX()];
  for (unsigned int i = 0; i < costmap_->getSizeInCellsX(); i++)
    voronoi_diagram_[i] = new VoronoiData[costmap_->getSizeInCellsY()];
}

VoronoiPlanner::~VoronoiPlanner()
{
  for (unsigned int i = 0; i < costmap_->getSizeInCellsX(); i++)
    delete[] voronoi_diagram_[i];
  delete[] voronoi_diagram_;
}

/**
 * @brief Voronoi-based planning implementation
 * @param start         start node
 * @param goal          goal node
 * @param path          optimal path consists of Node
 * @param expand        containing the node been search during the process
 * @return  true if path found, else false
 */
bool VoronoiPlanner::plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand)
{
  return true;
}
bool VoronoiPlanner::plan(const DynamicVoronoi& voronoi, const Node& start, const Node& goal, std::vector<Node>& path)
{
  // update voronoi diagram
  for (unsigned int j = 0; j < costmap_->getSizeInCellsY(); j++)
  {
    for (unsigned int i = 0; i < costmap_->getSizeInCellsX(); i++)
    {
      voronoi_diagram_[i][j].dist = voronoi.getDistance(i, j) * costmap_->getResolution();
      voronoi_diagram_[i][j].is_voronoi = voronoi.isVoronoi(i, j);
    }
  }

  // clear vector
  path.clear();

  // start/goal to Voronoi Diagram, shortest path in Voronoi Diagram
  std::vector<Node> path_s, path_g, path_v;

  // start/goal point in Voronoi Diagram
  Node v_start, v_goal;

  if (!searchPathWithVoronoi(start, goal, path_s, &v_start))
    return false;

  if (!searchPathWithVoronoi(goal, start, path_g, &v_goal))
    return false;
  std::reverse(path_g.begin(), path_g.end());

  if (!searchPathWithVoronoi(v_start, v_goal, path_v))
    return false;

  path_g.insert(path_g.end(), path_v.begin(), path_v.end());
  path_g.insert(path_g.end(), path_s.begin(), path_s.end());
  path = path_g;

  return true;
}

/**
 * @brief search the shortest path from start to VD, or search the shortest path in VD
 * @param start         start node
 * @param goal          goal node
 * @param v_goal        the voronoi node in VD which is closest to start node
 * @param path          shortest path from start to VD
 * @return  true if path found, else false
 */
bool VoronoiPlanner::searchPathWithVoronoi(const Node& start, const Node& goal, std::vector<Node>& path, Node* v_goal)
{
  path.clear();

  // open list and closed list
  std::priority_queue<Node, std::vector<Node>, Node::compare_cost> open_list;
  std::unordered_map<int, Node> closed_list;

  open_list.push(start);

  // get all possible motions
  const std::vector<Node> motion = Node::getMotion();

  while (!open_list.empty())
  {
    // pop current node from open list
    Node current = open_list.top();
    open_list.pop();

    // current node does not exist in closed list
    if (closed_list.find(current.id()) != closed_list.end())
      continue;

    closed_list.insert(std::make_pair(current.id(), current));

    // goal found
    if ((current == goal) || (v_goal == nullptr ? false : voronoi_diagram_[current.x()][current.y()].is_voronoi))
    {
      path = _convertClosedListToPath(closed_list, start, current);
      if (v_goal != nullptr)
      {
        v_goal->set_x(current.x());
        v_goal->set_y(current.y());
        v_goal->set_id(current.id());
      }
      return true;
    }

    // explore neighbor of current node
    for (const auto& m : motion)
    {
      Node node_new = current + m;

      // current node do not exist in closed list
      if (closed_list.find(node_new.id()) != closed_list.end())
        continue;

      // explore a new node
      node_new.set_id(grid2Index(node_new.x(), node_new.y()));
      node_new.set_pid(current.id());

      // next node hit the boundary or obstacle
      if ((node_new.id() < 0) || (node_new.id() >= map_size_) ||
          (voronoi_diagram_[node_new.x()][node_new.y()].dist < circumscribed_radius_))
        continue;

      // search in VD
      if ((v_goal == nullptr) && (!voronoi_diagram_[node_new.x()][node_new.y()].is_voronoi))
        continue;

      node_new.set_h(helper::dist(node_new, goal));

      open_list.push(node_new);
    }
  }
  return false;
}
}  // namespace global_planner
