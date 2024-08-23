/**
 * *********************************************************
 *
 * @file: voronoi.h
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
#ifndef VORONOI_H
#define VORONOI_H

#include <vector>

#include "global_planner.h"
#include "voronoi_layer.h"

namespace global_planner
{
struct VoronoiData
{
  bool is_voronoi;  // whether the grid is in VD or not
  double dist;      // the distance from the grid to the closest obstacle
};

/**
 * @brief Class for objects that plan using the Voronoi-based planning algorithm
 */
class VoronoiPlanner : public GlobalPlanner
{
public:
  /**
   * @brief Construct a new Voronoi-based planning object
   * @param costmap   the environment for path planning
   * @param circumscribed_radius  the circumscribed radius of robot
   */
  VoronoiPlanner(costmap_2d::Costmap2D* costmap, double circumscribed_radius);
  ~VoronoiPlanner();

  /**
   * @brief Voronoi-based planning implementation
   * @param start         start node
   * @param goal          goal node
   * @param path          optimal path consists of Node
   * @param expand        containing the node been search during the process
   * @return  true if path found, else false
   */
  bool plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand);
  bool plan(const DynamicVoronoi& voronoi, const Node& start, const Node& goal, std::vector<Node>& path);

protected:
  /**
   * @brief search the shortest path from start to VD, or search the shortest path in VD
   * @param start         start node
   * @param goal          goal node
   * @param v_goal        the voronoi node in VD which is closest to start node
   * @param path          shortest path from start to VD
   * @return  true if path found, else false
   */
  bool searchPathWithVoronoi(const Node& start, const Node& goal, std::vector<Node>& path, Node* v_goal = nullptr);

private:
  VoronoiData** voronoi_diagram_;  // voronoi diagram copy
  double circumscribed_radius_;    // the circumscribed radius of robot
};

}  // namespace global_planner
#endif
