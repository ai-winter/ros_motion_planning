/**
 * *********************************************************
 *
 * @file: voronoi_planner.h
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
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_VORONOI_PLANNER_H_
#define RMP_PATH_PLANNER_GRAPH_PLANNER_VORONOI_PLANNER_H_

#include <vector>

#include "path_planner/path_planner.h"
#include "voronoi_layer.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Class for objects that plan using the Voronoi-based planning algorithm
 */
class VoronoiPathPlanner : public PathPlanner
{
private:
  using Node = rmp::common::structure::Node<int>;
  struct VoronoiData
  {
    bool is_voronoi;  // whether the grid is in VD or not
    double dist;      // the distance from the grid to the closest obstacle
  };

public:
  /**
   * @brief Construct a new Voronoi-based planning object
   * @param costmap   the environment for path planning
   * @param circumscribed_radius  the circumscribed radius of robot
   * @param obstacle_factor obstacle factor(greater means obstacles)
   */
  VoronoiPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double circumscribed_radius, double obstacle_factor = 1.0);
  ~VoronoiPathPlanner();

  /**
   * @brief Voronoi-based planning implementation
   * @param start         start node
   * @param goal          goal node
   * @param path          optimal path consists of Node
   * @param expand        containing the node been search during the process
   * @return  true if path found, else false
   */
  bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand);

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
  /**
   * @brief update voronoi object using costmap ROS wrapper
   */
  void _updateVoronoi();

private:
  DynamicVoronoi voronoi_;         // dynamic voronoi map
  VoronoiData** voronoi_diagram_;  // voronoi diagram copy
  double circumscribed_radius_;    // the circumscribed radius of robot

  // allowable motions
  const std::vector<Node> motions = {
    { 0, 1, 1.0 },          { 1, 0, 1.0 },           { 0, -1, 1.0 },          { -1, 0, 1.0 },
    { 1, 1, std::sqrt(2) }, { 1, -1, std::sqrt(2) }, { -1, 1, std::sqrt(2) }, { -1, -1, std::sqrt(2) },
  };
};
}  // namespace path_planner
}  // namespace rmp
#endif
