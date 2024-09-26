/**
 * *********************************************************
 *
 * @file: hybrid_astar_planner.h
 * @brief: Contains the Hybrid A* planner class
 * @author: Yang Haodong
 * @date: 2024-01-03
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_HYBRID_A_STAR_H_
#define RMP_PATH_PLANNER_GRAPH_PLANNER_HYBRID_A_STAR_H_

#include "common/geometry/dubins_curve.h"

#include "path_planner/path_planner.h"
#include "path_planner/graph_planner/astar_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Class for objects that plan using the A* algorithm
 */
class HybridAStarPathPlanner : public PathPlanner
{
public:
  using Node = rmp::common::structure::Node<double>;
  class HybridNode : public Node
  {
  public:
    /**
     * @brief Constructor for Hybrid Node class
     * @param x   x value
     * @param y   y value
     * @param t
     * @param g   g value, cost to get to this node
     * @param h   h value, heuritic cost of this node
     * @param id  node's id
     * @param pid node's parent's id
     */
    HybridNode(double x = 0, double y = 0, double theta = 0, double g = 0.0, double h = 0.0, int id = 0, int pid = 0,
               int prim = 0);
    double theta() const;
    void set_theta(double theta);
    /**
     * @brief Overloading operator + for Node class
     * @param n another Node
     * @return Node with current node's and input node n's values added
     */
    HybridNode operator+(const HybridNode& n) const;

    /**
     * @brief Overloading operator == for Node class
     * @param n another Node
     * @return true if current node equals node n, else false
     */
    bool operator==(const HybridNode& n) const;

    /**
     * @brief Overloading operator != for Node class
     * @param n another Node
     * @return true if current node equals node n, else false
     */
    bool operator!=(const HybridNode& n) const;

    /**
     * @brief Get permissible motion
     * @return  Node vector of permissible motions
     */
    static std::vector<HybridNode> getMotion();

  private:
    double theta_;
    int prim_;
  };

  /**
   * @brief Construct a new Hybrid A* object
   * @param costmap   the environment for path planning
   * @param is_reverse whether reverse operation is allowed
   * @param max_curv   maximum curvature of model
   */
  HybridAStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, bool is_reverse, double max_curv);

  /**
   * @brief Destory the Hybrid A* object
   */
  ~HybridAStarPathPlanner() = default;

  /**
   * @brief Hybrid A* implementation
   * @param start          start node
   * @param goal           goal node
   * @param path           optimal path consists of Node
   * @param expand         containing the node been search during the process
   * @return true if path found, else false
   */
  bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand);

  /**
   * @brief Try using Dubins curves to connect the start and goal
   * @param start          start node
   * @param goal           goal node
   * @param path           dubins path between start and goal
   * @return true if shot successfully, else false
   */
  bool dubinsShot(const HybridNode& start, const HybridNode& goal, std::vector<HybridNode>& path);

  /**
   * @brief update index of hybrid node
   * @param node hybrid node to update
   */
  void updateIndex(HybridNode& node);

  /**
   * @brief update the h-value of hybrid node
   * @param node hybrid node to update
   */
  void updateHeuristic(HybridNode& node);

  /**
   * @brief generate heurisitic map using A* algorithm, each matric of map is the distance between it and start.
   * @param start start node
   */
  void genHeurisiticMap(const Node& start);

protected:
  /**
   * @brief Tranform from world map(x, y) to grid index(i)
   * @param wx world map x
   * @param wy world map y
   * @return index
   */
  int _worldToIndex(double wx, double wy);

  /**
   * @brief Convert closed list to path
   * @param closed_list closed list
   * @param start       start node
   * @param goal        goal node
   * @return vector containing path nodes
   */
  std::vector<HybridNode> _convertClosedListToPath(std::unordered_map<int, HybridNode>& closed_list,
                                                   const HybridNode& start, const HybridNode& goal);

private:
  HybridNode goal_;                                                 // the history goal point
  std::unordered_map<int, Node> h_map_;                             // heurisitic map
  bool is_reverse_;                                                 // whether reverse operation is allowed
  double max_curv_;                                                 // maximum curvature of model
  std::unique_ptr<AStarPathPlanner> a_star_planner_;                // A* planner
  std::unique_ptr<rmp::common::geometry::DubinsCurve> dubins_gen_;  // dubins curve generator
};
}  // namespace path_planner
}  // namespace rmp
#endif