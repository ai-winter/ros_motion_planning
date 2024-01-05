/***********************************************************
 *
 * @file: hybrid_a_star.h
 * @breif: Contains the Hybrid A* planner class
 * @author: Yang Haodong
 * @update: 2024-1-3
 * @version: 1.0
 *
 * Copyright (c) 2024， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef HYBRID_A_STAR_H
#define HYBRID_A_STAR_H

#include "global_planner.h"
#include "a_star.h"
#include "dubins_curve.h"

#define PENALTY_TURNING 1.05
#define PENALTY_COD 1.5
#define PENALTY_REVERSING 1.5
#define HEADINGS 72
#define DELTA_HEADING (2 * M_PI / HEADINGS)

namespace global_planner
{
/**
 * @brief Class for objects that plan using the A* algorithm
 */
class HybridAStar : public GlobalPlanner
{
public:
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
    HybridNode(double x = 0, double y = 0, double t = 0, double g = 0.0, double h = 0.0, int id = 0, int pid = 0,
               int prim = 0);

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

  public:
    double x_, y_, t_;
    int prim_;
  };

  /**
   * @brief Construct a new Hybrid A* object
   * @param nx         pixel number in costmap x direction
   * @param ny         pixel number in costmap y direction
   * @param resolution costmap resolution
   */
  HybridAStar(int nx, int ny, double resolution, bool is_reverse, double max_curv);
  ~HybridAStar();

  /**
   * @brief Hybrid A* implementation
   * @param global_costmap global costmap
   * @param start          start node
   * @param goal           goal node
   * @param path           optimal path consists of Node
   * @param expand         containing the node been search during the process
   * @return true if path found, else false
   */
  bool plan(const unsigned char* global_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
            std::vector<Node>& expand);
  bool plan(const unsigned char* global_costmap, HybridNode& start, HybridNode& goal, std::vector<Node>& path,
            std::vector<Node>& expand);

  bool dubinsShot(const HybridNode& start, const HybridNode& goal, std::vector<Node>& path);

  void updateIndex(HybridNode& node);
  void updateHeuristic(HybridNode& node, const HybridNode& goal);

protected:
  /**
   * @brief Tranform from world map(x, y) to grid map(x, y)
   * @param gx grid map x
   * @param gy grid map y
   * @param wx world map x
   * @param wy world map y
   */
  void _worldToGrid(double wx, double wy, int& gx, int& gy);

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
  std::vector<Node> _convertClosedListToPath(std::unordered_map<int, HybridNode>& closed_list, const HybridNode& start,
                                             const HybridNode& goal);

protected:
  const unsigned char* costmap_;
  bool is_reverse_;
  double max_curv_;

  trajectory_generation::Dubins dubins_gen_;
  AStar* a_star_planner_;
};
}  // namespace global_planner
#endif