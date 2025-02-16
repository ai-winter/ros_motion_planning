/**
 * *********************************************************
 *
 * @file: dstar_planner.h
 * @brief: Contains the D* planner class
 * @author: Zhanyu Guo, Haodong Yang
 * @date: 2023-03-19
 * @version: 1.0
 *
 * Copyright (c) 2024, Zhanyu Guo, Haodong Yang.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_D_STAR_H_
#define RMP_PATH_PLANNER_GRAPH_PLANNER_D_STAR_H_

#include <climits>
#include <map>

#include "path_planner/path_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Class for objects that plan using the D* algorithm
 */
class DStarPathPlanner : public PathPlanner
{
private:
  using Node = rmp::common::structure::Node<int>;
  class DNode : public Node
  {
  public:
    enum Tag
    {
      NEW = 0,
      OPEN = 1,
      CLOSED = 2
    };
    /**
     * @brief Construct a new DNode object
     * @param x   X value
     * @param y   Y value
     * @param g   Cost to get to this node
     * @param h   Heuritic cost of this node
     * @param id  DNode's id
     * @param pid DNode's parent's id
     * @param t   DNode's tag among enum Tag
     * @param k   DNode's k_min in history
     */
    DNode(const int x = 0, const int y = 0, const double g = std::numeric_limits<double>::max(),
          const double h = std::numeric_limits<double>::max(), const int id = 0, const int pid = -1, const int t = NEW,
          const double k = std::numeric_limits<double>::max())
      : Node(x, y, g, h, id, pid), t_(t), k_(k)
    {
    }

    int t() const
    {
      return t_;
    }
    double k() const
    {
      return k_;
    }
    void setTag(int t)
    {
      t_ = t;
    }
    void setK(double k)
    {
      k_ = k;
    }

  private:
    int t_;     // DNode's tag among enum Tag
    double k_;  // DNode's k_min in history
  };
  using DNodePtr = DNode*;

public:
  /**
   * @brief Construct a new DStar object
   * @param costmap   the environment for path planning
   * @param obstacle_factor obstacle factor(greater means obstacles)
   */
  DStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor = 1.0);

  /**
   * @brief Init map
   */
  void initMap();

  /**
   * @brief Reset the system
   */
  void reset();

  /**
   * @brief Insert nodePtr into the open_list with h_new
   * @param node_ptr DNode pointer of the DNode to be inserted
   * @param h_new    new h value
   */
  void insert(DNodePtr node_ptr, double h_new);

  /**
   * @brief Check if there is collision between n1 and n2
   * @param n1 DNode pointer of one DNode
   * @param n2 DNode pointer of the other DNode
   * @return true if collision, else false
   */
  bool isCollision(DNodePtr n1, DNodePtr n2);

  /**
   * @brief Get neighbour DNodePtrs of nodePtr
   * @param node_ptr   DNode to expand
   * @param neighbours neigbour DNodePtrs in vector
   */
  void getNeighbours(DNodePtr node_ptr, std::vector<DNodePtr>& neighbours);

  /**
   * @brief Get the cost between n1 and n2, return INF if collision
   * @param n1 DNode pointer of one DNode
   * @param n2 DNode pointer of the other DNode
   * @return cost between n1 and n2
   */
  double getCost(DNodePtr n1, DNodePtr n2);

  /**
   * @brief Main process of D*
   * @return k_min
   */
  double processState();

  /**
   * @brief Extract the expanded Nodes (CLOSED)
   * @param expand expanded Nodes in vector
   */
  void extractExpand(std::vector<DNode>& expand);

  /**
   * @brief Extract path for map
   * @param start start node
   * @param goal  goal node
   */
  void extractPath(const DNode& start, const DNode& goal);

  /**
   * @brief Get the closest DNode of the path to current state
   * @param current current state
   * @return the closest DNode
   */
  DNode getState(const DNode& current);

  /**
   * @brief Modify the map when collision occur between x and y in path, and then do processState()
   * @param x DNode pointer of one DNode
   */
  void modify(DNodePtr x);

  /**
   * @brief D* implementation
   * @param start  start node
   * @param goal   goal node
   * @param expand containing the node been search during the process
   * @return tuple contatining a bool as to whether a path was found, and the path
   */
  bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand);

public:
  unsigned char* curr_global_costmap_;         // current global costmap
  unsigned char* last_global_costmap_;         // last global costmap
  DNodePtr** map_;                             // grid pointer map
  std::multimap<double, DNodePtr> open_list_;  // open list, ascending order
  Points3d path_;                              // path
  Points3d expand_;                            // expand
  DNode goal_;                                 // last goal
};
}  // namespace path_planner
}  // namespace rmp
#endif
