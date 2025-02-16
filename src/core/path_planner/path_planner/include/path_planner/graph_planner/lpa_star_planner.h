/**
 * *********************************************************
 *
 * @file: lpa_star_planner.h
 * @brief: Contains the LPA* planner class
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
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_LPA_STAR_H_
#define RMP_PATH_PLANNER_GRAPH_PLANNER_LPA_STAR_H_

#include <map>
#include <climits>
#include <algorithm>

#include "path_planner/path_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Class for objects that plan using the LPA* algorithm
 * @param costmap   the environment for path planning
 */
class LPAStarPathPlanner : public PathPlanner
{
private:
  using Node = rmp::common::structure::Node<int>;

  class LNode : public Node
  {
  public:
    /**
     * @brief Construct a new LNode object
     * @param x      X value
     * @param y      Y value
     * @param g      Cost to get to this node
     * @param h      Heuritic cost of this node
     * @param id     Node's id
     * @param pid    Node's parent's id
     * @param rhs    Node's right hand side
     * @param key    Node's key value
     */
    LNode(const int x = 0, const int y = 0, const double g = std::numeric_limits<double>::max(),
          const double h = std::numeric_limits<double>::max(), const int id = 0, const int pid = -1,
          const double rhs = std::numeric_limits<double>::max(), const double key = std::numeric_limits<double>::max())
      : Node(x, y, g, h, id, pid), rhs_(rhs), key_(key)
    {
    }

    double rhs() const
    {
      return rhs_;
    }

    double key() const
    {
      return key_;
    }

    const std::multimap<double, LNode*>::iterator& iter() const
    {
      return open_it_;
    }

    void setRhs(double rhs)
    {
      rhs_ = rhs;
    }

    void setKey(double key)
    {
      key_ = key;
    }

    void setIterator(std::multimap<double, LNode*>::iterator iter)
    {
      open_it_ = iter;
    }

  private:
    double rhs_;                                       // minimum cost moving from start(value)
    double key_;                                       // priority
    std::multimap<double, LNode*>::iterator open_it_;  // iterator
  };

  using LNodePtr = LNode*;

public:
  /**
   * @brief Construct a new LPAStar object
   * @param costmap   the environment for path planning
   * @param obstacle_factor obstacle factor(greater means obstacles)
   */
  LPAStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor = 1.0);
  ~LPAStarPathPlanner();

  /**
   * @brief Init map
   */
  void initMap();

  /**
   * @brief Reset the system
   */
  void reset();

  /**
   * @brief Get heuristics between n1 and n2
   * @param n1 LNode pointer of on LNode
   * @param n2 LNode pointer of the other LNode
   * @return heuristics between n1 and n2
   */
  double getH(LNodePtr n1, LNodePtr n2);

  /**
   * @brief Calculate the key of s
   * @param s LNode pointer
   * @return the key value
   */
  double calculateKey(LNodePtr s);

  /**
   * @brief Check if there is collision between n1 and n2
   * @param n1  DNode pointer of one DNode
   * @param n2  DNode pointer of the other DNode
   * @return true if collision, else false
   */
  bool isCollision(LNodePtr n1, LNodePtr n2);

  /**
   * @brief Get neighbour LNodePtrs of nodePtr
   * @param node_ptr   DNode to expand
   * @param neighbours neigbour LNodePtrs in vector
   */
  void getNeighbours(LNodePtr u, std::vector<LNodePtr>& neighbours);

  /**
   * @brief Get the cost between n1 and n2, return INF if collision
   * @param n1 LNode pointer of one LNode
   * @param n2 LNode pointer of the other LNode
   * @return cost between n1 and n2
   */
  double getCost(LNodePtr n1, LNodePtr n2);

  /**
   * @brief Update vertex u
   * @param u LNode pointer to update
   */
  void updateVertex(LNodePtr u);

  /**
   * @brief Main process of LPA*
   */
  void computeShortestPath();

  /**
   * @brief Extract path for map
   *
   * @param start start node
   * @param goal  goal node
   * @return flag true if extract successfully else do not
   */
  bool extractPath(const LNode& start, const LNode& goal);

  /**
   * @brief Get the closest Node of the path to current state
   * @param current current state
   * @return the closest Node
   */
  LNode getState(const LNode& current);

  /**
   * @brief LPA* implementation
   * @param start  start node
   * @param goal   goal node
   * @param expand containing the node been search during the process
   * @return tuple contatining a bool as to whether a path was found, and the path
   */
  bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand);

public:
  // start and goal ptr
  unsigned char* curr_global_costmap_;         // current global costmap
  unsigned char* last_global_costmap_;         // last global costmap
  LNodePtr** map_;                             // grid pointer map
  std::multimap<double, LNodePtr> open_list_;  // open list, ascending order
  Points3d path_;                              // path
  Points3d expand_;                            // expand
  LNode start_, goal_;                         // start and goal
  LNodePtr start_ptr_, goal_ptr_, last_ptr_;   // start and goal ptr
};

}  // namespace path_planner
}  // namespace rmp
#endif