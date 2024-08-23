/**
 * *********************************************************
 *
 * @file: d_star_lite.h
 * @brief: Contains the D* lite planner class
 * @author: Zhanyu Guo
 * @date: 2023-03-19
 * @version: 1.0
 *
 * Copyright (c) 2024, Zhanyu Guo.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef D_STAR_LITE_H
#define D_STAR_LITE_H

#include "global_planner.h"

namespace global_planner
{
typedef LNode* LNodePtr;

/**
 * @brief Class for objects that plan using the LPA* algorithm
 */
class DStarLite : public GlobalPlanner
{
public:
  /**
   * @brief Construct a new DStarLite object
   * @param costmap the environment for path planning
   */
  DStarLite(costmap_2d::Costmap2D* costmap);

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
   * @param n1 LNode pointer of one LNode
   * @param n2 LNode pointer of the other LNode
   * @return true if collision, else false
   */
  bool isCollision(LNodePtr n1, LNodePtr n2);

  /**
   * @brief Get neighbour LNodePtrs of nodePtr
   * @param node_ptr   LNode to expand
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
   * @brief Main process of D* lite
   */
  void computeShortestPath();

  /**
   * @brief Extract path for map
   * @param start start node
   * @param goal  goal node
   * @return flag true if extract successfully else do not
   */
  bool extractPath(const Node& start, const Node& goal);

  /**
   * @brief D* lite implementation
   * @param start  start node
   * @param goal   goal node
   * @param expand containing the node been search during the process
   * @return tuple contatining a bool as to whether a path was found, and the path
   */
  bool plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand);

public:
  unsigned char* curr_global_costmap_;         // current global costmap
  unsigned char* last_global_costmap_;         // last global costmap
  LNodePtr** map_;                             // grid pointer map
  std::multimap<double, LNodePtr> open_list_;  // open list, ascending order
  std::vector<Node> path_;                     // path
  std::vector<Node> expand_;                   // expand
  Node start_, goal_;                          // start and goal
  LNodePtr start_ptr_, goal_ptr_, last_ptr_;   // start and goal ptr
  double km_;                                  // correction
};

}  // namespace global_planner

#endif