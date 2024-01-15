/**
 * *********************************************************
 *
 * @file: lpa_star.h
 * @brief: Contains the LPA* planner class
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
#ifndef LPA_STAR_H
#define LPA_STAR_H

#include <ros/ros.h>

#include <map>
#include <algorithm>

#include "global_planner.h"

#define WINDOW_SIZE 70  // local costmap window size (in grid, 3.5m / 0.05 = 70)

namespace global_planner
{
typedef LNode* LNodePtr;

/**
 * @brief Class for objects that plan using the LPA* algorithm
 */
class LPAStar : public GlobalPlanner
{
public:
  /**
   * @brief Construct a new LPAStar object
   * @param nx         pixel number in costmap x direction
   * @param ny         pixel number in costmap y direction
   * @param resolution costmap resolution
   */
  LPAStar(int nx, int ny, double resolution);

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
  bool extractPath(const Node& start, const Node& goal);

  /**
   * @brief Get the closest Node of the path to current state
   * @param current current state
   * @return the closest Node
   */
  Node getState(const Node& current);

  /**
   * @brief LPA* implementation
   * @param costs  costmap
   * @param start  start node
   * @param goal   goal node
   * @param expand containing the node been search during the process
   * @return tuple contatining a bool as to whether a path was found, and the path
   */
  bool plan(const unsigned char* global_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
            std::vector<Node>& expand);

public:
  // start and goal ptr
  unsigned char* curr_global_costmap_;         // current global costmap
  unsigned char* last_global_costmap_;         // last global costmap
  LNodePtr** map_;                             // grid pointer map
  std::multimap<double, LNodePtr> open_list_;  // open list, ascending order
  std::vector<Node> path_;                     // path
  std::vector<Node> expand_;                   // expand
  Node start_, goal_;                          // start and goal
  LNodePtr start_ptr_, goal_ptr_, last_ptr_;   // start and goal ptr
};

}  // namespace global_planner

#endif