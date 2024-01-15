/**
 * *********************************************************
 *
 * @file: d_star.h
 * @brief: Contains the D* planner class
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
#ifndef D_STAR_H
#define D_STAR_H

#include <map>

#include <ros/ros.h>

#include "global_planner.h"

#define WINDOW_SIZE 70  // local costmap window size (in grid, 3.5m / 0.05 = 70)

namespace global_planner
{
typedef DNode* DNodePtr;

/**
 * @brief Class for objects that plan using the D* algorithm
 */
class DStar : public GlobalPlanner
{
public:
  /**
   * @brief Construct a new DStar object
   * @param nx         pixel number in costmap x direction
   * @param ny         pixel number in costmap y direction
   * @param resolution costmap resolution
   */
  DStar(int nx, int ny, double resolution);

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
  void extractExpand(std::vector<Node>& expand);

  /**
   * @brief Extract path for map
   * @param start start node
   * @param goal  goal node
   */
  void extractPath(const Node& start, const Node& goal);

  /**
   * @brief Get the closest Node of the path to current state
   * @param current current state
   * @return the closest Node
   */
  Node getState(const Node& current);

  /**
   * @brief Modify the map when collision occur between x and y in path, and then do processState()
   * @param x DNode pointer of one DNode
   */
  void modify(DNodePtr x);

  /**
   * @brief D* implementation
   * @param costs  costmap
   * @param start  start node
   * @param goal   goal node
   * @param expand containing the node been search during the process
   * @return tuple contatining a bool as to whether a path was found, and the path
   */
  bool plan(const unsigned char* global_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
            std::vector<Node>& expand);

public:
  unsigned char* curr_global_costmap_;         // current global costmap
  unsigned char* last_global_costmap_;         // last global costmap
  DNodePtr** map_;                             // grid pointer map
  std::multimap<double, DNodePtr> open_list_;  // open list, ascending order
  std::vector<Node> path_;                     // path
  std::vector<Node> expand_;                   // expand
  Node goal_;                                  // last goal
};
}  // namespace global_planner

#endif
