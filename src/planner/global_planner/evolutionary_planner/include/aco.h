/***********************************************************
 *
 * @file: aco.h
 * @breif: Contains the Ant Colony Optimization(ACO) planner class
 * @author: Yang Haodong
 * @update: 2023-7-16
 * @version: 1.0
 *
 * Copyright (c) 2023， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef ACO_H
#define ACO_H

#include <thread>
#include <mutex>

#include "global_planner.h"

namespace aco_planner
{
struct Ant
{
  /**
   * @brief Constructor to create a new ant
   * @param cur_node Current node for ant
   */
  Ant(const Node& cur_node = Node()) : cur_node_(cur_node)
  {
  }

  Node cur_node_;            // current node for ant
  bool found_goal_ = false;  // whether found goal or not
  int steps_ = 0;            // iteration steps
                             // history path nodes
  std::unordered_set<Node, NodeIdAsHash, compare_coordinates> path_;
};

/**
 * @brief Class for objects that plan using the ACO algorithm
 */
class ACO : public global_planner::GlobalPlanner
{
public:
  /**
   * @brief Construct a new ACO object
   * @param nx          pixel number in costmap x direction
   * @param ny          pixel number in costmap y direction
   * @param resolution  costmap resolution
   * @param n_ants			number of ants
   * @param alpha				pheromone weight coefficient
   * @param beta				heuristic factor weight coefficient
   * @param rho					evaporation coefficient
   * @param Q						pheromone gain
   * @param max_iter		maximum iterations
   */
  ACO(int nx, int ny, double resolution, int n_ants, double alpha, double beta, double rho, double Q, int max_iter);
  ~ACO();

  /**
   * @brief ACO implementation
   * @param gloal_costmap global costmap
   * @param start         start node
   * @param goal          goal node
   * @param path          optimal path consists of Node
   * @param expand        containing the node been search during the process(unused)
   * @return  true if path found, else false
   */
  bool plan(const unsigned char* gloal_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
            std::vector<Node>& expand);

  void antSearch(const unsigned char* gloal_costmap, const Node& start, const Node& goal);

protected:
  int n_ants_;           // number of ants
  double alpha_, beta_;  // pheromone and heuristic factor weight coefficient
  double rho_;           // evaporation coefficient
  double Q_;             // pheromone gain
  int max_iter_;         // maximum iterations

  // best path and its length
  int best_path_length_;
  std::unordered_set<Node, NodeIdAsHash, compare_coordinates> best_path_;
  std::vector<Node> motion_;  // all possible motions
  double* pheromone_edges_;   // pheromone matrix

private:
  std::mutex lock_;
};
}  // namespace aco_planner
#endif
