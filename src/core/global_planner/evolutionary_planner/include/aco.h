/**
 * *********************************************************
 *
 * @file: aco.h
 * @brief: Contains the Ant Colony Optimization(ACO) planner class
 * @author: Yang Haodong
 * @date: 2023-12-27
 * @version: 2.0
 *
 * Copyright (c) 2024, Yang Haodong. 
 * All rights reserved.
 * 
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef ACO_H
#define ACO_H

#include <random>
#include <thread>
#include <mutex>
#include <vector>
#include "global_planner.h"
#include "bspline_curve.h"

#define GEN_MODE_RANDOM 1
#define GEN_MODE_CIRCLE 2

using PositionSequence = std::vector<std::vector<std::pair<int, int>>>;

namespace global_planner
{
struct Ant
{
  std::vector<std::pair<int, int>> position;  // genets position
  double fitness;                             // genets fitness
  std::vector<std::pair<int, int>> best_pos;  // Personal best position in iteration
  double best_fitness;                        // Personal best fitness in iteration

  Ant() = default;

  Ant(const std::vector<std::pair<int, int>>& initial_position, double initial_fitness)
    : position(initial_position), fitness(initial_fitness), best_pos(initial_position), best_fitness(initial_fitness)
  {
  }
};

/**
 * @brief Class for objects that plan using the GA algorithm
 */

class ACO : public GlobalPlanner
{
public:
  /**
   * @brief Construct a new ACO object
   * @param nx          pixel number in costmap x directionparticle_list
   * @param max_iter		maximum iterations
   */
  ACO(int nx, int ny, double resolution, int n_ants, int n_inherited, int point_num, double alpha, double beta,
      double rho, double Q, int init_mode, int max_iter);
  ~ACO();

  /**
   * @brief ACO implementation
   * @param global_costmap global costmap
   * @param start         start node
   * @param goal          goal node
   * @param path          optimal path consists of Node
   * @param expand        containing the node been search during the process
   * @return  true if path found, else false
   */
  bool plan(const unsigned char* global_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
            std::vector<Node>& expand);

  /**
   * @brief Generate n ants with pointNum_ positions each within the map range
   * @param initial_positions The initial position sequence of ants
   * @param start     start node
   * @param goal      goal node
   * @param gen_mode  generation mode
   */
  void initializePositions(PositionSequence& initial_positions, const Node& start, const Node& goal,
                           int gen_mode = GEN_MODE_CIRCLE);

  /**
   * @brief Calculate the value of fitness function
   * @param position  the control points calculated by PSO
   * @return fitness the value of fitness function
   */
  double calFitnessValue(std::vector<std::pair<int, int>> position);

  /**
   * @brief Ant update optimization iteration
   * @param ant       Ants to be updated for velocity
   * @param best_ant  Global optimal ant
   * @param gen            randomizer
   * @param expand         containing the node been search during the process
   */
  void optimizeAnt(Ant& ant, Ant& best_ant, std::mt19937& gen, std::vector<Node>& expand);

  void updateAnts(std::vector<Ant>& ants, const Node& start, const Node& goal, Ant& best_ant);

protected:
  int point_num_;        // number of position points contained in each ant
  int n_ants_;           // number of ants
  int n_inherited_;      // number of inherited ants
  double alpha_, beta_;  // pheromone and heuristic factor weight coefficient
  double rho_;           // evaporation coefficient
  double Q_;             // pheromone gain
  int max_iter_;         // maximum iterations
  int init_mode_;        // Set the generation mode for the initial position points of the genets swarm
  std::pair<double, double> start_, goal_;  // paired start and goal point for path smoothing
  const unsigned char* costmap_;
  double* pheromone_mat_;  // pheromone matrix

private:
  std::mutex lock_;                             // thread lock
  std::vector<Ant> inherited_ants_;             // inherited ants
  trajectory_generation::BSpline bspline_gen_;  // Path generation
};

}  // namespace global_planner

#endif