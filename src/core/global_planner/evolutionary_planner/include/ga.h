/**
 * *********************************************************
 *
 * @file: ga.h
 * @brief: Contains the Genetic Algorithm (GA) planner class
 * @author: Jing Zongxin, Yang Haodong
 * @date: 2023-12-21
 * @version: 1.1
 *
 * Copyright (c) 2024, Jing Zongxin, Yang Haodong. 
 * All rights reserved.
 * 
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef GA_H
#define GA_H

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
struct Genets
{
  std::vector<std::pair<int, int>> position;  // genets position
  double fitness;                             // genets fitness
  std::vector<std::pair<int, int>> best_pos;  // Personal best position in iteration
  double best_fitness;                        // Personal best fitness in iteration

  Genets() = default;

  Genets(const std::vector<std::pair<int, int>>& initial_position, double initial_fitness)
    : position(initial_position), fitness(initial_fitness), best_pos(initial_position), best_fitness(initial_fitness)
  {
  }
};

/**
 * @brief Class for objects that plan using the GA algorithm
 */

class GA : public GlobalPlanner
{
public:
  /**
   * @brief Construct a new GA object
   * @param nx            pixel number in costmap x direction
   * @param ny            pixel number in costmap y direction
   * @param resolution    costmap resolution
   * @param n_genets	    number of genets
   * @param n_inherited   number of inherited genets
   * @param point_num     number of position points contained in each genets
   * @param p_select	    selection probability
   * @param p_crs		      crossover probability
   * @param p_mut	        mutation probability
   * @param max_speed		  The maximum movement speed of genets
   * @param init_mode	  Set the generation mode for the initial position points of the genets swarm
   * @param max_iter		  maximum iterations
   */
  GA(int nx, int ny, double resolution, int n_genets, int n_inherited, int point_num, double p_select, double p_crs,
     double p_mut, int max_speed, int init_mode, int max_iter);
  ~GA();

  /**
   * @brief GA implementation
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
   * @brief Generate n genets with pointNum_ positions each within the map range
   * @param initial_positions The initial position sequence of particle swarm
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
   * @brief Perform selection.
   * @param population        The population of Genets.
   * @param selected_population The selected individuals will be stored in this vector.
   */
  void selection(const std::vector<Genets>& population, std::vector<Genets>& selected_population);

  /**
   * @brief Genets update optimization iteration
   * @param genets_p      individuals selected for retention
   * @param genets_c      descendants of selected individuals to be retained
   * @param best_genets   Global optimal genets
   * @param i             genets ID
   * @param gen           randomizer
   * @param expand        containing the node been search during the process
   */
  void optimizeGenets(const Genets& genets_p, Genets& genets_c, Genets& best_genets, const int& i, std::mt19937& gen,
                      std::vector<Node>& expand);

protected:
  int max_iter_;                     // maximum iterations
  int n_genets_;                     // number of genets
  int n_inherited_;                  // number of inherited genets
  int point_num_;                    // number of position points contained in each genets
  double p_select_, p_crs_, p_mut_;  // selection probability  crossover probability  mutation probability
  int max_speed_;                    // The maximum velocity of genets motion
  int init_mode_;                    // Set the generation mode for the initial position points of the genets swarm
  std::pair<double, double> start_, goal_;  // paired start and goal point for path smoothing
  const unsigned char* costmap_;

private:
  std::mutex genets_lock_;                      // thread lock
  std::vector<Genets> inherited_genets_;        // inherited genets
  trajectory_generation::BSpline bspline_gen_;  // Path generation
};

}  // namespace global_planner
#endif
