/**
 * *********************************************************
 *
 * @file: ga_planner.h
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
#ifndef RMP_PATH_PLANNER_EVOLUTIONARY_PLANNER_GA_H_
#define RMP_PATH_PLANNER_EVOLUTIONARY_PLANNER_GA_H_

#include <random>
#include <mutex>
#include <vector>

#include "path_planner/path_planner.h"
#include "common/geometry/curve/bspline_curve.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Class for objects that plan using the GA algorithm
 */

class GAPathPlanner : public PathPlanner
{
private:
  using PositionSequence = std::vector<Points2d>;

private:
  struct Genets
  {
    Points2d position;    // genets position
    double fitness;       // genets fitness
    Points2d best_pos;    // Personal best position in iteration
    double best_fitness;  // Personal best fitness in iteration

    Genets() = default;

    Genets(const Points2d& initial_position, double initial_fitness)
      : position(initial_position), fitness(initial_fitness), best_pos(initial_position), best_fitness(initial_fitness)
    {
    }
  };

public:
  /**
   * @brief Construct a new GA object
   * @param costmap   the environment for path planning
   * @param obstacle_factor obstacle factor(greater means obstacles)
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
  GAPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor, int n_genets, int n_inherited,
                int point_num, double p_select, double p_crs, double p_mut, double max_speed, int init_mode,
                int max_iter);
  ~GAPathPlanner();

  /**
   * @brief GA implementation
   * @param start         start node
   * @param goal          goal node
   * @param path          optimal path consists of Node
   * @param expand        containing the node been search during the process
   * @return  true if path found, else false
   */
  bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand);

  /**
   * @brief Generate n genets with pointNum_ positions each within the map range
   * @param initial_positions The initial position sequence of particle swarm
   * @param start     start node
   * @param goal      goal node
   * @param gen_mode  generation mode
   */
  void initializePositions(PositionSequence& initial_positions, const Point3d& start, const Point3d& goal,
                           int gen_mode = GEN_MODE::CIRCLE);

  /**
   * @brief Calculate the value of fitness function
   * @param position  the control points calculated by PSO
   * @return fitness the value of fitness function
   */
  double calFitnessValue(const Points2d& position);

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
                      Points3d& expand);

public:
  enum GEN_MODE
  {
    RANDOM = 1,
    CIRCLE = 2,
  };

protected:
  int max_iter_;                     // maximum iterations
  int n_genets_;                     // number of genets
  int n_inherited_;                  // number of inherited genets
  int point_num_;                    // number of position points contained in each genets
  double p_select_, p_crs_, p_mut_;  // selection probability  crossover probability  mutation probability
  double max_speed_;                 // The maximum velocity of genets motion
  int init_mode_;                    // Set the generation mode for the initial position points of the genets swarm
  Point3d start_, goal_;             // paired start and goal point for path smoothing

private:
  std::mutex mutex_;                                                  // thread lock
  std::vector<Genets> inherited_genets_;                              // inherited genets
  std::unique_ptr<rmp::common::geometry::BSplineCurve> bspline_gen_;  // Path generation
};
}  // namespace path_planner
}  // namespace rmp
#endif
