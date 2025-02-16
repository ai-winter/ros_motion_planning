/**
 * *********************************************************
 *
 * @file: aco_planner.h
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
#ifndef RMP_PATH_PLANNER_EVOLUTIONARY_PLANNER_ACO_H_
#define RMP_PATH_PLANNER_EVOLUTIONARY_PLANNER_ACO_H_

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

class ACOPathPlanner : public PathPlanner
{
private:
  using PositionSequence = std::vector<Points2d>;

protected:
  struct Ant
  {
    Points2d position;    // genets position
    double fitness;       // genets fitness
    Points2d best_pos;    // Personal best position in iteration
    double best_fitness;  // Personal best fitness in iteration

    Ant() = default;

    Ant(const Points2d& initial_position, double initial_fitness)
      : position(initial_position), fitness(initial_fitness), best_pos(initial_position), best_fitness(initial_fitness)
    {
    }
  };

public:
  /**
   * @brief Construct a new ACO object
   * @param costmap   the environment for path planning
   * @param obstacle_factor obstacle factor(greater means obstacles)
   * @param n_ants			number of ants
   * @param alpha				pheromone weight coefficient
   * @param beta				heuristic factor weight coefficient
   * @param rho					evaporation coefficient
   * @param Q						pheromone gain
   * @param max_iter		maximum iterations
   */
  ACOPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor, int n_ants, int n_inherited,
                 int point_num, double alpha, double beta, double rho, double Q, int init_mode, int max_iter);
  ~ACOPathPlanner();

  /**
   * @brief ACO implementation
   * @param start         start node
   * @param goal          goal node
   * @param path          optimal path consists of Node
   * @param expand        containing the node been search during the process
   * @return  true if path found, else false
   */
  bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand);

  /**
   * @brief Generate n ants with pointNum_ positions each within the map range
   * @param initial_positions The initial position sequence of ants
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
   * @brief Ant update optimization iteration
   * @param ant       Ants to be updated for velocity
   * @param best_ant  Global optimal ant
   * @param gen            randomizer
   * @param expand         containing the node been search during the process
   */
  void optimizeAnt(Ant& ant, Ant& best_ant, std::mt19937& gen, Points3d& expand);

  void updateAnts(std::vector<Ant>& ants, const Point3d& start, const Point3d& goal, Ant& best_ant);

public:
  enum GEN_MODE
  {
    RANDOM = 1,
    CIRCLE = 2,
  };

protected:
  int n_ants_;             // number of ants
  int n_inherited_;        // number of inherited ants
  int point_num_;          // number of position points contained in each ant
  double alpha_, beta_;    // pheromone and heuristic factor weight coefficient
  double rho_;             // evaporation coefficient
  double Q_;               // pheromone gain
  int init_mode_;          // Set the generation mode for the initial position points of the genets swarm
  int max_iter_;           // maximum iterations
  Point3d start_, goal_;   // paired start and goal point for path smoothing
  double* pheromone_mat_;  // pheromone matrix

private:
  std::mutex mutex_;                                                  // thread lock
  std::vector<Ant> inherited_ants_;                                   // inherited ants
  std::unique_ptr<rmp::common::geometry::BSplineCurve> bspline_gen_;  // Path generation
};

}  // namespace path_planner
}  // namespace rmp
#endif