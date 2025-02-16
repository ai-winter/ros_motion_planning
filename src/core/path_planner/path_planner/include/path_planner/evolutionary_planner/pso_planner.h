/**
 * *********************************************************
 *
 * @file: pso_planner.h
 * @brief: Contains the Particle Swarm Optimization(PSO) planner class
 * @author: Jing Zongxin, Yang Haodong
 * @date: 2023-12-20
 * @version: 1.1
 *
 * Copyright (c) 2024, Jing Zongxin, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_PATH_PLANNER_EVOLUTIONARY_PLANNER_PSO_H_
#define RMP_PATH_PLANNER_EVOLUTIONARY_PLANNER_PSO_H_

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
 * @brief Class for objects that plan using the PSO algorithm
 */

class PSOPathPlanner : public PathPlanner
{
private:
  using PositionSequence = std::vector<Points2d>;

  struct Particle
  {
    Points2d position;    // Particle position
    Points2d velocity;    // Particle velocity
    double fitness;       // Particle fitness
    Points2d best_pos;    // Personal best position in iteration
    double best_fitness;  // Personal best fitness in iteration

    Particle() = default;

    Particle(const Points2d& initial_position, const Points2d& initial_velocity, double initial_fitness)
      : position(initial_position)
      , velocity(initial_velocity)
      , fitness(initial_fitness)
      , best_pos(initial_position)
      , best_fitness(initial_fitness)
    {
    }
  };

public:
  /**
   * @brief Construct a new PSO object
   * @param costmap   the environment for path planning
   * @param obstacle_factor obstacle factor(greater means obstacles)
   * @param n_particles	  number of particles
   * @param n_inherited   number of inherited particles
   * @param point_num      number of position points contained in each particle
   * @param w_inertial	  inertia weight
   * @param w_social		  social weight
   * @param w_cognitive	  cognitive weight
   * @param max_speed		  The maximum movement speed of particles
   * @param init_mode	  Set the generation mode for the initial position points of the particle swarm
   * @param max_iter		  maximum iterations
   */
  PSOPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor, int n_particles, int n_inherited,
                 int point_num, double w_inertial, double w_social, double w_cognitive, double max_speed, int init_mode,
                 int max_iter);
  ~PSOPathPlanner();

  /**
   * @brief PSO implementation
   * @param start         start node
   * @param goal          goal node
   * @param path          optimal path consists of Node
   * @param expand        containing the node been search during the process
   * @return  true if path found, else false
   */
  bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand);

  /**
   * @brief Generate n particles with pointNum_ positions each within the map range
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
   * @brief A function to update the particle velocity
   * @param particle     Particles to be updated for velocity
   * @param global_best  Global optimal particle
   * @param gen          randomizer
   */
  void updateParticleVelocity(Particle& particle, const Particle& global_best, std::mt19937& gen);

  /**
   * @brief A function to update the particle position
   * @param particle     Particles to be updated for velocity
   */
  void updateParticlePosition(Particle& particle);

  /**
   * @brief Particle update optimization iteration
   * @param particle       Particles to be updated for velocity
   * @param best_particle  Global optimal particle
   * @param gen            randomizer
   * @param expand         containing the node been search during the process
   */
  void optimizeParticle(Particle& particle, Particle& best_particle, std::mt19937& gen, Points3d& expand);

public:
  enum GEN_MODE
  {
    RANDOM = 1,
    CIRCLE = 2,
  };

protected:
  int max_iter_;     // maximum iterations
  int n_particles_;  // number of particles
  int n_inherited_;  // number of inherited particles
  int point_num_;    // number of position points contained in each particle
  double w_inertial_, w_social_,
      w_cognitive_;   // Weight coefficients for fitness calculation: inertia weight, social weight, cognitive weight
  double max_speed_;  // The maximum velocity of particle motion
  int init_mode_;     // Set the generation mode for the initial position points of the particle swarm
  Point3d start_, goal_;  // paired start and goal point for path smoothing

private:
  std::mutex mutex_;                                                  // thread lock
  std::vector<Particle> inherited_particles_;                         // inherited particles
  std::unique_ptr<rmp::common::geometry::BSplineCurve> bspline_gen_;  // Path generation
};

}  // namespace path_planner
}  // namespace rmp
#endif
