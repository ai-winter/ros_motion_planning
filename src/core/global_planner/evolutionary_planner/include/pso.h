/**
 * *********************************************************
 *
 * @file: pso.h
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
#ifndef PSO_H
#define PSO_H

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
struct Particle
{
  std::vector<std::pair<int, int>> position;  // Particle position
  std::vector<std::pair<int, int>> velocity;  // Particle velocity
  double fitness;                             // Particle fitness
  std::vector<std::pair<int, int>> best_pos;  // Personal best position in iteration
  double best_fitness;                        // Personal best fitness in iteration

  Particle() = default;

  Particle(const std::vector<std::pair<int, int>>& initial_position,
           const std::vector<std::pair<int, int>>& initial_velocity, double initial_fitness)
    : position(initial_position)
    , velocity(initial_velocity)
    , fitness(initial_fitness)
    , best_pos(initial_position)
    , best_fitness(initial_fitness)
  {
  }
};

/**
 * @brief Class for objects that plan using the PSO algorithm
 */

class PSO : public GlobalPlanner
{
public:
  /**
   * @brief Construct a new PSO object
   * @param nx            pixel number in costmap x direction
   * @param ny            pixel number in costmap y direction
   * @param resolution    costmap resolution
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
  PSO(int nx, int ny, double resolution, int n_particles, int n_inherited, int point_num, double w_inertial,
      double w_social, double w_cognitive, int max_speed, int init_mode, int max_iter);
  ~PSO();

  /**
   * @brief PSO implementation
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
   * @brief Generate n particles with pointNum_ positions each within the map range
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
  void optimizeParticle(Particle& particle, Particle& best_particle, std::mt19937& gen, std::vector<Node>& expand);

protected:
  int max_iter_;     // maximum iterations
  int n_particles_;  // number of particles
  int n_inherited_;  // number of inherited particles
  int point_num_;    // number of position points contained in each particle
  double w_inertial_, w_social_,
      w_cognitive_;  // Weight coefficients for fitness calculation: inertia weight, social weight, cognitive weight
  int max_speed_;    // The maximum velocity of particle motion
  int init_mode_;    // Set the generation mode for the initial position points of the particle swarm
  std::pair<double, double> start_, goal_;  // paired start and goal point for path smoothing
  const unsigned char* costmap_;

private:
  std::mutex particles_lock_;                   // thread lock
  std::vector<Particle> inherited_particles_;   // inherited particles
  trajectory_generation::BSpline bspline_gen_;  // Path generation
};

}  // namespace global_planner
#endif
