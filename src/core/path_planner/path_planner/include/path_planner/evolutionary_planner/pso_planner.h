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
#include "system_config/path_planner_protos/evolutionary_planner/pso_planner.pb.h"

namespace rmp {
namespace path_planner {
/**
 * @brief Class for objects that plan using the PSO algorithm
 */

class PSOPathPlanner : public PathPlanner {
private:
  using PositionSequence = std::vector<common::geometry::Points2d>;

  struct Particle {
    common::geometry::Points2d position;  // Particle position
    common::geometry::Points2d velocity;  // Particle velocity
    double fitness;                       // Particle fitness
    common::geometry::Points2d best_pos;  // Personal best position in iteration
    double best_fitness;                  // Personal best fitness in iteration

    Particle() = default;

    Particle(const common::geometry::Points2d& initial_position,
             const common::geometry::Points2d& initial_velocity, double initial_fitness)
      : position(initial_position)
      , velocity(initial_velocity)
      , fitness(initial_fitness)
      , best_pos(initial_position)
      , best_fitness(initial_fitness) {
    }
  };

public:
  /**
   * @brief Construct a new PSO object
   * @param costmap   the environment for path planning
   */
  PSOPathPlanner(costmap_2d::Costmap2DROS* costmap_ros);
  ~PSOPathPlanner();

  /**
   * @brief PSO implementation
   * @param start         start node
   * @param goal          goal node
   * @param path          The resulting path in (x, y, theta)
   * @param expand        containing the node been search during the process
   * @return  true if path found, else false
   */
  bool plan(const common::geometry::Point3d& start, const common::geometry::Point3d& goal,
            common::geometry::Points3d* path, common::geometry::Points3d* expand);

  /**
   * @brief Generate n particles with pointNum_ positions each within the map range
   * @param initial_positions The initial position sequence of particle swarm
   * @param start     start node
   * @param goal      goal node
   */
  void initializePositions(PositionSequence& initial_positions,
                           const common::geometry::Point3d& start,
                           const common::geometry::Point3d& goal);

  /**
   * @brief Calculate the value of fitness function
   * @param position  the control points calculated by PSO
   * @return fitness the value of fitness function
   */
  double calFitnessValue(const common::geometry::Points2d& position);

  /**
   * @brief A function to update the particle velocity
   * @param particle     Particles to be updated for velocity
   * @param global_best  Global optimal particle
   * @param gen          randomizer
   */
  void updateParticleVelocity(Particle& particle, const Particle& global_best,
                              std::mt19937& gen);

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
  void optimizeParticle(Particle& particle, Particle& best_particle, std::mt19937& gen,
                        common::geometry::Points3d* expand);

protected:
  // paired start and goal point for path smoothing
  common::geometry::Point3d start_, goal_;

private:
  pb::path_planner::PSOPlanner pso_config_;
  std::mutex mutex_;                           // thread lock
  std::vector<Particle> inherited_particles_;  // inherited particles
  std::unique_ptr<rmp::common::geometry::BSplineCurve> bspline_gen_;  // Path generation
};

}  // namespace path_planner
}  // namespace rmp
#endif
