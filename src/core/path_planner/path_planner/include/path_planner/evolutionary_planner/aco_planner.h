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
#include "system_config/path_planner_protos/evolutionary_planner/aco_planner.pb.h"

namespace rmp {
namespace path_planner {
/**
 * @brief Class for objects that plan using the GA algorithm
 */

class ACOPathPlanner : public PathPlanner {
private:
  using PositionSequence = std::vector<common::geometry::Points2d>;

protected:
  struct Ant {
    common::geometry::Points2d position;  // genets position
    double fitness;                       // genets fitness
    common::geometry::Points2d best_pos;  // Personal best position in iteration
    double best_fitness;                  // Personal best fitness in iteration

    Ant() = default;

    Ant(const common::geometry::Points2d& initial_position, double initial_fitness)
      : position(initial_position)
      , fitness(initial_fitness)
      , best_pos(initial_position)
      , best_fitness(initial_fitness) {
    }
  };

public:
  /**
   * @brief Construct a new ACO object
   * @param costmap   the environment for path planning
   */
  ACOPathPlanner(costmap_2d::Costmap2DROS* costmap_ros);
  ~ACOPathPlanner();

  /**
   * @brief ACO implementation
   * @param start         start node
   * @param goal          goal node
   * @param path          The resulting path in (x, y, theta)
   * @param expand        containing the node been search during the process
   * @return  true if path found, else false
   */
  bool plan(const common::geometry::Point3d& start, const common::geometry::Point3d& goal,
            common::geometry::Points3d* path, common::geometry::Points3d* expand);

  /**
   * @brief Generate n ants with pointNum_ positions each within the map range
   * @param initial_positions The initial position sequence of ants
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
   * @brief Ant update optimization iteration
   * @param ant       Ants to be updated for velocity
   * @param best_ant  Global optimal ant
   * @param gen            randomizer
   * @param expand         containing the node been search during the process
   */
  void optimizeAnt(Ant& ant, Ant& best_ant, std::mt19937& gen,
                   common::geometry::Points3d* expand);

  void updateAnts(std::vector<Ant>& ants, const common::geometry::Point3d& start,
                  const common::geometry::Point3d& goal, Ant& best_ant);

protected:
  // paired start and goal point for path smoothing
  common::geometry::Point3d start_, goal_;
  double* pheromone_mat_;  // pheromone matrix

private:
  pb::path_planner::ACOPlanner aco_config_;
  std::mutex mutex_;                                             // thread lock
  std::vector<Ant> inherited_ants_;                              // inherited ants
  std::unique_ptr<common::geometry::BSplineCurve> bspline_gen_;  // Path generation
};

}  // namespace path_planner
}  // namespace rmp
#endif