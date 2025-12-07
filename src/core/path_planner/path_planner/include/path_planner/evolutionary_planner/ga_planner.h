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
#include "system_config/path_planner_protos/evolutionary_planner/ga_planner.pb.h"

namespace rmp {
namespace path_planner {
/**
 * @brief Class for objects that plan using the GA algorithm
 */

class GAPathPlanner : public PathPlanner {
private:
  using PositionSequence = std::vector<common::geometry::Points2d>;

private:
  struct Genets {
    common::geometry::Points2d position;  // genets position
    double fitness;                       // genets fitness
    common::geometry::Points2d best_pos;  // Personal best position in iteration
    double best_fitness;                  // Personal best fitness in iteration

    Genets() = default;

    Genets(const common::geometry::Points2d& initial_position, double initial_fitness)
      : position(initial_position)
      , fitness(initial_fitness)
      , best_pos(initial_position)
      , best_fitness(initial_fitness) {
    }
  };

public:
  /**
   * @brief Construct a new GA object
   * @param costmap   the environment for path planning
   */
  GAPathPlanner(costmap_2d::Costmap2DROS* costmap_ros);
  ~GAPathPlanner();

  /**
   * @brief GA implementation
   * @param start         start node
   * @param goal          goal node
   * @param path          The resulting path in (x, y, theta)
   * @param expand        containing the node been search during the process
   * @return  true if path found, else false
   */
  bool plan(const common::geometry::Point3d& start, const common::geometry::Point3d& goal,
            common::geometry::Points3d* path, common::geometry::Points3d* expand);

  /**
   * @brief Generate n genets with pointNum_ positions each within the map range
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
   * @brief Perform selection.
   * @param population        The population of Genets.
   * @param selected_population The selected individuals will be stored in this vector.
   */
  void selection(const std::vector<Genets>& population,
                 std::vector<Genets>& selected_population);

  /**
   * @brief Genets update optimization iteration
   * @param genets_p      individuals selected for retention
   * @param genets_c      descendants of selected individuals to be retained
   * @param best_genets   Global optimal genets
   * @param i             genets ID
   * @param gen           randomizer
   * @param expand        containing the node been search during the process
   */
  void optimizeGenets(const Genets& genets_p, Genets& genets_c, Genets& best_genets,
                      const int& i, std::mt19937& gen,
                      common::geometry::Points3d* expand);

protected:
  // paired start and goal point for path smoothing
  common::geometry::Point3d start_, goal_;

private:
  pb::path_planner::GAPlanner ga_config_;
  std::mutex mutex_;                                             // thread lock
  std::vector<Genets> inherited_genets_;                         // inherited genets
  std::unique_ptr<common::geometry::BSplineCurve> bspline_gen_;  // Path generation
};
}  // namespace path_planner
}  // namespace rmp
#endif
