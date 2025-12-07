/**
 * *********************************************************
 *
 * @file: pso_planner.cpp
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
#include <cmath>
#include <thread>
#include <algorithm>
#include <unordered_set>

#include "path_planner/evolutionary_planner/pso_planner.h"

using namespace rmp::common::math;
using namespace rmp::common::geometry;

namespace rmp {
namespace path_planner {
/**
 * @brief Construct a new PSO object
 * @param costmap   the environment for path planning
 */
PSOPathPlanner::PSOPathPlanner(costmap_2d::Costmap2DROS* costmap_ros)
  : PathPlanner(costmap_ros), bspline_gen_(std::make_unique<BSplineCurve>()) {
  pso_config_ = config_.evolutionary_planner().pso_planner();
  inherited_particles_.emplace_back(Points2d(pso_config_.control_point_num()),
                                    Points2d(pso_config_.control_point_num()), 0.0);
}

PSOPathPlanner::~PSOPathPlanner() {
}

/**
 * @brief PSO implementation
 * @param start         start node
 * @param goal          goal node
 * @param path          The resulting path in (x, y, theta)
 * @param expand        containing the node been search during the process(unused)
 * @return  true if path found, else false
 */
bool PSOPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d* path,
                          Points3d* expand) {
  double m_start_x, m_start_y, m_goal_x, m_goal_y;
  if ((!validityCheck(start.x(), start.y(), m_start_x, m_start_y)) ||
      (!validityCheck(goal.x(), goal.y(), m_goal_x, m_goal_y))) {
    return false;
  }

  start_.setX(m_start_x);
  start_.setY(m_start_y);
  start_.setTheta(start.theta());
  goal_.setX(m_goal_x);
  goal_.setY(m_goal_y);
  goal_.setTheta(goal.theta());
  expand->clear();

  // variable initialization
  double init_fitness;
  Particle best_particle;
  PositionSequence init_positions;
  std::vector<Particle> particles;

  // Generate initial position of particle swarm
  initializePositions(init_positions, start_, goal_);

  // Particle initialization
  for (int i = 0; i < pso_config_.population_num(); ++i) {
    Points2d init_position;
    if ((i < pso_config_.inherited_num()) &&
        (inherited_particles_.size() == pso_config_.inherited_num())) {
      init_position = inherited_particles_[i].best_pos;
    } else {
      init_position = init_positions[i];
    }

    Points2d init_velocity(pso_config_.control_point_num());

    // Calculate fitness
    init_fitness = calFitnessValue(init_position);

    if ((i == 0) || (init_fitness > best_particle.fitness)) {
      best_particle.fitness = init_fitness;
      best_particle.position = init_position;
    }
    // Create and add particle objects to containers
    particles.emplace_back(init_position, init_velocity, init_fitness);
  }

  // random data
  std::random_device rd;
  std::mt19937 gen(rd());

  // Iterative optimization
  for (size_t iter = 0; iter < pso_config_.max_iterations(); iter++) {
    std::vector<std::thread> particle_list =
        std::vector<std::thread>(pso_config_.population_num());
    for (size_t i = 0; i < pso_config_.population_num(); ++i) {
      particle_list[i] =
          std::thread(&PSOPathPlanner::optimizeParticle, this, std::ref(particles[i]),
                      std::ref(best_particle), std::ref(gen), std::ref(expand));
    }
    for (size_t i = 0; i < pso_config_.population_num(); ++i) {
      particle_list[i].join();
    }
  }

  // Generating Paths from Optimal Particles
  Points3d points, b_path;
  points.emplace_back(m_start_x, m_start_y, start.theta());
  for (const auto& pos : best_particle.position) {
    points.emplace_back(pos.x(), pos.y());
  }
  points.emplace_back(m_goal_x, m_goal_y, goal.theta());
  points.erase(std::unique(std::begin(points), std::end(points)), std::end(points));

  bspline_gen_->run(points, b_path);

  // Path data structure conversion
  path->clear();
  for (const auto& pt : b_path) {
    // convert to world frame
    double wx, wy;
    costmap_->mapToWorld(pt.x(), pt.y(), wx, wy);
    path->emplace_back(wx, wy, pt.theta());
  }

  // Update inheritance particles based on optimal fitness
  std::sort(particles.begin(), particles.end(), [](const Particle& a, const Particle& b) {
    return a.best_fitness > b.best_fitness;
  });
  inherited_particles_.clear();

  for (size_t inherit = 0; inherit < pso_config_.inherited_num(); ++inherit)
    inherited_particles_.emplace_back(particles[inherit]);

  return !path->empty();
}

/**
 * @brief Generate n particles with pointNum_ positions each within the map range
 * @param initial_positions The initial position sequence of particle swarm
 * @param start     start node
 * @param goal      goal node
 */
void PSOPathPlanner::initializePositions(PositionSequence& initial_positions,
                                         const Point3d& start, const Point3d& goal) {
  // Use a random device and engine to generate random numbers
  std::random_device rd;
  std::mt19937 gen(rd());
  int x[pso_config_.control_point_num()], y[pso_config_.control_point_num()];
  int point_id, pos_id;

  // Calculate sequence direction
  bool x_order = (goal.x() > start.x());
  bool y_order = (goal.y() > start.y());

  // circle generation
  int center_x, center_y;
  double radius;
  if (config_.evolutionary_planner().init_mode() ==
      pb::path_planner::EvolutionaryInitMode::EVOLUTIONARY_INIT_MODE_RANDOM_IN_CIRCLE) {
    // Calculate the center and the radius of the circle (midpoint between start and goal)
    center_x = (start.x() + goal.x()) / 2;
    center_y = (start.y() + goal.y()) / 2;
    double dist = 0.5 * std::hypot(start.x() - goal.x(), start.y() - goal.y());
    radius = dist < 5 ? 5 : dist;
  }

  // initialize n_particles positions
  for (int i = 0; i < pso_config_.population_num(); ++i) {
    point_id = 0;
    Points2d particle_positions;
    std::unordered_set<int> visited;

    // Generate pso_config_.control_point_num() unique coordinates
    while (point_id < pso_config_.control_point_num()) {
      if (config_.evolutionary_planner().init_mode() ==
          pb::path_planner::EvolutionaryInitMode::
              EVOLUTIONARY_INIT_MODE_RANDOM_IN_MAP_UNSPECIFIED) {
        x[point_id] = std::uniform_int_distribution<int>(0, nx_ - 1)(gen);
        y[point_id] = std::uniform_int_distribution<int>(0, ny_ - 1)(gen);
        pos_id = grid2Index(x[point_id], y[point_id]);
      } else {
        // Generate random angle in radians
        double angle = std::uniform_real_distribution<double>(0, 2 * M_PI)(gen);
        // Generate random distance from the center within the circle
        double r = std::sqrt(std::uniform_real_distribution<double>(0, 1)(gen)) * radius;
        // Convert polar coordinates to Cartesian coordinates
        x[point_id] = static_cast<int>(std::round(center_x + r * std::cos(angle)));
        y[point_id] = static_cast<int>(std::round(center_y + r * std::sin(angle)));
        // Check if the coordinates are within the map range
        if (x[point_id] >= 0 && x[point_id] < nx_ && y[point_id] >= 0 &&
            y[point_id] < ny_) {
          pos_id = grid2Index(x[point_id], y[point_id]);
        } else {
          continue;
        }
      }

      // Check if the coordinates have already been used
      if (visited.find(pos_id) == visited.end()) {
        point_id++;
        visited.insert(pos_id);
      }
    }

    // sort
    if (x_order) {
      std::sort(x, x + pso_config_.control_point_num(),
                [](int a, int b) { return a < b; });
    } else {
      std::sort(x, x + pso_config_.control_point_num(),
                [](int a, int b) { return a > b; });
    }
    if (y_order) {
      std::sort(y, y + pso_config_.control_point_num(),
                [](int a, int b) { return a < b; });
    } else {
      std::sort(y, y + pso_config_.control_point_num(),
                [](int a, int b) { return a > b; });
    }

    // Store elements from x and y in particle_positions
    for (int j = 0; j < pso_config_.control_point_num(); ++j) {
      particle_positions.emplace_back(x[j], y[j]);
    }

    initial_positions.push_back(particle_positions);
  }
}

/**
 * @brief Calculate the value of fitness function
 * @param position  the control points calculated by PSO
 * @return fitness the value of fitness function
 */
double PSOPathPlanner::calFitnessValue(const Points2d& position) {
  Points3d points, b_path;
  points.emplace_back(start_.x(), start_.y(), start_.theta());
  for (const auto& pos : position) {
    points.emplace_back(pos.x(), pos.y());
  }
  points.emplace_back(goal_.x(), goal_.y(), goal_.theta());
  points.erase(std::unique(std::begin(points), std::end(points)), std::end(points));
  bspline_gen_->run(points, b_path);

  // collision detection
  int point_index;
  double obs_cost = 1;
  for (size_t i = 1; i < b_path.size(); ++i) {
    point_index =
        grid2Index(static_cast<int>(b_path[i].x()), static_cast<int>(b_path[i].y()));
    // next node hit the boundary or obstacle
    if ((point_index < 0) || (point_index >= map_size_) ||
        (costmap_->getCharMap()[point_index] >=
         costmap_2d::LETHAL_OBSTACLE * config_.obstacle_inflation_factor()))
      obs_cost++;
  }
  // Calculate particle fitness
  return 100000.0 / (bspline_gen_->distance(b_path) + 1000 * obs_cost);
}

/**
 * @brief A function to update the particle velocity
 * @param particle     Particles to be updated for velocity
 * @param global_best  Global optimal particle
 * @param gen          randomizer
 */
void PSOPathPlanner::updateParticleVelocity(Particle& particle,
                                            const Particle& global_best,
                                            std::mt19937& gen) {
  // The random numbers are distributed between [0, 1).
  std::uniform_real_distribution<double> dist(0.0, 1.0);

  // update Velocity
  for (size_t i = 0; i < pso_config_.control_point_num(); ++i) {
    double rand1 = dist(gen);
    double rand2 = dist(gen);

    particle.velocity[i].setX(
        pso_config_.weight_inertial() * particle.velocity[i].x() +
        pso_config_.weight_social() * rand1 *
            (particle.best_pos[i].x() - particle.position[i].x()) +
        pso_config_.weight_cognitive() * rand2 *
            (global_best.position[i].x() - particle.position[i].x()));

    particle.velocity[i].setY(
        pso_config_.weight_inertial() * particle.velocity[i].y() +
        pso_config_.weight_social() * rand1 *
            (particle.best_pos[i].y() - particle.position[i].y()) +
        pso_config_.weight_cognitive() * rand2 *
            (global_best.position[i].y() - particle.position[i].y()));

    // Velocity limit
    particle.velocity[i].setX(clamp(particle.velocity[i].x(),
                                    -1.0 * pso_config_.max_speed(),
                                    pso_config_.max_speed()));
    particle.velocity[i].setY(clamp(particle.velocity[i].y(),
                                    -1.0 * pso_config_.max_speed(),
                                    pso_config_.max_speed()));
  }
}

/**
 * @brief A function to update the particle position
 * @param particle     Particles to be updated for velocity
 */
void PSOPathPlanner::updateParticlePosition(Particle& particle) {
  // update Position
  for (size_t i = 0; i < pso_config_.control_point_num(); ++i) {
    particle.position[i].setX(particle.position[i].x() + particle.velocity[i].x());
    particle.position[i].setY(particle.position[i].y() + particle.velocity[i].y());

    // Position limit
    particle.position[i].setX(rmp::common::math::clamp(particle.position[i].x(), 1.0,
                                                       static_cast<double>(nx_) - 1));
    particle.position[i].setY(rmp::common::math::clamp(particle.position[i].y(), 1.0,
                                                       static_cast<double>(ny_) - 1));
  }
}

/**
 * @brief Particle update optimization iteration
 * @param particle       Particles to be updated for velocity
 * @param best_particle  Global optimal particle
 * @param gen            randomizer
 * @param expand         containing the node been search during the process
 */
void PSOPathPlanner::optimizeParticle(Particle& particle, Particle& best_particle,
                                      std::mt19937& gen, Points3d* expand) {
  // update speed
  updateParticleVelocity(particle, best_particle, gen);
  // update position
  updateParticlePosition(particle);

  // Calculate fitness
  particle.fitness = calFitnessValue(particle.position);

  // Update individual optima
  if (particle.fitness > particle.best_fitness) {
    particle.best_fitness = particle.fitness;
    particle.best_pos = particle.position;
  }

  // Update expand points
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto pos : particle.position) {
    expand->emplace_back(pos.x(), pos.y());
  }

  if (particle.best_fitness > best_particle.fitness) {
    best_particle.fitness = particle.best_fitness;
    best_particle.position = particle.position;
  }
}
}  // namespace path_planner
}  // namespace rmp