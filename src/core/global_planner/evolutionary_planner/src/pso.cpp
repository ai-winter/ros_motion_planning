/**
 * *********************************************************
 *
 * @file: pso.cpp
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
#include <algorithm>
#include <cmath>
#include "pso.h"

namespace global_planner
{
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
PSO::PSO(int nx, int ny, double resolution, int n_particles, int n_inherited, int point_num, double w_inertial,
         double w_social, double w_cognitive, int max_speed, int init_mode, int max_iter)
  : GlobalPlanner(nx, ny, resolution)
  , n_particles_(n_particles)
  , n_inherited_(n_inherited)
  , point_num_(point_num)
  , w_inertial_(w_inertial)
  , w_social_(w_social)
  , w_cognitive_(w_cognitive)
  , max_speed_(max_speed)
  , init_mode_(init_mode)
  , max_iter_(max_iter)
{
  inherited_particles_.emplace_back(std::vector<std::pair<int, int>>(point_num, std::make_pair(1, 1)),
                                    std::vector<std::pair<int, int>>(point_num, std::make_pair(0, 0)), 0.0);
}

PSO::~PSO()
{
}

/**
 * @brief PSO implementation
 * @param global_costmap global costmap
 * @param start         start node
 * @param goal          goal node
 * @param path          optimal path consists of Node
 * @param expand        containing the node been search during the process(unused)
 * @return  true if path found, else false
 */
bool PSO::plan(const unsigned char* global_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
               std::vector<Node>& expand)
{
  start_ = std::pair<double, double>(static_cast<double>(start.x_), static_cast<double>(start.y_));
  goal_ = std::pair<double, double>(static_cast<double>(goal.x_), static_cast<double>(goal.y_));
  costmap_ = global_costmap;
  expand.clear();

  // variable initialization
  double init_fitness;
  Particle best_particle;
  PositionSequence init_positions;
  std::vector<Particle> particles;

  // Generate initial position of particle swarm
  initializePositions(init_positions, start, goal, init_mode_);

  // Particle initialization
  for (int i = 0; i < n_particles_; ++i)
  {
    std::vector<std::pair<int, int>> init_position;
    if ((i < n_inherited_) && (inherited_particles_.size() == n_inherited_))
      init_position = inherited_particles_[i].best_pos;
    else
      init_position = init_positions[i];

    std::vector<std::pair<int, int>> init_velocity(point_num_, std::make_pair(0, 0));

    // Calculate fitness
    init_fitness = calFitnessValue(init_position);

    if ((i == 0) || (init_fitness > best_particle.fitness))
    {
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
  for (size_t iter = 0; iter < max_iter_; iter++)
  {
    std::vector<std::thread> particle_list = std::vector<std::thread>(n_particles_);
    for (size_t i = 0; i < n_particles_; ++i)
      particle_list[i] = std::thread(&PSO::optimizeParticle, this, std::ref(particles[i]), std::ref(best_particle),
                                     std::ref(gen), std::ref(expand));
    for (size_t i = 0; i < n_particles_; ++i)
      particle_list[i].join();
  }

  // Generating Paths from Optimal Particles
  std::vector<std::pair<double, double>> points, b_path;
  points.emplace_back(static_cast<double>(start.x_), static_cast<double>(start.y_));
  for (const auto& pos : best_particle.position)
    points.emplace_back(static_cast<double>(pos.first), static_cast<double>(pos.second));
  points.emplace_back(static_cast<double>(goal.x_), static_cast<double>(goal.y_));
  points.erase(std::unique(std::begin(points), std::end(points)), std::end(points));

  bspline_gen_.run(points, b_path);

  // Path data structure conversion
  path.clear();

  if (!b_path.empty())
  {
    // Add the last point
    path.emplace_back(static_cast<int>(b_path.back().first), static_cast<int>(b_path.back().second), 0.0, 0.0,
                      static_cast<int>(b_path.size()) - 1, 0);

    // Iterate in reverse order, starting from the second-to-last point
    for (int p = b_path.size() - 2; p >= 0; --p)
    {
      int x = static_cast<int>(b_path[p].first);
      int y = static_cast<int>(b_path[p].second);

      // Check if the current point is different from the last point
      if (x != path.back().x_ || y != path.back().y_)
        path.emplace_back(x, y, 0.0, 0.0, p, 0);
    }
  }

  // Update inheritance particles based on optimal fitness
  std::sort(particles.begin(), particles.end(),
            [](const Particle& a, const Particle& b) { return a.best_fitness > b.best_fitness; });
  inherited_particles_.clear();

  for (size_t inherit = 0; inherit < n_inherited_; ++inherit)
    inherited_particles_.emplace_back(particles[inherit]);

  return !path.empty();
}

/**
 * @brief Generate n particles with pointNum_ positions each within the map range
 * @param initial_positions The initial position sequence of particle swarm
 * @param start     start node
 * @param goal      goal node
 * @param gen_mode  generation mode
 */
void PSO::initializePositions(PositionSequence& initial_positions, const Node& start, const Node& goal, int gen_mode)
{
  // Use a random device and engine to generate random numbers
  std::random_device rd;
  std::mt19937 gen(rd());
  int x[point_num_], y[point_num_];
  int point_id, pos_id;

  // Calculate sequence direction
  bool x_order = (goal.x_ > start.x_);
  bool y_order = (goal.y_ > start.y_);

  // circle generation
  int center_x, center_y;
  double radius;
  if (gen_mode == GEN_MODE_CIRCLE)
  {
    // Calculate the center and the radius of the circle (midpoint between start and goal)
    center_x = (start.x_ + goal.x_) / 2;
    center_y = (start.y_ + goal.y_) / 2;
    radius = helper::dist(start, goal) / 2.0 < 5 ? 5 : helper::dist(start, goal) / 2.0;
  }

  // initialize n_particles positions
  for (int i = 0; i < n_particles_; ++i)
  {
    std::unordered_set<int> visited;
    std::vector<std::pair<int, int>> particle_positions;
    point_id = 0;

    // Generate point_num_ unique coordinates
    while (point_id < point_num_)
    {
      if (gen_mode == GEN_MODE_RANDOM)
      {
        x[point_id] = std::uniform_int_distribution<int>(0, nx_ - 1)(gen);
        y[point_id] = std::uniform_int_distribution<int>(0, ny_ - 1)(gen);
        pos_id = grid2Index(x[point_id], y[point_id]);
      }
      else
      {
        // Generate random angle in radians
        double angle = std::uniform_real_distribution<double>(0, 2 * M_PI)(gen);
        // Generate random distance from the center within the circle
        double r = std::sqrt(std::uniform_real_distribution<double>(0, 1)(gen)) * radius;
        // Convert polar coordinates to Cartesian coordinates
        x[point_id] = static_cast<int>(std::round(center_x + r * std::cos(angle)));
        y[point_id] = static_cast<int>(std::round(center_y + r * std::sin(angle)));
        // Check if the coordinates are within the map range
        if (x[point_id] >= 0 && x[point_id] < nx_ && y[point_id] >= 0 && y[point_id] < ny_)
          pos_id = grid2Index(x[point_id], y[point_id]);
        else
          continue;
      }

      // Check if the coordinates have already been used
      if (visited.find(pos_id) == visited.end())
      {
        point_id++;
        visited.insert(pos_id);
      }
    }

    // sort
    if (x_order)
      std::sort(x, x + point_num_, [](int a, int b) { return a < b; });
    else
      std::sort(x, x + point_num_, [](int a, int b) { return a > b; });
    if (y_order)
      std::sort(y, y + point_num_, [](int a, int b) { return a < b; });
    else
      std::sort(y, y + point_num_, [](int a, int b) { return a > b; });

    // Store elements from x and y in particle_positions
    for (int j = 0; j < point_num_; ++j)
      particle_positions.emplace_back(x[j], y[j]);

    initial_positions.push_back(particle_positions);
  }
}

/**
 * @brief Calculate the value of fitness function
 * @param position  the control points calculated by PSO
 * @return fitness the value of fitness function
 */
double PSO::calFitnessValue(std::vector<std::pair<int, int>> position)
{
  std::vector<std::pair<double, double>> points, b_path;
  points.push_back(start_);
  for (const auto& pos : position)
    points.emplace_back(static_cast<double>(pos.first), static_cast<double>(pos.second));
  points.push_back(goal_);
  points.erase(std::unique(std::begin(points), std::end(points)), std::end(points));

  bspline_gen_.run(points, b_path);

  // collision detection
  int point_index;
  double obs_cost = 1;
  for (size_t i = 1; i < b_path.size(); ++i)
  {
    point_index = grid2Index(static_cast<int>(b_path[i].first), static_cast<int>(b_path[i].second));
    // next node hit the boundary or obstacle
    if ((point_index < 0) || (point_index >= ns_) || (costmap_[point_index] >= lethal_cost_ * factor_))
      obs_cost++;
  }
  // Calculate particle fitness
  return 100000.0 / (bspline_gen_.len(b_path) + 1000 * obs_cost);
}

/**
 * @brief A function to update the particle velocity
 * @param particle     Particles to be updated for velocity
 * @param global_best  Global optimal particle
 * @param gen          randomizer
 */
void PSO::updateParticleVelocity(Particle& particle, const Particle& global_best, std::mt19937& gen)
{
  // The random numbers are distributed between [0, 1).
  std::uniform_real_distribution<double> dist(0.0, 1.0);

  // update Velocity
  for (size_t i = 0; i < point_num_; ++i)
  {
    double rand1 = dist(gen);
    double rand2 = dist(gen);

    particle.velocity[i].first =
        static_cast<int>(w_inertial_ * particle.velocity[i].first +
                         w_social_ * rand1 * (particle.best_pos[i].first - particle.position[i].first) +
                         w_cognitive_ * rand2 * (global_best.position[i].first - particle.position[i].first));

    particle.velocity[i].second =
        static_cast<int>(w_inertial_ * particle.velocity[i].second +
                         w_social_ * rand1 * (particle.best_pos[i].second - particle.position[i].second) +
                         w_cognitive_ * rand2 * (global_best.position[i].second - particle.position[i].second));

    // Velocity limit
    particle.velocity[i].first = helper::clamp(particle.velocity[i].first, -1 * max_speed_, max_speed_);
    particle.velocity[i].second = helper::clamp(particle.velocity[i].second, -1 * max_speed_, max_speed_);
  }
}

/**
 * @brief A function to update the particle position
 * @param particle     Particles to be updated for velocity
 */
void PSO::updateParticlePosition(Particle& particle)
{
  // update Position
  for (size_t i = 0; i < point_num_; ++i)
  {
    particle.position[i].first += particle.velocity[i].first;
    particle.position[i].second += particle.velocity[i].second;

    // Position limit
    particle.position[i].first = helper::clamp(particle.position[i].first, 1, nx_ - 1);
    particle.position[i].second = helper::clamp(particle.position[i].second, 1, ny_ - 1);
  }
}

/**
 * @brief Particle update optimization iteration
 * @param particle       Particles to be updated for velocity
 * @param best_particle  Global optimal particle
 * @param gen            randomizer
 * @param expand         containing the node been search during the process
 */
void PSO::optimizeParticle(Particle& particle, Particle& best_particle, std::mt19937& gen, std::vector<Node>& expand)
{
  // update speed
  updateParticleVelocity(particle, best_particle, gen);
  // update position
  updateParticlePosition(particle);

  // Calculate fitness
  particle.fitness = calFitnessValue(particle.position);

  // Update individual optima
  if (particle.fitness > particle.best_fitness)
  {
    particle.best_fitness = particle.fitness;
    particle.best_pos = particle.position;
  }

  // Update global optimal particles
  particles_lock_.lock();

  // Update expand points
  for (auto pos : particle.position)
    expand.emplace_back(Node(pos.first, pos.second));

  if (particle.best_fitness > best_particle.fitness)
  {
    best_particle.fitness = particle.best_fitness;
    best_particle.position = particle.position;
  }

  particles_lock_.unlock();
}

}  // namespace global_planner