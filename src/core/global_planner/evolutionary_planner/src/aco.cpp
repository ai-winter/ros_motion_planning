/**
 * *********************************************************
 *
 * @file: aco.cpp
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
#include <algorithm>
#include "aco.h"

namespace global_planner
{
/**
 * @brief Construct a new ACO object
 * @param nx          pixel number in costmap x direction
 * @param ny          pixel number in costmap y direction
 * @param resolution  costmap resolution
 * @param n_ants			number of ants
 * @param alpha				pheromone weight coefficient
 * @param beta				heuristic factor weight coefficient
 * @param rho					evaporation coefficient
 * @param Q						pheromone gain
 * @param max_iter		maximum iterations
 */
ACO::ACO(int nx, int ny, double resolution, int n_ants, int n_inherited, int point_num, double alpha, double beta,
         double rho, double Q, int init_mode, int max_iter)
  : GlobalPlanner(nx, ny, resolution)
  , n_ants_(n_ants)
  , n_inherited_(n_inherited)
  , point_num_(point_num)
  , alpha_(alpha)
  , beta_(beta)
  , rho_(rho)
  , Q_(Q)
  , init_mode_(init_mode)
  , max_iter_(max_iter)
{
  inherited_ants_.emplace_back(std::vector<std::pair<int, int>>(point_num, std::make_pair(1, 1)), 0.0);
  pheromone_mat_ = new double[static_cast<int>(nx_ * ny_)];
}

ACO::~ACO()
{
  delete[] pheromone_mat_;
}

/**
 * @brief ACO implementation
 * @param global_costmap global costmap
 * @param start         start node
 * @param goal          goal node
 * @param path          optimal path consists of Node
 * @param expand        containing the node been search during the process(unused)
 * @return  true if path found, else false
 */
bool ACO::plan(const unsigned char* global_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
               std::vector<Node>& expand)
{
  start_ = std::pair<double, double>(static_cast<double>(start.x_), static_cast<double>(start.y_));
  goal_ = std::pair<double, double>(static_cast<double>(goal.x_), static_cast<double>(goal.y_));
  costmap_ = global_costmap;
  expand.clear();
  for (size_t i = 0; i < nx_ * ny_; i++)
    pheromone_mat_[i] = 1.0;

  // variable initialization
  Ant best_ant;
  best_ant.fitness = -1;
  std::vector<Ant> ants(n_ants_);

  updateAnts(ants, start, goal, best_ant);

  // random data
  std::random_device rd;
  std::mt19937 gen(rd());

  // Iterative optimization
  for (size_t iter = 0; iter < max_iter_; iter++)
  {
    updateAnts(ants, start, goal, best_ant);
    std::vector<std::thread> ants_list = std::vector<std::thread>(n_ants_);
    for (size_t i = 0; i < n_ants_; ++i)
      ants_list[i] =
          std::thread(&ACO::optimizeAnt, this, std::ref(ants[i]), std::ref(best_ant), std::ref(gen), std::ref(expand));
    for (size_t i = 0; i < n_ants_; ++i)
      ants_list[i].join();

    // best_ant.position = ants[best_ant_idx_].best_pos;

    // pheromone deterioration
    for (size_t i = 0; i < nx_ * ny_; i++)
      pheromone_mat_[i] *= (1 - rho_);
  }

  // Generating Paths from Optimal Particles
  std::vector<std::pair<double, double>> points, b_path;
  points.emplace_back(static_cast<double>(start.x_), static_cast<double>(start.y_));
  for (const auto& pos : best_ant.position)
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

  // Update inheritance ants based on optimal fitness
  std::sort(ants.begin(), ants.end(), [](const Ant& a, const Ant& b) { return a.best_fitness > b.best_fitness; });
  inherited_ants_.clear();

  for (size_t inherit = 0; inherit < n_inherited_; ++inherit)
    inherited_ants_.emplace_back(ants[inherit]);

  return !path.empty();
}

/**
 * @brief Generate n ants with pointNum_ positions each within the map range
 * @param initial_positions The initial position sequence of ants
 * @param start     start node
 * @param goal      goal node
 * @param gen_mode  generation mode
 */
void ACO::initializePositions(PositionSequence& initial_positions, const Node& start, const Node& goal, int gen_mode)
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
  for (size_t i = 0; i < n_ants_; ++i)
  {
    std::unordered_set<int> visited;
    std::vector<std::pair<int, int>> ant_positions;
    std::vector<double> probabilities;
    double prob_sum = 0.0;
    point_id = 0;

    // Generate k * point_num_ unique coordinates
    int temp_size = 3 * point_num_;
    int temp_x[temp_size], temp_y[temp_size];
    while (point_id < temp_size)
    {
      if (gen_mode == GEN_MODE_RANDOM)
      {
        temp_x[point_id] = std::uniform_int_distribution<int>(0, nx_ - 1)(gen);
        temp_y[point_id] = std::uniform_int_distribution<int>(0, ny_ - 1)(gen);
        pos_id = grid2Index(temp_x[point_id], temp_y[point_id]);
      }
      else
      {
        // Generate random angle in radians
        double angle = std::uniform_real_distribution<double>(0, 2 * M_PI)(gen);
        // Generate random distance from the center within the circle
        double r = std::sqrt(std::uniform_real_distribution<double>(0, 1)(gen)) * radius;
        // Convert polar coordinates to Cartesian coordinates
        temp_x[point_id] = static_cast<int>(std::round(center_x + r * std::cos(angle)));
        temp_y[point_id] = static_cast<int>(std::round(center_y + r * std::sin(angle)));
        // Check if the coordinates are within the map range
        if (temp_x[point_id] >= 0 && temp_x[point_id] < nx_ && temp_y[point_id] >= 0 && temp_y[point_id] < ny_)
          pos_id = grid2Index(temp_x[point_id], temp_y[point_id]);
        else
          continue;
      }

      // Check if the coordinates have already been used
      if (visited.find(pos_id) == visited.end())
      {
        point_id++;
        visited.insert(pos_id);
        double prob_new = std::pow(pheromone_mat_[pos_id], alpha_) *
                          std::pow(1.0 / hypot(temp_x[point_id] - goal.x_, temp_y[point_id] - goal.y_), beta_);
        probabilities.push_back(prob_new);
        prob_sum += prob_new;
      }
    }

    // roulette selection
    std::for_each(probabilities.begin(), probabilities.end(), [&](double& p) { p /= prob_sum; });
    std::random_device device;
    std::mt19937 engine(device());
    std::discrete_distribution<> dist(probabilities.begin(), probabilities.end());
    for (size_t j = 0; j < point_num_; j++)
    {
      int idx = dist(engine);
      x[j] = temp_x[idx];
      y[j] = temp_y[idx];
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
      ant_positions.emplace_back(x[j], y[j]);

    initial_positions.push_back(ant_positions);
  }
}

/**
 * @brief Calculate the value of fitness function
 * @param position  the control points calculated by PSO
 * @return fitness the value of fitness function
 */
double ACO::calFitnessValue(std::vector<std::pair<int, int>> position)
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
  double b_path_length = bspline_gen_.len(b_path);
  if (b_path_length > 0)
    return 100000.0 / (b_path_length + 1000 * obs_cost);
  else
    return 0;
}

/**
 * @brief Ant update optimization iteration
 * @param ant       Ants to be updated for velocity
 * @param best_ant  Global optimal ant
 * @param gen            randomizer
 * @param expand         containing the node been search during the process
 */
void ACO::optimizeAnt(Ant& ant, Ant& best_ant, std::mt19937& gen, std::vector<Node>& expand)
{
  // Calculate fitness
  ant.fitness = calFitnessValue(ant.position);

  // reward here, increased pheromone
  double c = Q_ / static_cast<double>(ant.fitness);

  // Update global optimal ant
  lock_.lock();

  // Update expand points
  for (const auto& pos : ant.position)
  {
    pheromone_mat_[grid2Index(pos.first, pos.second)] += c;
    expand.emplace_back(Node(pos.first, pos.second));
  }

  if (ant.best_fitness > best_ant.fitness)
  {
    best_ant.fitness = ant.best_fitness;
    best_ant.position = ant.position;
  }

  lock_.unlock();
}

void ACO::updateAnts(std::vector<Ant>& ants, const Node& start, const Node& goal, Ant& best_ant)
{
  double init_fitness;
  PositionSequence init_positions;

  // Generate initial position of particle swarm
  initializePositions(init_positions, start, goal, init_mode_);

  // Ant initialization
  for (int i = 0; i < n_ants_; ++i)
  {
    std::vector<std::pair<int, int>> init_position;
    if ((i < n_inherited_) && (inherited_ants_.size() == n_inherited_))
      init_position = inherited_ants_[i].best_pos;
    else
      init_position = init_positions[i];

    // Calculate fitness
    init_fitness = calFitnessValue(init_position);
    if (init_fitness > best_ant.fitness)
    {
      best_ant.position = init_position;
      best_ant.fitness = init_fitness;
    }

    // Create and add particle objects to containers
    ants[i] = { init_position, init_fitness };
  }
}

}  // namespace global_planner