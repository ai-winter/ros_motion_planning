/**
 * *********************************************************
 *
 * @file: ga_planner.cpp
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
#include <thread>
#include <algorithm>
#include <unordered_set>

#include "path_planner/evolutionary_planner/ga_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Construct a new GA object
 * @param costmap   the environment for path planning
 * @param obstacle_factor obstacle factor(greater means obstacles)
 * @param n_genets	    number of genets
 * @param n_inherited   number of inherited genets
 * @param point_num   number of position points contained in each genets
 * @param p_select	    selection probability
 * @param p_crs		      crossover probability
 * @param p_mut	        mutation probability
 * @param max_speed		  The maximum movement speed of genets
 * @param init_mode	  Set the generation mode for the initial position points of the genets swarm
 * @param max_iter		  maximum iterations
 */
GAPathPlanner::GAPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor, int n_genets,
                             int n_inherited, int point_num, double p_select, double p_crs, double p_mut,
                             double max_speed, int init_mode, int max_iter)
  : PathPlanner(costmap_ros, obstacle_factor)
  , n_genets_(n_genets)
  , n_inherited_(n_inherited)
  , point_num_(point_num)
  , p_select_(p_select)
  , p_crs_(p_crs)
  , p_mut_(p_mut)
  , max_speed_(max_speed)
  , init_mode_(init_mode)
  , max_iter_(max_iter)
{
  bspline_gen_ = std::make_unique<rmp::common::geometry::BSplineCurve>();
  inherited_genets_.emplace_back(Points2d(point_num), 0.0);
}

GAPathPlanner::~GAPathPlanner()
{
}

/**
 * @brief GA implementation
 * @param start         start node
 * @param goal          goal node
 * @param path          optimal path consists of Node
 * @param expand        containing the node been search during the process(unused)
 * @return  true if path found, else false
 */
bool GAPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  double m_start_x, m_start_y, m_goal_x, m_goal_y;
  if ((!validityCheck(start.x(), start.y(), m_start_x, m_start_y)) ||
      (!validityCheck(goal.x(), goal.y(), m_goal_x, m_goal_y)))
  {
    return false;
  }

  start_.setX(m_start_x);
  start_.setY(m_start_y);
  start_.setTheta(start.theta());
  goal_.setX(m_goal_x);
  goal_.setY(m_goal_y);
  goal_.setTheta(goal.theta());
  expand.clear();

  if ((n_genets_ <= 0) || (n_genets_ % 2 != 0))
  {
    R_ERROR << " GA : The parameter n_genets is set improperly. Please ensure that it is a positive even number.";
    return false;
  }

  // variable initialization
  double init_fitness;
  Genets best_genet;
  PositionSequence init_positions;
  std::vector<Genets> genets_swarm;
  std::vector<Genets> genets_parent;
  std::vector<Genets> genets_children;

  // Generate initial position of genets swarm
  initializePositions(init_positions, start_, goal_, init_mode_);

  // genets initialization
  for (int i = 0; i < n_genets_; ++i)
  {
    Points2d init_position;

    if ((i < n_inherited_) && (inherited_genets_.size() == n_inherited_))
      init_position = inherited_genets_[i].best_pos;
    else
      init_position = init_positions[i];

    // Calculate fitness
    init_fitness = calFitnessValue(init_position);

    if ((i == 0) || (init_fitness > best_genet.fitness))
    {
      best_genet.fitness = init_fitness;
      best_genet.position = init_position;
    }
    // Create and add genets objects to containers
    genets_swarm.emplace_back(init_position, init_fitness);
  }

  // random data
  std::random_device rd;
  std::mt19937 gen(rd());

  // Iterative optimization
  for (size_t iter = 0; iter < max_iter_; iter++)
  {
    selection(genets_swarm, genets_parent);

    genets_children = genets_parent;
    std::rotate(genets_children.begin(), genets_children.begin() + 1, genets_children.end());

    std::vector<std::thread> genets_list = std::vector<std::thread>(genets_parent.size());
    for (size_t i = 0; i < genets_parent.size(); ++i)
      genets_list[i] =
          std::thread(&GAPathPlanner::optimizeGenets, this, std::cref(genets_parent[i]), std::ref(genets_children[i]),
                      std::ref(best_genet), i, std::ref(gen), std::ref(expand));
    for (size_t i = 0; i < genets_parent.size(); ++i)
      genets_list[i].join();

    // Copy the elements from genets_parent and genets_children to genets_swarm
    std::copy(genets_children.begin(), genets_children.end(), genets_swarm.begin());
    std::copy(genets_parent.begin(), genets_parent.end(), genets_swarm.begin() + genets_children.size());
  }

  // Generating Paths from Optimal Genets
  Points3d points, b_path;
  points.emplace_back(m_start_x, m_start_y, start.theta());
  for (const auto& pos : best_genet.position)
  {
    points.emplace_back(pos.x(), pos.y());
  }
  points.emplace_back(m_goal_x, m_goal_y, goal.theta());
  points.erase(std::unique(std::begin(points), std::end(points)), std::end(points));

  bspline_gen_->run(points, b_path);

  // Path data structure conversion
  path.clear();
  for (const auto& pt : b_path)
  {
    // convert to world frame
    double wx, wy;
    costmap_->mapToWorld(pt.x(), pt.y(), wx, wy);
    path.emplace_back(wx, wy, pt.theta());
  }

  // Update inheritance genets based on optimal fitness
  std::sort(genets_swarm.begin(), genets_swarm.end(),
            [](const Genets& a, const Genets& b) { return a.best_fitness > b.best_fitness; });
  inherited_genets_.clear();

  for (size_t inherit = 0; inherit < n_inherited_; ++inherit)
    inherited_genets_.emplace_back(genets_swarm[inherit]);

  return !path.empty();
}

/**
 * @brief Generate n particles with pointNum_ positions each within the map range
 * @param initial_positions The initial position sequence of particle swarm
 * @param start     start node
 * @param goal      goal node
 * @param gen_mode  generation mode
 */
void GAPathPlanner::initializePositions(PositionSequence& initial_positions, const Point3d& start, const Point3d& goal,
                                        int gen_mode)
{
  // Use a random device and engine to generate random numbers
  std::random_device rd;
  std::mt19937 gen(rd());
  int x[point_num_], y[point_num_];
  int point_id, pos_id;

  // Calculate sequence direction
  bool x_order = (goal.x() > start.x());
  bool y_order = (goal.y() > start.y());

  // circle generation
  int center_x, center_y;
  double radius;
  if (gen_mode == GEN_MODE::CIRCLE)
  {
    // Calculate the center and the radius of the circle (midpoint between start and goal)
    center_x = (start.x() + goal.x()) / 2;
    center_y = (start.y() + goal.y()) / 2;
    double dist = 0.5 * std::hypot(start.x() - goal.x(), start.y() - goal.y());
    radius = dist < 5 ? 5 : dist;
  }

  // initialize n_particles positions
  for (int i = 0; i < n_genets_; ++i)
  {
    std::unordered_set<int> visited;
    Points2d genet_positions;
    point_id = 0;

    // Generate point_num_ unique coordinates
    while (point_id < point_num_)
    {
      if (gen_mode == GEN_MODE::RANDOM)
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
      genet_positions.emplace_back(x[j], y[j]);

    initial_positions.push_back(genet_positions);
  }
}

/**
 * @brief Calculate the value of fitness function
 * @param position  the control points calculated by PSO
 * @return fitness the value of fitness function
 */
double GAPathPlanner::calFitnessValue(const Points2d& position)
{
  Points3d points, b_path;
  points.emplace_back(start_.x(), start_.y(), start_.theta());
  for (const auto& pos : position)
  {
    points.emplace_back(pos.x(), pos.y());
  }
  points.emplace_back(goal_.x(), goal_.y(), goal_.theta());
  points.erase(std::unique(std::begin(points), std::end(points)), std::end(points));

  bspline_gen_->run(points, b_path);

  // collision detection
  int point_index;
  double obs_cost = 1;
  for (size_t i = 1; i < b_path.size(); ++i)
  {
    point_index = grid2Index(static_cast<int>(b_path[i].x()), static_cast<int>(b_path[i].y()));
    // next node hit the boundary or obstacle
    if ((point_index < 0) || (point_index >= map_size_) ||
        (costmap_->getCharMap()[point_index] >= costmap_2d::LETHAL_OBSTACLE * obstacle_factor_))
      obs_cost++;
  }
  // Calculate particle fitness
  return 100000.0 / (bspline_gen_->distance(b_path) + 1000 * obs_cost);
}

/**
 * @brief Perform selection.
 * @param population        The population of Genets.
 * @param selected_population The selected individuals will be stored in this vector.
 */
void GAPathPlanner::selection(const std::vector<Genets>& population, std::vector<Genets>& selected_population)
{
  // Calculate selection mode
  int select_mode = (static_cast<int>(100 * p_select_) % 2 == 0) ? 0 : 1;
  // Calculate the number of individuals to be selected
  size_t select_num = static_cast<size_t>(population.size() * p_select_);

  // Perform selection using roulette wheel method.
  if (select_mode)
  {
    // Calculate the inverse fitness values
    std::vector<double> fitness_values;
    fitness_values.reserve(population.size());
    for (const auto& genet : population)
      fitness_values.push_back(genet.fitness);

    // Calculate the total fitness
    double total_fit = std::accumulate(fitness_values.begin(), fitness_values.end(), 0.0);

    // Calculate cumulative probability
    std::vector<double> acc_p;
    acc_p.reserve(population.size());
    double sum = 0.0;
    for (const auto& v : fitness_values)
    {
      sum += v / total_fit;
      acc_p.push_back(sum);
    }

    // Initialize the selected individuals with the first individual from the population
    selected_population.clear();
    selected_population.push_back(population.front());

    std::vector<size_t> selected_indices;
    selected_indices.reserve(select_num);

    // Set up random number generation
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(0.0, 1.0);

    // Execute the selection process
    for (size_t i = 1; i < select_num; ++i)
    {
      // Generate a random number
      double rand_num = dis(gen);
      // Find the cumulative probability greater than the random number
      auto it = std::find_if(acc_p.begin(), acc_p.end(), [rand_num](double val) { return val > rand_num; });

      if (it != acc_p.end())
      {
        // Get the index of the first cumulative probability greater than the random number
        size_t idx = std::distance(acc_p.begin(), it);

        // Allow duplicate selection
        selected_population.push_back(population[idx]);
        selected_indices.push_back(idx);
      }
      else
        --i;  // Retry if not found, i.e., random number is too close to 1.0
    }
  }
  // Select individuals to be retained based on their fitness level
  else
  {
    auto population_c = population;

    std::sort(population_c.begin(), population_c.end(),
              [](const Genets& a, const Genets& b) { return a.fitness > b.fitness; });
    selected_population.clear();

    for (size_t i = 0; i < select_num; ++i)
      selected_population.emplace_back(population_c[i]);
  }
}

/**
 * @brief Genets update optimization iteration
 * @param genets_p      individuals selected for retention
 * @param genets_c      descendants of selected individuals to be retained
 * @param best_genets   Global optimal genets
 * @param i             genets ID
 * @param gen           randomizer
 * @param expand        containing the node been search during the process
 */
void GAPathPlanner::optimizeGenets(const Genets& genets_p, Genets& genets_c, Genets& best_genet, const int& i,
                                   std::mt19937& gen, Points3d& expand)
{
  std::uniform_real_distribution<double> dist_d(0.0, 1.0);
  std::uniform_int_distribution<int> dist_i(0, point_num_ - 1);
  double random1 = dist_d(gen);
  double random2 = dist_d(gen);

  // Perform crossover operation
  if (random1 < p_crs_)
  {
    int random_id = dist_i(gen);
    std::copy(genets_p.position.begin() + random_id, genets_p.position.end(), genets_c.position.begin() + random_id);
  }

  // Perform mutation operation
  if (random2 < p_mut_)
  {
    int random_id = dist_i(gen);

    // If it exceeds a certain number of times, it is considered that the mutation has failed
    for (size_t i = 0; i < 10; i++)
    {
      std::uniform_real_distribution<double> dist_m(-1.0, 1.0);
      double random3 = dist_m(gen);
      double random4 = dist_m(gen);
      double x = genets_c.position[random_id].x() + random3 * max_speed_;
      double y = genets_c.position[random_id].y() + random4 * max_speed_;

      int point_index = grid2Index(static_cast<int>(x), static_cast<int>(y));
      if ((point_index >= 0) && (point_index < map_size_) &&
          (costmap_->getCharMap()[point_index] < costmap_2d::LETHAL_OBSTACLE * obstacle_factor_))
      {
        genets_c.position[random_id].setX(x);
        genets_c.position[random_id].setY(y);
        break;
      }
    }
  }

  // Calculate fitness
  genets_c.fitness = calFitnessValue(genets_c.position);

  // Update individual optima
  if (genets_c.fitness > genets_c.best_fitness)
  {
    genets_c.best_fitness = genets_c.fitness;
    genets_c.best_pos = genets_c.position;
  }

  // Update expand points
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto pos : genets_c.position)
    expand.emplace_back(pos.x(), pos.y());

  for (auto pos : genets_p.position)
    expand.emplace_back(pos.x(), pos.y());

  if (genets_c.best_fitness > best_genet.fitness)
  {
    best_genet.fitness = genets_c.best_fitness;
    best_genet.position = genets_c.best_pos;
  }
}
}  // namespace path_planner
}  // namespace rmp
