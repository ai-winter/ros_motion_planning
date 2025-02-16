/**
 * *********************************************************
 *
 * @file: aco_planner.cpp
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
#include <thread>
#include <algorithm>
#include <unordered_set>

#include "common/math/math_helper.h"
#include "path_planner/evolutionary_planner/aco_planner.h"

namespace rmp
{
namespace path_planner
{
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
ACOPathPlanner::ACOPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor, int n_ants,
                               int n_inherited, int point_num, double alpha, double beta, double rho, double Q,
                               int init_mode, int max_iter)
  : PathPlanner(costmap_ros, obstacle_factor)
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
  bspline_gen_ = std::make_unique<rmp::common::geometry::BSplineCurve>();
  inherited_ants_.emplace_back(Points2d(point_num), 0.0);
  pheromone_mat_ = new double[map_size_];
}

ACOPathPlanner::~ACOPathPlanner()
{
  delete[] pheromone_mat_;
}

/**
 * @brief ACO implementation
 * @param start         start node
 * @param goal          goal node
 * @param path          optimal path consists of Node
 * @param expand        containing the node been search during the process(unused)
 * @return  true if path found, else false
 */
bool ACOPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
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
  for (int i = 0; i < map_size_; i++)
    pheromone_mat_[i] = 1.0;

  // variable initialization
  Ant best_ant;
  best_ant.fitness = -1;
  std::vector<Ant> ants(n_ants_);

  updateAnts(ants, start_, goal_, best_ant);

  // random data
  std::random_device rd;
  std::mt19937 gen(rd());

  // Iterative optimization
  for (int iter = 0; iter < max_iter_; iter++)
  {
    updateAnts(ants, start_, goal_, best_ant);
    std::vector<std::thread> ants_list = std::vector<std::thread>(n_ants_);
    for (int i = 0; i < n_ants_; ++i)
      ants_list[i] = std::thread(&ACOPathPlanner::optimizeAnt, this, std::ref(ants[i]), std::ref(best_ant),
                                 std::ref(gen), std::ref(expand));
    for (int i = 0; i < n_ants_; ++i)
      ants_list[i].join();

    // best_ant.position = ants[best_ant_idx_].best_pos;

    // pheromone deterioration
    for (int i = 0; i < map_size_; i++)
      pheromone_mat_[i] *= (1 - rho_);
  }

  // Generating Paths from Optimal Particles
  Points3d points, b_path;
  points.emplace_back(m_start_x, m_start_y, start.theta());
  for (const auto& pos : best_ant.position)
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

  // Update inheritance ants based on optimal fitness
  std::sort(ants.begin(), ants.end(), [](const Ant& a, const Ant& b) { return a.best_fitness > b.best_fitness; });
  inherited_ants_.clear();

  for (int inherit = 0; inherit < n_inherited_; ++inherit)
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
void ACOPathPlanner::initializePositions(PositionSequence& initial_positions, const Point3d& start, const Point3d& goal,
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
  for (int i = 0; i < n_ants_; ++i)
  {
    Points2d ant_positions;
    std::unordered_set<int> visited;
    std::vector<double> probabilities;
    double prob_sum = 0.0;
    point_id = 0;

    // Generate k * point_num_ unique coordinates
    int temp_size = 3 * point_num_;
    int temp_x[temp_size], temp_y[temp_size];
    while (point_id < temp_size)
    {
      if (gen_mode == GEN_MODE::RANDOM)
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
                          std::pow(1.0 / hypot(temp_x[point_id] - goal.x(), temp_y[point_id] - goal.y()), beta_);
        probabilities.push_back(prob_new);
        prob_sum += prob_new;
      }
    }

    // roulette selection
    std::for_each(probabilities.begin(), probabilities.end(), [&](double& p) { p /= prob_sum; });
    std::random_device device;
    std::mt19937 engine(device());
    std::discrete_distribution<> dist(probabilities.begin(), probabilities.end());
    for (int j = 0; j < point_num_; j++)
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
double ACOPathPlanner::calFitnessValue(const Points2d& position)
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
  double b_path_length = bspline_gen_->distance(b_path);
  return b_path_length > 0 ? 100000.0 / (b_path_length + 1000 * obs_cost) : 0;
}

/**
 * @brief Ant update optimization iteration
 * @param ant       Ants to be updated for velocity
 * @param best_ant  Global optimal ant
 * @param gen            randomizer
 * @param expand         containing the node been search during the process
 */
void ACOPathPlanner::optimizeAnt(Ant& ant, Ant& best_ant, std::mt19937& gen, Points3d& expand)
{
  // Calculate fitness
  ant.fitness = calFitnessValue(ant.position);

  // reward here, increased pheromone
  double c = Q_ / static_cast<double>(ant.fitness);

  // Update expand points
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& pos : ant.position)
  {
    pheromone_mat_[grid2Index(static_cast<int>(pos.x()), static_cast<int>(pos.y()))] += c;
    expand.emplace_back(pos.x(), pos.y());
  }

  if (ant.best_fitness > best_ant.fitness)
  {
    best_ant.fitness = ant.best_fitness;
    best_ant.position = ant.position;
  }
}

void ACOPathPlanner::updateAnts(std::vector<Ant>& ants, const Point3d& start, const Point3d& goal, Ant& best_ant)
{
  double init_fitness;
  PositionSequence init_positions;

  // Generate initial position of particle swarm
  initializePositions(init_positions, start, goal, init_mode_);

  // Ant initialization
  for (int i = 0; i < n_ants_; ++i)
  {
    Points2d init_position;
    if ((i < n_inherited_) && (static_cast<int>(inherited_ants_.size()) == n_inherited_))
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
}  // namespace path_planner
}  // namespace rmp