/***********************************************************
 *
 * @file: aco.cpp
 * @breif: Contains the Ant Colony Optimization(ACO) planner class
 * @author: Yang Haodong
 * @update: 2023-7-16
 * @version: 1.0
 *
 * Copyright (c) 2023， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include <random>
#include <unordered_map>
#include <algorithm>
#include <iostream>

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
ACO::ACO(int nx, int ny, double resolution, int n_ants, double alpha, double beta, double rho, double Q, int max_iter)
  : GlobalPlanner(nx, ny, resolution)
  , n_ants_(n_ants)
  , alpha_(alpha)
  , beta_(beta)
  , rho_(rho)
  , Q_(Q)
  , max_iter_(max_iter)
{
  motion_ = getMotion();
  pheromone_edges_ = new double[static_cast<int>(nx_ * ny_ * motion_.size())];
}

ACO::~ACO()
{
  delete[] pheromone_edges_;
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
  // pheromone initialization
  for (size_t k = 0; k < nx_ * ny_ * motion_.size(); k++)
    pheromone_edges_[k] = 1.0;

  // heuristically set max steps
  int max_steps = nx_ * ny_ / 2;

  // main loop
  best_path_length_ = std::numeric_limits<int>::max();
  for (size_t i = 0; i < max_iter_; i++)
  {
    std::vector<std::thread> ants_list = std::vector<std::thread>(n_ants_);
    for (size_t j = 0; j < n_ants_; j++)
      ants_list[j] = std::thread(&ACO::antSearch, this, global_costmap, start, goal);
    for (size_t j = 0; j < n_ants_; j++)
      ants_list[j].join();

    // pheromone deterioration
    for (size_t k = 0; k < nx_ * ny_ * motion_.size(); k++)
      pheromone_edges_[k] *= (1 - rho_);
  }

  if (best_path_.size() > 0)
  {
    path = _convertClosedListToPath(best_path_, start, goal);
    return true;
  }
  return false;
}

void ACO::antSearch(const unsigned char* global_costmap, const Node& start, const Node& goal)
{
  int max_steps = nx_ * ny_ / 2;
  Ant ant(start);

  while ((!ant.found_goal_) && (ant.cur_node_ != goal) && (ant.steps_ < max_steps))
  {
    ant.path_.insert(ant.cur_node_);

    // candidate
    float prob_sum = 0.0;
    std::vector<Node> next_positions;
    std::vector<double> next_probabilities;

    for (size_t z = 0; z < motion_.size(); z++)
    {
      Node node_n = ant.cur_node_ + motion_[z];
      node_n.id_ = grid2Index(node_n.x_, node_n.y_);

      // next node hit the boundary or obstacle
      if ((node_n.id_ < 0) || (node_n.id_ >= ns_) || (global_costmap[node_n.id_] >= lethal_cost_ * factor_))
        continue;

      // current node exists in history path
      if (ant.path_.find(node_n) != ant.path_.end())
        continue;

      node_n.pid_ = ant.cur_node_.id_;

      // goal found
      if (node_n == goal)
      {
        ant.path_.insert(node_n);
        ant.found_goal_ = true;
        break;
      }
      next_positions.push_back(node_n);
      double prob_new =
          std::pow(pheromone_edges_[motion_.size() * ant.cur_node_.x_ + nx_ * ant.cur_node_.y_ + z], alpha_) *
          std::pow(1.0 / std::sqrt(std::pow((node_n.x_ - goal.x_), 2) + std::pow((node_n.y_ - goal.y_), 2)), beta_);
      next_probabilities.push_back(prob_new);
      prob_sum += prob_new;
    }
    // Ant in a cul-de-sac or no next node (ie start surrounded by obstacles)
    if (prob_sum == 0 || ant.found_goal_)
      break;

    // roulette selection
    std::for_each(next_probabilities.begin(), next_probabilities.end(), [&](double& p) { p /= prob_sum; });
    std::random_device device;
    std::mt19937 engine(device());
    std::discrete_distribution<> dist(next_probabilities.begin(), next_probabilities.end());
    ant.cur_node_ = next_positions[dist(engine)];
    ant.steps_ += 1;
  }

  // pheromone update based on successful ants
  lock_.lock();
  if (ant.found_goal_)
  {
    if (static_cast<int>(ant.path_.size()) < best_path_length_)
    {
      // save best path yet in this iteration
      best_path_length_ = ant.path_.size();
      best_path_ = ant.path_;
    }

    // reward here, increased pheromone
    double c = Q_ / static_cast<double>(ant.path_.size() - 1);
    for (const auto node : ant.path_)
    {
      // TODO:
      int px, py, z;
      index2Grid(node.pid_, px, py);
      if (node.x_ - px == 0 && node.y_ - py == 1)
        z = 0;
      else if (node.x_ - px == 1 && node.y_ - py == 0)
        z = 1;
      else if (node.x_ - px == 0 && node.y_ - py == -1)
        z = 2;
      else if (node.x_ - px == -1 && node.y_ - py == 0)
        z = 3;
      else if (node.x_ - px == 1 && node.y_ - py == 1)
        z = 4;
      else if (node.x_ - px == 1 && node.y_ - py == -1)
        z = 5;
      else if (node.x_ - px == -1 && node.y_ - py == 1)
        z = 6;
      else
        z = 7;
      pheromone_edges_[motion_.size() * px + nx_ * py + z] += c;
    }
  }
  lock_.unlock();
}

}  // namespace global_planner