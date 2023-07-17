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

namespace aco_planner
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
}

/**
 * @brief ACO implementation
 * @param gloal_costmap global costmap
 * @param start         start node
 * @param goal          goal node
 * @param path          optimal path consists of Node
 * @param expand        containing the node been search during the process(unused)
 * @return  true if path found, else false
 */
bool ACO::plan(const unsigned char* gloal_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
               std::vector<Node>& expand)
{
  // best path
  int best_path_length = std::numeric_limits<int>::max();
  std::unordered_set<Node, NodeIdAsHash, compare_coordinates> best_path;

  // get all possible motions
  const std::vector<Node> motion = getMotion();

  // pheromone initialization
  double* pheromone_edges = new double[static_cast<int>(nx_ * ny_ * motion.size())];
  for (size_t k = 0; k < nx_ * ny_ * motion.size(); k++)
    pheromone_edges[k] = 1.0;

  // heuristically set max steps
  int max_steps = nx_ * ny_ / 2;

  // main loop
  for (size_t i = 0; i < max_iter_; i++)
  {
    std::vector<Ant> ants_list = std::vector<Ant>(n_ants_);
    for (size_t j = 0; j < n_ants_; j++)
    {
      Ant ant(start);
      while ((ant.cur_node_ != goal) && ant.steps_ < max_steps)
      {
        ant.path_.insert(ant.cur_node_);

        // candidate
        float prob_sum = 0.0;
        std::vector<Node> next_positions;
        std::vector<double> next_probabilities;

        for (size_t z = 0; z < motion.size(); z++)
        {
          Node node_n = ant.cur_node_ + motion[z];
          node_n.id_ = grid2Index(node_n.x_, node_n.y_);

          // next node hit the boundary or obstacle
          if ((node_n.id_ < 0) || (node_n.id_ >= ns_) || (gloal_costmap[node_n.id_] >= lethal_cost_ * factor_))
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
              std::pow(pheromone_edges[motion.size() * ant.cur_node_.x_ + nx_ * ant.cur_node_.y_ + z], alpha_) *
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
      ants_list[j] = ant;
    }

    // pheromone deterioration
    for (size_t k = 0; k < nx_ * ny_ * motion.size(); k++)
      pheromone_edges[k] *= (1 - rho_);

    // pheromone update based on successful ants
    int bpl = std::numeric_limits<int>::max();
    std::unordered_set<Node, NodeIdAsHash, compare_coordinates> bp;
    for (const Ant& ant : ants_list)
    {
      if (ant.found_goal_)
      {
        if (static_cast<int>(ant.path_.size()) < bpl)
        {
          // save best path yet in this iteration
          bpl = ant.path_.size();
          bp = ant.path_;
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
          pheromone_edges[motion.size() * px + nx_ * py + z] += c;
        }
      }
    }

    // update best path
    if (bpl <= best_path_length)
    {
      best_path_length = bpl;
      best_path = bp;
    }
  }

  if (best_path.size() > 0)
  {
    path = _convertClosedListToPath(best_path, start, goal);
    return true;
  }
  return false;
}
}  // namespace aco_planner