/**
 * *********************************************************
 *
 * @file: quick_informed_rrt.cpp
 * @brief: Contains the quick informed RRT* planner class
 * @author: Honbo He
 * @date: 2024-06-05
 * @version: 0.9
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <random>
#include <omp.h>

#include "quick_informed_rrt.h"

namespace global_planner
{
/**
 * @brief Construct a quick informed new RRTStar object
 * @param costmap    the environment for path planning
 * @param sample_num andom sample points
 * @param max_dist   max distance between sample points
 * @param r          optimization radius
 * @param r_set      radius of priority circles set
 * @param n_threads  parallel rewire process
 * @param d_extend   increased distance of adaptive extend step size
 * @param t_freedom  freedom of t distribution
 */
QuickInformedRRT::QuickInformedRRT(costmap_2d::Costmap2D* costmap, int sample_num, double max_dist, double r,
                                   double r_set, int n_threads, double d_extend, double t_freedom)
  : InformedRRT(costmap, sample_num, max_dist, r)
  , r_set_(r_set)
  , n_threads_(n_threads)
  , d_extend_(d_extend)
  , max_dist_(max_dist)
  , t_freedom_(t_freedom)
{
}

/**
 * @brief Quick informed RRT star implementation
 * @param start  start node
 * @param goal   goal node
 * @param expand containing the node been search during the process
 * @return true if path found, else false
 */
bool QuickInformedRRT::plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand)
{
  // clear vector
  path.clear();
  expand.clear();
  sample_list_.clear();

  // initialization
  c_best_ = std::numeric_limits<double>::max();
  c_min_ = helper::dist(start, goal);
  int best_parent = -1;

  // copy
  start_ = start, goal_ = goal;
  sample_list_.insert(std::make_pair(start.id(), start));
  expand.push_back(start);

  // adaptive sampling bias
  double dist_s2g = helper::dist(start, goal);
  double dist_m2g = dist_s2g * 0.95;

  // priority circles set
  double mu = 0.0;
  int mu_cnt = 0;

  // main loop
  int iteration = 0;
  while (iteration < sample_num_)
  {
    // update probability of sampling bias
    opti_sample_p_ = std::min(0.75, 1 - dist_m2g / dist_s2g);

    // generate a random node in the map
    Node sample_node = _generateRandomNode(mu, path);

    // regular the sample node
    Node new_node = _findNearestPoint(sample_list_, sample_node);
    if (new_node.id() == -1)
      continue;
    else
    {
      sample_list_.insert(std::make_pair(new_node.id(), new_node));
      expand.push_back(new_node);
    }

    auto dist = helper::dist(new_node, goal_);
    // update min dist from tree to goal
    if (dist < dist_m2g)
      dist_m2g = dist;

    // goal found
    if (dist <= max_dist_ && !_isAnyObstacleInPath(new_node, goal_))
    {
      double cost = dist + new_node.g();
      if (cost < c_best_)
      {
        best_parent = new_node.id();
        c_best_ = cost;

        // update path
        Node goal_star(goal_.x(), goal_.y(), c_best_, 0, grid2Index(goal_.x(), goal_.y()), best_parent);
        sample_list_.insert(std::make_pair(goal_star.id(), goal_star));
        path = _convertClosedListToPath(sample_list_, start, goal);
        sample_list_.erase(goal_star.id());
        mu = std::fmin(5, mu + 0.5);
      }
    }
    else  // if do not update, decrease mu termly
    {
      mu_cnt++;
      if (mu_cnt % 100 == 0)
        mu = std::fmax(0, mu - 0.5);
    }

    iteration++;
  }

  if (best_parent != -1)
  {
    Node goal_star(goal_.x(), goal_.y(), c_best_, 0, grid2Index(goal_.x(), goal_.y()), best_parent);
    sample_list_.insert(std::make_pair(goal_star.id(), goal_star));

    path = _convertClosedListToPath(sample_list_, start, goal);
    return true;
  }

  return false;
}

/**
 * @brief Generates a random node
 * @return generated node
 * @param mu
 * @param path
 */
Node QuickInformedRRT::_generateRandomNode(int mu, std::vector<Node> path)
{
  std::random_device rd;
  std::mt19937 eng(rd());
  std::student_t_distribution<> t_distr(t_freedom_);

  if (std::abs(t_distr(eng)) < mu && path.size() != 0)  // sample in priority circles
  {
    int wc = rand() % path.size();
    std::uniform_real_distribution<float> p(-r_set_, r_set_);
    int cx = path[wc].x() + p(eng);
    int cy = path[wc].y() + p(eng);
    return Node(cx, cy, 0, 0, grid2Index(cx, cy), 0);
  }
  else  // ellipse sample
  {
    if (c_best_ < std::numeric_limits<double>::max())
    {
      while (true)
      {
        // unit ball sample
        double x, y;
        std::uniform_real_distribution<float> p(-1, 1);
        while (true)
        {
          x = p(eng);
          y = p(eng);
          if (x * x + y * y < 1)
            break;
        }
        // transform to ellipse
        Node temp = _transform(x, y);
        if (temp.id() < map_size_ - 1)
          return temp;
      }
    }
    else
      return RRTStar::_generateRandomNode();
  }
}

/**
 * @brief Regular the new node by the nearest node in the sample list
 * @param list sample list
 * @param node sample node
 * @return nearest node
 */
Node QuickInformedRRT::_findNearestPoint(std::unordered_map<int, Node> list, Node& node)
{
  Node nearest_node, new_node(node);
  double min_dist = std::numeric_limits<double>::max();

  for (const auto p : list)
  {
    // calculate distance
    double new_dist = helper::dist(p.second, new_node);

    // update nearest node
    if (new_dist < min_dist)
    {
      nearest_node = p.second;
      new_node.set_pid(nearest_node.id());
      new_node.set_g(new_dist + p.second.g());
      min_dist = new_dist;
    }
  }

  // distance longer than the threshold
  if (min_dist > max_dist_)
  {
    // connect sample node and nearest node, then move the nearest node
    // forward to sample node with `max_distance` as result
    double theta = helper::angle(nearest_node, new_node);
    new_node.set_x(nearest_node.x() + (int)(max_dist_ * cos(theta)));
    new_node.set_y(nearest_node.y() + (int)(max_dist_ * sin(theta)));
    new_node.set_id(grid2Index(new_node.x(), new_node.y()));
    new_node.set_g(max_dist_ + nearest_node.g());
  }

  // already in tree or collide
  if (list.count(new_node.id()) || _isAnyObstacleInPath(new_node, nearest_node))
    new_node.set_id(-1);
  else
  {
    max_dist_ += d_extend_;

    // parallel rewire optimization
    std::vector<int> v_iters;
    for (auto& p : sample_list_)
    {
      v_iters.push_back(p.first);
    }

#pragma omp parallel for num_threads(n_threads_)
    for (int i = 0; i < v_iters.size(); i++)
    {
      auto& p = sample_list_[v_iters[i]];

      // inside the optimization circle
      double new_dist = helper::dist(p, new_node);
      if (new_dist >= r_ || _isAnyObstacleInPath(new_node, p))
        continue;

#pragma omp critical
      // other thread may update new_node.g() or p.g()
      {
        double cost;
        // update new sample node's cost and parent
        cost = p.g() + new_dist;
        if (new_node.g() > cost)
        {
          new_node.set_pid(p.id());
          new_node.set_g(cost);
        }

        // update nodes' cost inside the radius
        cost = new_node.g() + new_dist;
        if (cost < p.g())
        {
          p.set_pid(new_node.id());
          p.set_g(cost);
        }
      }
    }
  }

  return new_node;
}
}  // namespace global_planner