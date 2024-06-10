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
#include <cmath>
#include <random>
#include <omp.h>
#include "quick_informed_rrt.h"

namespace global_planner
{
/**
 * @brief  Constructor
 * @param   costmap   the environment for path planning
 * @param   max_dist    max distance between sample points
 */
QuickInformedRRT::QuickInformedRRT(costmap_2d::Costmap2D* costmap, int sample_num, double max_dist, double r,
                                   double r_set, int n_threads, double d_extend, double t_freedom)
  : InformedRRT(costmap, sample_num, max_dist, r)
{
  set_r_ = r_set;
  rewire_threads_ = n_threads;
  step_extend_d_ = d_extend;
  recover_max_dist_ = max_dist;
  t_distr_freedom_ = t_freedom;
}

/**
 * @brief Informed RRT* implementation
 * @param start     start node
 * @param goal      goal node
 * @param expand    containing the node been search during the process
 * @return tuple contatining a bool as to whether a path was found, and the path
 */
bool QuickInformedRRT::plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand)
{
  // initialization
  c_best_ = std::numeric_limits<double>::max();
  c_min_ = helper::dist(start, goal);
  int best_parent = -1;
  sample_list_.clear();
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
    iteration++;

    // update probability of sampling bias
    opti_sample_p_ = std::min(0.75, 1 - dist_m2g / dist_s2g);

    // generate a random node in the map
    Node sample_node = _generateRandomNode(mu, path);

    // obstacle
    if (costmap_->getCharMap()[sample_node.id()] >= costmap_2d::LETHAL_OBSTACLE * factor_)
      continue;

    // visited
    if (sample_list_.find(sample_node.id()) != sample_list_.end())
      continue;

    // regular the sample node
    Node new_node = _findNearestPoint(sample_list_, sample_node);
    if (new_node.id() == -1)
      continue;
    else
    {
      sample_list_.insert(std::make_pair(new_node.id(), new_node));
      expand.push_back(new_node);
    }

    auto dist_ = helper::dist(new_node, goal_);
    // update min dist from tree to goal
    if (dist_ < dist_m2g)
      dist_m2g = dist_;
    // goal found
    if (dist_ <= max_dist_ && !_isAnyObstacleInPath(new_node, goal_))
    {
      double cost = dist_ + new_node.g();
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
 * @return Generated node
 */
Node QuickInformedRRT::_generateRandomNode(int mu, std::vector<Node> path)
{
  std::random_device rd;
  std::mt19937 eng(rd());
  std::student_t_distribution<> t_distr(t_distr_freedom_);

  if (std::abs(t_distr(eng)) < mu && path.size() != 0)  // sample in priority circles
  {
    int wc = rand() % path.size();
    std::uniform_real_distribution<float> p(-set_r_, set_r_);
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
 * @param list     sample list
 * @param node     sample node
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

  // obstacle check
  if (!_isAnyObstacleInPath(new_node, nearest_node))
  {
    max_dist_ += step_extend_d_;

    // parallel rewire optimization
    std::vector<int> v_iters;
    for (auto p : sample_list_)
    {
      v_iters.push_back(p.first);
    }

#pragma omp parallel for num_threads(rewire_threads_)
    for (int i = 0; i < v_iters.size(); i++)
    {
      auto& p = sample_list_[v_iters[i]];
      // inside the optimization circle
      double new_dist = helper::dist(p, new_node);
      if (new_dist > r_)
        continue;

      double cost = p.g() + new_dist;
      // update new sample node's cost and parent
      if (new_node.g() > cost)
      {
        if (!_isAnyObstacleInPath(new_node, p))
        {
          // other thread may update new_node.g()
#pragma omp critical
          if (new_node.g() > cost)
          {
            new_node.set_pid(p.id());
            new_node.set_g(cost);
          }
        }
      }
      else
      {
        // update nodes' cost inside the radius
        cost = new_node.g() + new_dist;
        if (cost < p.g())
          if (!_isAnyObstacleInPath(new_node, p))
          {
            p.set_pid(new_node.id());
            p.set_g(cost);
          }
      }
    }
  }
  else
  {
    max_dist_ = recover_max_dist_;
    new_node.set_id(-1);
  }
  return new_node;
}
}  // namespace global_planner