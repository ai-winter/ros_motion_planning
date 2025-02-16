/**
 * *********************************************************
 *
 * @file: quick_informed_rrt_star_planner.cpp
 * @brief: Contains the quick informed RRT* planner class
 * @author: Honbo He, Haodong Yang
 * @date: 2024-06-05
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <omp.h>
#include <random>

#include "common/geometry/collision_checker.h"
#include "path_planner/sample_planner/quick_informed_rrt_star_planner.h"

namespace rmp
{
namespace path_planner
{
namespace
{
using CollisionChecker = rmp::common::geometry::CollisionChecker;
}
/**
 * @brief  Constructor
 * @param   costmap   the environment for path planning
 * @param   obstacle_factor obstacle factor(greater means obstacles)
 * @param   sample_num  andom sample points
 * @param   max_dist    max distance between sample points
 * @param   r           optimization radius
 */
QuickInformedRRTStarPathPlanner::QuickInformedRRTStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros,
                                                                 double obstacle_factor, int sample_num,
                                                                 double max_dist, double r, double r_set, int n_threads,
                                                                 double d_extend, double t_freedom)
  : InformedRRTStarPathPlanner(costmap_ros, obstacle_factor, sample_num, max_dist, r)
  , set_r_(r_set)
  , rewire_threads_(n_threads)
  , step_extend_d_(d_extend)
  , recover_max_dist_(max_dist)
  , t_distr_freedom_(t_freedom)
{
}

/**
 * @brief Informed RRT* implementation
 * @param start     start node
 * @param goal      goal node
 * @param expand    containing the node been search during the process
 * @return tuple contatining a bool as to whether a path was found, and the path
 */
bool QuickInformedRRTStarPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  double m_start_x, m_start_y, m_goal_x, m_goal_y;
  if ((!validityCheck(start.x(), start.y(), m_start_x, m_start_y)) ||
      (!validityCheck(goal.x(), goal.y(), m_goal_x, m_goal_y)))
  {
    return false;
  }

  path.clear();
  expand.clear();

  // initialization
  c_best_ = std::numeric_limits<double>::max();
  c_min_ = std::hypot(m_start_x - m_goal_x, m_start_y - m_goal_y);
  int best_parent = -1;
  sample_list_.clear();

  // copy
  start_.set_x(m_start_x);
  start_.set_y(m_start_y);
  start_.set_id(grid2Index(start_.x(), start_.y()));
  goal_.set_x(m_goal_x);
  goal_.set_y(m_goal_y);
  goal_.set_id(grid2Index(goal_.x(), goal_.y()));
  sample_list_.insert(std::make_pair(start_.id(), start_));
  expand.emplace_back(m_start_x, m_start_y, 0);

  // adaptive sampling bias
  double dist_s2g = c_min_;
  double dist_m2g = dist_s2g * 0.95;
  // priority circles set
  double mu = 0.0;
  int mu_cnt = 0;

  // main loop
  int iteration = 0;
  std::vector<Node> nodes;
  while (iteration < sample_num_)
  {
    iteration++;

    // update probability of sampling bias
    opti_sample_p_ = std::min(0.75, 1 - dist_m2g / dist_s2g);

    // generate a random node in the map
    Node sample_node = _generateRandomNode(mu, nodes);

    // obstacle
    if (costmap_->getCharMap()[sample_node.id()] >= costmap_2d::LETHAL_OBSTACLE * obstacle_factor_)
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
      expand.emplace_back(new_node.x(), new_node.y(), new_node.pid());
    }

    auto dist_ = std::hypot(new_node.x() - goal_.x(), new_node.y() - goal_.y());
    // update min dist from tree to goal
    if (dist_ < dist_m2g)
      dist_m2g = dist_;

    // goal found
    auto isCollision = [&](const Node& node1, const Node& node2) {
      return CollisionChecker::BresenhamCollisionDetection(node1, node2, [&](const Node& node) {
        return costmap_->getCharMap()[grid2Index(node.x(), node.y())] >= costmap_2d::LETHAL_OBSTACLE * obstacle_factor_;
      });
    };
    if (dist_ <= max_dist_ && !isCollision(new_node, goal_))
    {
      double cost = dist_ + new_node.g();
      if (cost < c_best_)
      {
        best_parent = new_node.id();
        c_best_ = cost;
        // update path
        Node goal_star(goal_.x(), goal_.y(), c_best_, 0, grid2Index(goal_.x(), goal_.y()), best_parent);
        sample_list_.insert(std::make_pair(goal_star.id(), goal_star));
        nodes = _convertClosedListToPath<Node>(sample_list_, start_, goal_);
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

    const auto& backtrace = _convertClosedListToPath<Node>(sample_list_, start_, goal_);
    for (auto iter = backtrace.rbegin(); iter != backtrace.rend(); ++iter)
    {
      // convert to world frame
      double wx, wy;
      costmap_->mapToWorld(iter->x(), iter->y(), wx, wy);
      path.emplace_back(wx, wy);
    }
    return true;
  }

  return false;
}

/**
 * @brief Generates a random node
 * @return Generated node
 */
QuickInformedRRTStarPathPlanner::Node QuickInformedRRTStarPathPlanner::_generateRandomNode(int mu,
                                                                                           std::vector<Node> path)
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
      return RRTStarPathPlanner::_generateRandomNode();
  }
}

/**
 * @brief Regular the new node by the nearest node in the sample list
 * @param list     sample list
 * @param node     sample node
 * @return nearest node
 */
QuickInformedRRTStarPathPlanner::Node
QuickInformedRRTStarPathPlanner::_findNearestPoint(std::unordered_map<int, Node> list, Node& node)
{
  Node nearest_node, new_node(node);
  double min_dist = std::numeric_limits<double>::max();

  for (const auto p : list)
  {
    // calculate distance
    double new_dist = std::hypot(p.second.x() - new_node.x(), p.second.y() - new_node.y());

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
    double theta = std::atan2(new_node.y() - nearest_node.y(), new_node.x() - nearest_node.x());
    new_node.set_x(nearest_node.x() + (int)(max_dist_ * cos(theta)));
    new_node.set_y(nearest_node.y() + (int)(max_dist_ * sin(theta)));
    new_node.set_id(grid2Index(new_node.x(), new_node.y()));
    new_node.set_g(max_dist_ + nearest_node.g());
  }

  // obstacle check
  auto isCollision = [&](const Node& node1, const Node& node2) {
    return CollisionChecker::BresenhamCollisionDetection(node1, node2, [&](const Node& node) {
      return costmap_->getCharMap()[grid2Index(node.x(), node.y())] >= costmap_2d::LETHAL_OBSTACLE * obstacle_factor_;
    });
  };
  if (!isCollision(new_node, nearest_node))
  {
    max_dist_ += step_extend_d_;

    // parallel rewire optimization
    std::vector<int> v_iters;
    for (auto p : sample_list_)
    {
      v_iters.push_back(p.first);
    }

#pragma omp parallel for num_threads(rewire_threads_)
    for (size_t i = 0; i < v_iters.size(); i++)
    {
      auto& p = sample_list_[v_iters[i]];
      // inside the optimization circle
      double new_dist = std::hypot(p.x() - new_node.x(), p.y() - new_node.y());
      if (new_dist > r_)
        continue;

      double cost = p.g() + new_dist;
      // update new sample node's cost and parent
      if (new_node.g() > cost)
      {
        if (!isCollision(new_node, p))
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
          if (!isCollision(new_node, p))
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
}  // namespace path_planner
}  // namespace rmp