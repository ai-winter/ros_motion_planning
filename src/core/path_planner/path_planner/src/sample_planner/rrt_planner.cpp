/**
 * *********************************************************
 *
 * @file: rrt_planner.cpp
 * @brief: Contains the Rapidly-Exploring Random Tree (RRT) planner class
 * @author: Yang Haodong
 * @date: 2024-9-24
 * @version: 2.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <random>

#include "common/geometry/collision_checker.h"
#include "path_planner/sample_planner/rrt_planner.h"

using CollisionChecker = rmp::common::geometry::CollisionChecker;
using namespace rmp::common::geometry;

namespace rmp {
namespace path_planner {

/**
 * @brief  Constructor
 * @param   costmap   the environment for path planning
 */
RRTPathPlanner::RRTPathPlanner(costmap_2d::Costmap2DROS* costmap_ros)
  : PathPlanner(costmap_ros) {
}

/**
 * @brief RRT implementation
 * @param start         start node
 * @param goal          goal node
 * @param path          The resulting path in (x, y, theta)
 * @param expand        containing the node been search during the process
 * @return  true if path found, else false
 */
bool RRTPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d* path,
                          Points3d* expand) {
  double m_start_x, m_start_y, m_goal_x, m_goal_y;
  if ((!validityCheck(start.x(), start.y(), m_start_x, m_start_y)) ||
      (!validityCheck(goal.x(), goal.y(), m_goal_x, m_goal_y))) {
    return false;
  }

  path->clear();
  expand->clear();
  sample_list_.clear();

  // copy
  start_.set_x(m_start_x);
  start_.set_y(m_start_y);
  start_.set_id(grid2Index(start_.x(), start_.y()));
  goal_.set_x(m_goal_x);
  goal_.set_y(m_goal_y);
  goal_.set_id(grid2Index(goal_.x(), goal_.y()));
  sample_list_.insert(std::make_pair(start_.id(), start_));
  expand->emplace_back(m_start_x, m_start_y, 0);

  // main loop
  int iteration = 0;
  while (iteration < config_.sample_planner().sample_points()) {
    // generate a random node in the map
    Node sample_node = _generateRandomNode();

    // obstacle
    if (costmap_->getCharMap()[sample_node.id()] >=
        costmap_2d::LETHAL_OBSTACLE * config_.obstacle_inflation_factor()) {
      continue;
    }

    // visited
    if (sample_list_.find(sample_node.id()) != sample_list_.end()) {
      continue;
    }

    // regular the sample node
    Node new_node = _findNearestPoint(sample_list_, sample_node);

    if (new_node.id() == -1) {
      continue;
    } else {
      sample_list_.insert(std::make_pair(new_node.id(), new_node));
      expand->emplace_back(new_node.x(), new_node.y(), new_node.pid());
    }

    // goal found
    if (_checkGoal(new_node)) {
      const auto& backtrace = _convertClosedListToPath<Node>(sample_list_, start_, goal_);
      for (auto iter = backtrace.rbegin(); iter != backtrace.rend(); ++iter) {
        // convert to world frame
        double wx, wy;
        costmap_->mapToWorld(iter->x(), iter->y(), wx, wy);
        path->emplace_back(wx, wy);
      }
      return true;
    }
    iteration++;
  }
  return false;
}

/**
 * @brief Generates a random node
 * @return Generated node
 */
RRTPathPlanner::Node RRTPathPlanner::_generateRandomNode() {
  // obtain a random number from hardware
  std::random_device rd;
  // seed the generator
  std::mt19937 eng(rd());
  // define the range
  std::uniform_real_distribution<float> p(0, 1);
  // heuristic
  if (p(eng) > config_.sample_planner().optimization_sampe_probability()) {
    // generate node
    std::uniform_int_distribution<int> distr(0, map_size_ - 1);
    const int id = distr(eng);
    int x, y;
    index2Grid(id, x, y);
    return Node(x, y, 0, 0, id, 0);
  } else
    return Node(goal_.x(), goal_.y(), 0, 0, goal_.id(), 0);
}

/**
 * @brief Regular the sample node by the nearest node in the sample list
 * @param list  samplee list
 * @param node  sample node
 * @return nearest node
 */
RRTPathPlanner::Node
RRTPathPlanner::_findNearestPoint(std::unordered_map<int, Node>& list, const Node& node) {
  Node nearest_node, new_node(node);
  double min_dist = std::numeric_limits<double>::max();

  for (const auto& p : list) {
    // calculate distance
    double new_dist =
        std::hypot(p.second.x() - new_node.x(), p.second.y() - new_node.y());

    // update nearest node
    if (new_dist < min_dist) {
      nearest_node = p.second;
      new_node.set_pid(nearest_node.id());
      new_node.set_g(new_dist + p.second.g());
      min_dist = new_dist;
    }
  }

  // distance longer than the threshold
  const double max_dist = config_.sample_planner().sample_max_distance();
  if (min_dist > max_dist) {
    // connect sample node and nearest node, then move the nearest node
    // forward to sample node with `max_distance` as result
    double theta =
        std::atan2(new_node.y() - nearest_node.y(), new_node.x() - nearest_node.x());
    new_node.set_x(nearest_node.x() + static_cast<int>(max_dist * cos(theta)));
    new_node.set_y(nearest_node.y() + static_cast<int>(max_dist * sin(theta)));
    new_node.set_id(grid2Index(new_node.x(), new_node.y()));
    new_node.set_g(max_dist + nearest_node.g());
  }

  // obstacle check
  auto isCollision = [&](const Node& node1, const Node& node2) {
    return CollisionChecker::BresenhamCollisionDetection(
        node1, node2, [&](const Node& node) {
          return costmap_->getCharMap()[grid2Index(node.x(), node.y())] >=
                 costmap_2d::LETHAL_OBSTACLE * config_.obstacle_inflation_factor();
        });
  };

  if (isCollision(new_node, nearest_node)) {
    new_node.set_id(-1);
  }

  return new_node;
}

/**
 * @brief Check if goal is reachable from current node
 * @param new_node Current node
 * @return bool value of whether goal is reachable from current node
 */
bool RRTPathPlanner::_checkGoal(const Node& new_node) {
  auto dist_ = std::hypot(new_node.x() - goal_.x(), new_node.y() - goal_.y());
  if (dist_ > config_.sample_planner().sample_max_distance())
    return false;

  auto isCollision = [&](const Node& node1, const Node& node2) {
    return CollisionChecker::BresenhamCollisionDetection(
        node1, node2, [&](const Node& node) {
          return costmap_->getCharMap()[grid2Index(node.x(), node.y())] >=
                 costmap_2d::LETHAL_OBSTACLE * config_.obstacle_inflation_factor();
        });
  };

  if (!isCollision(new_node, goal_)) {
    Node goal(goal_.x(), goal_.y(), dist_ + new_node.g(), 0,
              grid2Index(goal_.x(), goal_.y()), new_node.id());
    sample_list_.insert(std::make_pair(goal.id(), goal));
    return true;
  }
  return false;
}
}  // namespace path_planner
}  // namespace rmp
