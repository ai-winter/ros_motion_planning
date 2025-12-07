/**
 * *********************************************************
 *
 * @file: hybrid_astar_planner.cpp
 * @brief: Contains the Hybrid A* planner class
 * @author: Yang Haodong
 * @date: 2025-05-04
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "common/util/visualizer.h"
#include "common/math/math_helper.h"
#include "common/geometry/collision_checker.h"
#include "path_planner/graph_planner/hybrid_astar_planner/hybrid_astar_planner.h"

using namespace rmp::common::util;
using namespace rmp::common::math;
using namespace rmp::common::geometry;
using namespace rmp::common::structure;

namespace rmp {
namespace path_planner {
// defining static member for all instance to share
HybridAStarMotionTable HybridAStarPathPlanner::motion_table_;
std::vector<Node<int>> HybridAStarPathPlanner::grid_motions_ = {
  { 0, 1, 1.0 },           { 1, 0, 1.0 },
  { 0, -1, 1.0 },          { -1, 0, 1.0 },
  { 1, 1, std::sqrt(2) },  { 1, -1, std::sqrt(2) },
  { -1, 1, std::sqrt(2) }, { -1, -1, std::sqrt(2) },
};

/**
 * @brief Construct a new Hybrid A* object
 * @param costmap   the environment for path planning
 */
HybridAStarPathPlanner::HybridAStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros)
  : PathPlanner(costmap_ros) {
  hybrid_astar_config_ = config_.graph_planner().hybrid_astar_planner();
  const double minimum_turning_info_in_pixel =
      hybrid_astar_config_.minimum_turning_radius() /
      costmap_ros->getCostmap()->getResolution();
  if (hybrid_astar_config_.motion_model() ==
      pb::path_planner::MotionModel::DUBINS_UNSPECIFIED) {
    motion_table_.initDubins(hybrid_astar_config_.dim_3_size(),
                             minimum_turning_info_in_pixel, costmap_->getSizeInCellsX(),
                             hybrid_astar_config_.curve_sample_ratio(),
                             hybrid_astar_config_.change_penalty(),
                             hybrid_astar_config_.non_straight_penalty(),
                             hybrid_astar_config_.reverse_penalty(),
                             hybrid_astar_config_.retrospective_penalty());
  } else {
    R_ERROR << "Invalid motion model for HybridAStarPathPlanner";
  }
}

/**
 * @brief Creates a path between the start and goal points while expanding nodes.
 * @param start    The starting node in (x, y, theta).
 * @param goal     The goal node in (x, y, theta).
 * @param path     The resulting path in (x, y, theta).
 * @param expand   The expanded nodes visited during the search.
 * @return true if a path is created successfully, false otherwise.
 */
bool HybridAStarPathPlanner::plan(const Point3d& start, const Point3d& goal,
                                  Points3d* path, Points3d* expand) {
  double m_start_x, m_start_y, m_goal_x, m_goal_y;
  if ((!validityCheck(start.x(), start.y(), m_start_x, m_start_y)) ||
      (!validityCheck(goal.x(), goal.y(), m_goal_x, m_goal_y))) {
    return false;
  }
  path->clear();
  expand->clear();

  if ((!last_path_.empty()) && (goal_ == goal)) {
    bool is_collision = false;
    Points3d::iterator closest_iter;
    double min_dist = std::numeric_limits<double>::max();
    for (auto iter = last_path_.begin(); iter != last_path_.end(); ++iter) {
      unsigned int mx, my;
      costmap_->worldToMap((*iter).x(), (*iter).y(), mx, my);
      if (isCollision(grid2Index(mx, my))) {
        is_collision = true;
        break;
      }
      double dist = std::hypot((*iter).x() - start.x(), (*iter).y() - start.y());
      if (dist < min_dist) {
        min_dist = dist;
        closest_iter = iter;
      }
    }
    if (!is_collision) {
      for (auto iter = closest_iter; iter != last_path_.end(); ++iter) {
        path->emplace_back((*iter).x(), (*iter).y(), (*iter).theta());
      }
      last_path_ = *path;
      return true;
    } else {
      last_path_.clear();
    }
  }

  goal_ = goal;
  Points3d path_in_map;
  if (createPath({ m_start_x, m_start_y,
                   static_cast<double>(motion_table_.getOrientationBin(start.theta())) },
                 { m_goal_x, m_goal_y,
                   static_cast<double>(motion_table_.getOrientationBin(goal.theta())) },
                 &path_in_map, expand)) {
    for (auto iter = path_in_map.rbegin(); iter != path_in_map.rend(); ++iter) {
      // convert to world frame
      double wx, wy;
      costmap_->mapToWorld(iter->x(), iter->y(), wx, wy);
      path->emplace_back(wx, wy, iter->theta());
    }
    last_path_ = *path;
    return true;
  }

  return false;
}

/**
 * @brief Creates a path between the start and goal points while expanding nodes.
 * @param start    The starting node in (x, y, theta).
 * @param goal     The goal node in (x, y, theta).
 * @param path     The resulting path in (x, y, theta).
 * @param expand   The expanded nodes visited during the search.
 * @return true if a path is created successfully, false otherwise.
 */
bool HybridAStarPathPlanner::createPath(const Point3d& start, const Point3d& goal,
                                        Points3d* path, Points3d* expand) {
  clearGraph();
  clearQueue();
  best_heuristic_node_ = { std::numeric_limits<float>::max(), 0 };
  auto start_node = addToGraph(getIndex(start));
  auto goal_node = addToGraph(getIndex(goal));

  precomputeObstacleHeuristic(goal_node);

  // 0) Add starting point to the open set
  addToQueue(0.0, start_node);
  start_node->setAccumulatedCost(0.0);
  std::vector<NodeHybrid::NodePtr> neighbors;  // neighbors of current node
  NodeHybrid::NodePtr neighbor = nullptr;

  // main loop
  int iterations = 0, approach_iterations = 0;
  while (iterations < hybrid_astar_config_.max_iterations() && !queue_.empty()) {
    // 1) Pick the best node (Nbest) from open list
    NodeHybrid::NodePtr current_node = queue_.top().second;
    queue_.pop();

    // Save current node coordinates for debug
    expand->emplace_back(current_node->pose().x(), current_node->pose().y(),
                         motion_table_.getAngleFromBin(current_node->pose().theta()));

    // Current node exists in closed list
    if (current_node->is_visited()) {
      continue;
    }
    iterations++;

    // 2) Mark Nbest as visited
    current_node->visited();

    // 2.1) Use an analytic expansion (if available) to generate a path
    NodeHybrid::NodePtr expansion_result = tryAnalyticExpansion(current_node, goal_node);
    if (expansion_result != nullptr) {
      current_node = expansion_result;
    }

    // 3) Goal found
    if (current_node == goal_node) {
      return backtracePath(current_node, path);
    } else if (best_heuristic_node_.first <
               hybrid_astar_config_.goal_tolerance()) {  // near goal
      approach_iterations++;
      if (approach_iterations >= hybrid_astar_config_.max_approach_iterations()) {
        NodeHybrid::NodePtr node_ptr = &(graph_.at(best_heuristic_node_.second));
        return backtracePath(node_ptr, path);
      }
    }

    // 4) Expand neighbors of Nbest not visited
    neighbors.clear();
    getNeighbors(current_node, neighbors);
    for (auto neighbor_iterator = neighbors.begin(); neighbor_iterator != neighbors.end();
         ++neighbor_iterator) {
      neighbor = *neighbor_iterator;

      // 4.1) Compute the cost to go to this node
      double g_cost = current_node->accumulated_cost() +
                      current_node->getTraversalCost(neighbor, motion_table_);

      // 4.2) If this is a lower cost than prior, we set this as the new cost and new
      // approach
      if (g_cost < neighbor->accumulated_cost()) {
        neighbor->setAccumulatedCost(g_cost);
        neighbor->parent = current_node;
        // 4.3) Add to queue with heuristic cost
        addToQueue(g_cost + hybrid_astar_config_.lamda_h() *
                                getHeuristicCost(neighbor, goal_node),
                   neighbor);
      }
    }
  }

  // If we run out of search options, return the path that is closest, if within
  // tolerance.
  if (best_heuristic_node_.first < hybrid_astar_config_.goal_tolerance()) {
    NodeHybrid::NodePtr node_ptr = &(graph_.at(best_heuristic_node_.second));
    return backtracePath(node_ptr, path);
  }

  return false;
}

/**
 * @brief Computes the combined heuristic cost (obstacle and distance) for the given node.
 * @param node The current node.
 * @param goal The goal node.
 * @return The heuristic cost for the given node.
 */
double HybridAStarPathPlanner::getHeuristicCost(const NodeHybrid::NodePtr& node,
                                                const NodeHybrid::NodePtr& goal) const {
  return std::max(getObstacleHeuristic(node), getDistanceHeuristic(node, goal));
}

/**
 * @brief Computes the obstacle heuristic cost for the given node.
 * @param node The current node.
 * @return The obstacle heuristic cost.
 */
double
HybridAStarPathPlanner::getObstacleHeuristic(const NodeHybrid::NodePtr& node) const {
  const int x = static_cast<int>(node->pose().x());
  const int y = static_cast<int>(node->pose().y());
  const int height = costmap_->getSizeInCellsY();
  const int width = costmap_->getSizeInCellsX();

  if (x < 0 || x >= width || y < 0 || y >= height) {
    R_ERROR << "Position at " << x << ", " << y << " is out of the map.";
  }

  return obstacle_hmap_[y][x];
}

/**
 * @brief Computes the distance heuristic cost between the given node and the goal.
 * @param node The current node.
 * @param goal The goal node.
 * @return The distance heuristic cost.
 */
double HybridAStarPathPlanner::getDistanceHeuristic(
    const NodeHybrid::NodePtr& node, const NodeHybrid::NodePtr& goal) const {
  Points3d motion_path;
  Point3d from(node->pose().x(), node->pose().y(),
               motion_table_.getAngleFromBin(node->pose().theta()));
  Point3d to(goal->pose().x(), goal->pose().y(),
             motion_table_.getAngleFromBin(goal->pose().theta()));
  if (motion_table_.curve_gen->generation(from, to, motion_path)) {
    double dist = 0.0;
    for (size_t i = 0; i < motion_path.size() - 1; ++i) {
      dist += std::hypot(motion_path[i].x() - motion_path[i + 1].x(),
                         motion_path[i].y() - motion_path[i + 1].y());
    }
    return dist;
  } else {
    R_WARN << "Heuristic curve generation failed.";
    return -1;
  }
}

/**
 * @brief Precomputes the obstacle heuristic values for the entire costmap based on the
 * goal position.
 * @param goal The goal node.
 * @return true if the heuristic map is successfully computed, false otherwise.
 */
bool HybridAStarPathPlanner::precomputeObstacleHeuristic(
    const NodeHybrid::NodePtr& goal) {
  const int goal_x = static_cast<int>(goal->pose().x());
  const int goal_y = static_cast<int>(goal->pose().y());
  const int height = costmap_->getSizeInCellsY();
  const int width = costmap_->getSizeInCellsX();
  const int idx = grid2Index(goal->pose().x(), goal->pose().y());

  if (isCollision(goal->pose())) {
    R_ERROR << "Goal not in free space";
    return false;
  }

  // initialize heurisitics
  obstacle_hmap_.clear();
  obstacle_hmap_.resize(height);
  for (auto& row : obstacle_hmap_) {
    row.resize(width, std::numeric_limits<double>::infinity());
  }
  obstacle_hmap_[goal_y][goal_x] = 0.0;

  // dijkstra queue
  using QueueElement = std::pair<float, std::pair<int, int>>;
  std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<>> queue;
  queue.emplace(0.0f, std::make_pair(goal_y, goal_x));

  // main loop
  while (!queue.empty()) {
    const auto node = queue.top();
    float curr_cost = node.first;
    int y = node.second.first;
    int x = node.second.second;
    queue.pop();

    for (const auto& motion : grid_motions_) {
      const int ny = y + motion.y();
      const int nx = x + motion.x();

      if (nx < 0 || nx >= width || ny < 0 || ny >= height)
        continue;

      if (isCollision({ static_cast<double>(nx), static_cast<double>(ny) }))
        continue;

      const double new_cost = curr_cost + motion.g();

      if (new_cost < obstacle_hmap_[ny][nx]) {
        obstacle_hmap_[ny][nx] = new_cost;
        queue.emplace(new_cost, std::make_pair(ny, nx));
      }
    }
  }

  for (auto& row : obstacle_hmap_) {
    for (auto& val : row) {
      if (std::isinf(val)) {
        val = -1.0;
      }
    }
  }
  return true;
}

/**
 * @brief Attempts an analytic expansion to connect the current node to the goal directly.
 * @param node The current node.
 * @param goal The goal node.
 * @return A pointer to the goal node if the expansion is successful, nullptr otherwise.
 */
NodeHybrid::NodePtr HybridAStarPathPlanner::tryAnalyticExpansion(
    const NodeHybrid::NodePtr& node, const NodeHybrid::NodePtr& goal) {
  // too far to expand
  if (getHeuristicCost(node, goal) >
      hybrid_astar_config_.analytic_expansion_max_length() / costmap_->getResolution()) {
    return nullptr;
  }

  Points3d motion_path;
  Point3d from(node->pose().x(), node->pose().y(),
               motion_table_.getAngleFromBin(node->pose().theta()));
  Point3d to(goal->pose().x(), goal->pose().y(),
             motion_table_.getAngleFromBin(goal->pose().theta()));
  if (motion_table_.curve_gen->generation(from, to, motion_path)) {
    expansions_node_.clear();
    NodeHybrid::NodePtr prev = node;
    for (size_t i = 1; i < motion_path.size() - 1; i++) {
      auto& pose = motion_path[i];
      pose.setTheta(motion_table_.getOrientationBin(pose.theta()));
      if (isCollision(pose)) {
        return nullptr;
      }
      NodeHybrid::NodePtr n = new NodeHybrid(getIndex(pose));
      n->setPose(pose);
      n->parent = prev;
      n->visited();
      expansions_node_.push_back(n);
      prev = n;
    }
    goal->parent = prev;
    goal->visited();
    return goal;
  }

  return nullptr;
}

/**
 * @brief Backtraces from the given node to reconstruct the path.
 * @param node The final node in the path.
 * @param path The reconstructed path consisting of 3D points (x, y, theta).
 * @return true if the path is successfully reconstructed, false otherwise.
 */
bool HybridAStarPathPlanner::backtracePath(NodeHybrid::NodePtr& node, Points3d* path) {
  if (!node->parent) {
    return false;
  }

  NodeHybrid::NodePtr current_node = node;
  while (current_node->parent) {
    path->push_back(current_node->pose());
    // Convert angle to radians
    path->back().setTheta(motion_table_.getAngleFromBin(path->back().theta()));
    current_node = current_node->parent;
  }

  // add the start pose
  path->push_back(current_node->pose());
  // Convert angle to radians
  path->back().setTheta(motion_table_.getAngleFromBin(path->back().theta()));
  return true;
}

/**
 * @brief Retrieves the neighboring nodes of the given node.
 * @param node      The current node.
 * @param neighbors A vector to store the neighboring nodes.
 */
void HybridAStarPathPlanner::getNeighbors(const NodeHybrid::NodePtr& node,
                                          std::vector<NodeHybrid::NodePtr>& neighbors) {
  const uint64_t max_index = static_cast<uint64_t>(costmap_->getSizeInCellsX()) *
                             static_cast<uint64_t>(costmap_->getSizeInCellsY()) *
                             static_cast<uint64_t>(hybrid_astar_config_.dim_3_size());
  NodeHybrid::NodePtr neighbor = nullptr;
  const auto& projections = motion_table_.getMotionPrimitives(node->pose());
  for (size_t i = 0; i < projections.size(); ++i) {
    Point3d new_pose(projections[i].x(), projections[i].y(), projections[i].theta());
    const int index = getIndex(new_pose);
    if (index < max_index) {
      neighbor = addToGraph(index);
      if (!neighbor->is_visited()) {
        neighbor->setPose(new_pose);
        if (!isCollision(neighbor->pose())) {
          neighbor->setMotionPrimitiveIndex(i, projections[i].turn_dir());
          neighbors.push_back(neighbor);
        }
      }
    }
  }
}

/**
 * @brief Checks if the given pose is in collision with any obstacles.
 * @param pose The pose to check in (x, y, theta).
 * @return true if the pose is in collision, false otherwise.
 */
bool HybridAStarPathPlanner::isCollision(const Point3d& pose) {
  return collision_checker_->inCollision(pose.x(), pose.y(), true);
}

/**
 * @brief Adds a node to the graph of visited nodes.
 * @param index The index of the node in the graph.
 * @return A pointer to the added node.
 */
NodeHybrid::NodePtr HybridAStarPathPlanner::addToGraph(const uint64_t& index) {
  auto iter = graph_.find(index);
  if (iter != graph_.end()) {
    return &(iter->second);
  }
  auto node = NodeHybrid(index);
  node.setPose(getPose(index));
  return &(graph_.emplace(index, node).first->second);
}

/**
 * @brief Clear graph of nodes searched
 */
void HybridAStarPathPlanner::clearGraph() {
  Graph g;
  std::swap(graph_, g);
  graph_.reserve(hybrid_astar_config_.default_graph_size());
}

/**
 * @brief Add a node to the open set
 * @param cost The cost to sort into the open set of the node
 * @param node Node pointer reference to add to open set
 */
void HybridAStarPathPlanner::addToQueue(const double& cost, NodeHybrid::NodePtr& node) {
  queue_.emplace(QueueNode(cost, node));
}

/**
 * @brief Clear heuristic queue of nodes to search
 */
void HybridAStarPathPlanner::clearQueue() {
  Queue q;
  std::swap(queue_, q);
}

/**
 * @brief Checks if the given node has reached the goal.
 * @param node The current node.
 * @param goal The goal node.
 * @return true if the node is within the goal tolerance, false otherwise.
 */
bool HybridAStarPathPlanner::isReachGoal(const NodeHybrid::NodePtr& node,
                                         const NodeHybrid::NodePtr& goal) const {
  const double dx = node->pose().x() - goal->pose().x();
  const double dy = node->pose().y() - goal->pose().y();
  const double dtheta = node->pose().theta() - goal->pose().theta();
  return (dx * dx + dy * dy + dtheta * dtheta) < kMathEpsilon;
}
}  // namespace path_planner
}  // namespace rmp