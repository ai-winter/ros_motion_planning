/**
 * *********************************************************
 *
 * @file: astar_framework.h
 * @brief: A*-based searching framework
 * @author: Yang Haodong
 * @date: 2025-01-17
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "path_planner/graph_planner/hybrid_astar_planner/node_hybrid.h"
#include "path_planner/graph_planner/hybrid_astar_planner/astar_framework.h"

using namespace rmp::common::geometry;

namespace
{
constexpr int kGraphSizeDefault = 100000;
}

namespace rmp
{
namespace path_planner
{
template <typename NodeT>
AStarFramework<NodeT>::AStarFramework(const MotionModel& motion_model, const HybridSearchInfo& search_info)
  : is_initialized_(false)
  , size_x_(0)
  , size_y_(0)
  , goal_pose_(Pose())
  , start_(nullptr)
  , goal_(nullptr)
  , motion_model_(motion_model)
  , search_info_(search_info)
{
  graph_.reserve(kGraphSizeDefault);
}

template <typename NodeT>
AStarFramework<NodeT>::~AStarFramework()
{
}

template <typename NodeT>
void AStarFramework<NodeT>::initialize()
{
  if (!is_initialized_)
  {
    NodeT::precomputeDistanceHeuristic(motion_model_, search_info_);
  }
  is_initialized_ = true;
  expander_ = std::make_unique<AnalyticExpansion<NodeT>>(motion_model_, search_info_);
}

/**
 * @brief Sets the collision checker to use
 * @param collision_checker Collision checker to use for checking state validity
 */
template <typename NodeT>
void AStarFramework<NodeT>::setCollisionChecker(const std::shared_ptr<CollisionChecker>& collision_checker)
{
  collision_checker_ = collision_checker;
  costmap_ = collision_checker->getCostmapROS()->getCostmap();
  unsigned int size_x = costmap_->getSizeInCellsX();
  unsigned int size_y = costmap_->getSizeInCellsY();

  clearGraph();

  if (size_x_ != size_x || size_y_ != size_y)
  {
    size_x_ = size_x;
    size_y_ = size_y;
    NodeT::initMotionModel(motion_model_, size_x_, size_y_, search_info_);
  }
  expander_->setCollisionChecker(collision_checker_);
}

template <typename NodeT>
bool AStarFramework<NodeT>::createPath(Poses& path, Poses& expand)
{
  // initialization
  path.clear();
  expand.clear();
  best_heuristic_node_ = { std::numeric_limits<float>::max(), 0 };

  if (!areInputsValid())
  {
    return false;
  }

  // preallocate variables
  NodeVector neighbors;  // neighbors of current node
  NodePtr neighbor = nullptr;
  // int analytic_iterations = 0;
  // int closest_distance = std::numeric_limits<int>::max();

  // 0) Add starting point to the open set
  clearQueue();
  addNode(0.0, start_);
  start_->setAccumulatedCost(0.0);

  // Given an index, return a node ptr reference if its collision-free and valid
  const uint64_t max_index =
      static_cast<uint64_t>(size_x_) * static_cast<uint64_t>(size_y_) * static_cast<uint64_t>(search_info_.dim_3_size);
  NodeGetter neighborGetter = [&, this](const uint64_t& index, NodePtr& neighbor_rtn) -> bool {
    if (index >= max_index)
    {
      return false;
    }
    neighbor_rtn = addToGraph(index);
    return true;
  };

  // main loop
  int iterations = 0, approach_iterations = 0;
  while (iterations < search_info_.max_iterations && !queue_.empty())
  {
    // 1) Pick the best node (Nbest) from open list
    NodePtr current_node = getNextNode();

    // Save current node coordinates for debug
    updateExpand(current_node, expand);

    // Current node exists in closed list
    if (current_node->is_visited())
    {
      continue;
    }
    iterations++;

    // 2) Mark Nbest as visited
    current_node->visited();

    // 2.1) Use an analytic expansion (if available) to generate a path
    NodePtr expansion_result = nullptr;
    expansion_result = expander_->tryAnalyticExpansion(current_node, goal_);
    if (expansion_result != nullptr)
    {
      current_node = expansion_result;
    }

    // 3) Goal found
    if (current_node == goal_)
    {
      return current_node->backtracePath(path);
    }
    else if (best_heuristic_node_.first < search_info_.goal_tolerance)
    {  // near goal
      approach_iterations++;
      if (approach_iterations >= search_info_.max_approach_iterations)
      {
        return graph_.at(best_heuristic_node_.second).backtracePath(path);
      }
    }

    // 4) Expand neighbors of Nbest not visited
    neighbors.clear();
    current_node->getNeighbors(neighborGetter, collision_checker_, search_info_.traverse_unknown, neighbors);
    for (auto neighbor_iterator = neighbors.begin(); neighbor_iterator != neighbors.end(); ++neighbor_iterator)
    {
      neighbor = *neighbor_iterator;

      // 4.1) Compute the cost to go to this node
      double g_cost = current_node->accumulated_cost() + current_node->getTraversalCost(neighbor);

      // 4.2) If this is a lower cost than prior, we set this as the new cost and new approach
      if (g_cost < neighbor->accumulated_cost())
      {
        neighbor->setAccumulatedCost(g_cost);
        neighbor->parent = current_node;
        // 4.3) Add to queue with heuristic cost
        addNode(g_cost + 1.5 * getHeuristicCost(neighbor), neighbor);
      }
    }
  }

  // If we run out of search options, return the path that is closest, if within tolerance.
  if (best_heuristic_node_.first < search_info_.goal_tolerance)
  {
    return graph_.at(best_heuristic_node_.second).backtracePath(path);
  }

  return false;
}

/**
 * @brief Get size of graph in X
 * @return Size in X
 */
template <typename NodeT>
unsigned int& AStarFramework<NodeT>::size_x()
{
  return size_x_;
}

/**
 * @brief Get size of graph in Y
 * @return Size in Y
 */
template <typename NodeT>
unsigned int& AStarFramework<NodeT>::size_y()
{
  return size_y_;
}

/**
 * @brief Get pointer reference to starting node
 * @return Node pointer reference to starting node
 */
template <typename NodeT>
typename AStarFramework<NodeT>::NodePtr& AStarFramework<NodeT>::start()
{
  return start_;
}

/**
 * @brief Get pointer reference to goal node
 * @return Node pointer reference to goal node
 */
template <typename NodeT>
typename AStarFramework<NodeT>::NodePtr& AStarFramework<NodeT>::goal()
{
  return goal_;
}

/**
 * @brief Set the starting pose for planning, as a node index
 * @param pose the start pose
 */
template <typename NodeT>
void AStarFramework<NodeT>::setStart(const Pose& pose)
{
  start_ = addToGraph(NodeT::getIndex(pose));
}

/**
 * @brief Set the goal for planning, as a node index
 * @param pose the start pose
 */
template <typename NodeT>
void AStarFramework<NodeT>::setGoal(const Pose& pose)
{
  goal_ = addToGraph(NodeT::getIndex(pose));
  if (!search_info_.cache_obstacle_heuristic || goal_pose_ != pose)
  {
    if (!start_)
    {
      throw std::runtime_error("Start must be set before goal.");
    }

    NodeT::resetObstacleHeuristic(collision_checker_->getCostmapROS(), start_->pose().x(), start_->pose().y(), pose.x(),
                                  pose.y());
  }
  goal_pose_ = pose;
}

/**
 * @brief Adds node to graph
 * @param index Node index to add
 */
template <typename NodeT>
typename AStarFramework<NodeT>::NodePtr AStarFramework<NodeT>::addToGraph(const uint64_t& index)
{
  auto iter = graph_.find(index);
  if (iter != graph_.end())
  {
    return &(iter->second);
  }

  return &(graph_.emplace(index, NodeT(index)).first->second);
}

/**
 * @brief Clear graph of nodes searched
 */
template <typename NodeT>
void AStarFramework<NodeT>::clearGraph()
{
  Graph g;
  std::swap(graph_, g);
  graph_.reserve(kGraphSizeDefault);
}

/**
 * @brief Add a node to the open set
 * @param cost The cost to sort into the open set of the node
 * @param node Node pointer reference to add to open set
 */
template <typename NodeT>
void AStarFramework<NodeT>::addNode(const double& cost, NodePtr& node)
{
  NodeWrapper<NodeT> queued_node(node->index());
  queued_node.populateSearchNode(node);
  queue_.emplace(cost, queued_node);
}

/**
 * @brief Clear heuristic queue of nodes to search
 */
template <typename NodeT>
void AStarFramework<NodeT>::clearQueue()
{
  NodeQueue q;
  std::swap(queue_, q);
}

/**
 * @brief Check if inputs to planner are valid
 * @return Are valid
 */
template <typename NodeT>
bool AStarFramework<NodeT>::areInputsValid()
{
  // Check if graph was filled in
  if (graph_.empty())
  {
    throw std::runtime_error("Failed to compute path, no costmap given.");
  }

  // Check if points were filled in
  if (!start_ || !goal_)
  {
    throw std::runtime_error("Failed to compute path, no valid start or goal given.");
  }

  // Check if ending point is valid
  if (search_info_.goal_tolerance < 0.001 && !goal_->isValid(search_info_.traverse_unknown, collision_checker_))
  {
    throw std::runtime_error("Goal was in lethal cost");
  }

  return true;
}

/**
 * @brief Get pointer to next goal in open set
 * @return Node pointer reference to next heuristically scored node
 */
template <typename NodeT>
typename AStarFramework<NodeT>::NodePtr AStarFramework<NodeT>::getNextNode()
{
  NodeWrapper<NodeT> node = queue_.top().second;
  queue_.pop();
  node.processSearchNode();
  return node.graph_node_ptr;
}

/**
 * @brief Populate expansions for debug or visualization
 * @param node Node expanded
 * @param expand expansions zone
 */
template <typename NodeT>
void AStarFramework<NodeT>::updateExpand(const NodePtr& node, Poses& expand)
{
  expand.emplace_back(node->pose().x(), node->pose().y(), NodeT::motion_table.getAngleFromBin(node->pose().theta()));
}

/**
 * @brief Get cost of heuristic of node
 * @param node Node pointer to get heuristic for
 * @return Heuristic cost for node
 */
template <typename NodeT>
float AStarFramework<NodeT>::getHeuristicCost(const NodePtr& node)
{
  const Pose cur_pose = NodeT::getCoords(node->index());
  double heuristic = NodeT::getHeuristicCost(cur_pose, goal_pose_);

  if (heuristic < best_heuristic_node_.first)
  {
    best_heuristic_node_ = { heuristic, node->index() };
  }

  return heuristic;
}

// Instantiate algorithm for the supported template types
template class AStarFramework<NodeHybrid>;

}  // namespace path_planner
}  // namespace rmp