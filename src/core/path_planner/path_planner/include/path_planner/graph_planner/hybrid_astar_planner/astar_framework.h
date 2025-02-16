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
#ifndef RMP_PATH_PLANNER_UTIL_ASTAR_FRAMEWORK_H
#define RMP_PATH_PLANNER_UTIL_ASTAR_FRAMEWORK_H

#include <queue>
#include <vector>
#include <unordered_map>

#include "common/geometry/collision_checker.h"

#include "path_planner/utils/motions.h"
#include "path_planner/graph_planner/hybrid_astar_planner/node_wrapper.h"
#include "path_planner/graph_planner/hybrid_astar_planner/analytic_expansion.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief An A* implementation for planning in a costmap.
 */
template <typename NodeT>
class AStarFramework
{
public:
  typedef NodeT* NodePtr;
  typedef std::unordered_map<uint64_t, NodeT> Graph;
  typedef std::vector<NodePtr> NodeVector;
  typedef std::pair<float, NodeWrapper<NodeT>> NodeElement;
  typedef typename NodeT::Pose Pose;
  typedef typename NodeT::Poses Poses;
  typedef std::function<bool(const uint64_t&, NodeT*&)> NodeGetter;

  /**
   * @brief Node comparison for priority queue sorting
   */
  struct NodeComparator
  {
    bool operator()(const NodeElement& a, const NodeElement& b) const
    {
      return a.first > b.first;
    }
  };

  typedef std::priority_queue<NodeElement, std::vector<NodeElement>, NodeComparator> NodeQueue;

  /**
   * @brief A constructor for AStarFramework
   */
  explicit AStarFramework(const MotionModel& motion_model, const HybridSearchInfo& search_info);

  /**
   * @brief A destructor for AStarFramework
   */
  ~AStarFramework();

  /**
   * @brief Initialization of the planner with defaults
   */
  void initialize();

  /**
   * @brief Sets the collision checker to use
   * @param collision_checker Collision checker to use for checking state validity
   */
  void setCollisionChecker(const std::shared_ptr<rmp::common::geometry::CollisionChecker>& collision_checker);

  /**
   * @brief Creating path from given costmap, start, and goal
   * @param path Reference to a vector of indices of generated path
   * @param expand expansions for debug or visualization
   * @return if plan was successful
   */
  bool createPath(Poses& path, Poses& expand);

  /**
   * @brief Get size of graph in X
   * @return Size in X
   */
  unsigned int& size_x();

  /**
   * @brief Get size of graph in Y
   * @return Size in Y
   */
  unsigned int& size_y();

  /**
   * @brief Get pointer reference to starting node
   * @return Node pointer reference to starting node
   */
  NodePtr& start();

  /**
   * @brief Get pointer reference to goal node
   * @return Node pointer reference to goal node
   */
  NodePtr& goal();

  /**
   * @brief Set the starting pose for planning, as a node index
   * @param pose the start pose
   */
  void setStart(const Pose& pose);

  /**
   * @brief Set the goal for planning, as a node index
   * @param pose the start pose
   */
  void setGoal(const Pose& pose);

protected:
  /**
   * @brief Adds node to graph
   * @param index Node index to add
   */
  inline NodePtr addToGraph(const uint64_t& index);

  /**
   * @brief Clear graph of nodes searched
   */
  inline void clearGraph();

  /**
   * @brief Add a node to the open set
   * @param cost The cost to sort into the open set of the node
   * @param node Node pointer reference to add to open set
   */
  inline void addNode(const double& cost, NodePtr& node);

  /**
   * @brief Clear heuristic queue of nodes to search
   */
  inline void clearQueue();

  /**
   * @brief Check if inputs to planner are valid
   * @return Are valid
   */
  inline bool areInputsValid();

  /**
   * @brief Get pointer to next goal in open set
   * @return Node pointer reference to next heuristically scored node
   */
  inline NodePtr getNextNode();

  /**
   * @brief Populate expansions for debug or visualization
   * @param node Node expanded
   * @param expand expansions zone
   */
  inline void updateExpand(const NodePtr& node, Poses& expand);

  /**
   * @brief Get cost of heuristic of node
   * @param node Node pointer to get heuristic for
   * @return Heuristic cost for node
   */
  inline float getHeuristicCost(const NodePtr& node);

private:
  bool is_initialized_;           // framework initialization flag
  unsigned int size_x_;           // width of map
  unsigned int size_y_;           // height of map
  Pose goal_pose_;                // history goal pose
  NodePtr start_;                 // start pose
  NodePtr goal_;                  // goal pose
  MotionModel motion_model_;      // the allowed motions for the node
  HybridSearchInfo search_info_;  // search properties and penalties

  Graph graph_;                     // closed list
  NodeQueue queue_;                 // open list
  costmap_2d::Costmap2D* costmap_;  // costmap ptr

  std::pair<float, uint64_t> best_heuristic_node_;
  std::shared_ptr<rmp::common::geometry::CollisionChecker> collision_checker_;
  std::unique_ptr<AnalyticExpansion<NodeT>> expander_;
};
}  // namespace path_planner
}  // namespace rmp

#endif