/**
 * *********************************************************
 *
 * @file: hybrid_astar_planner.h
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
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_HYBRID_ASTAR_H_
#define RMP_PATH_PLANNER_GRAPH_PLANNER_HYBRID_ASTAR_H_

#include "path_planner/path_planner.h"
#include "path_planner/graph_planner/hybrid_astar_planner/node_hybrid.h"
#include "system_config/path_planner_protos/graph_planner/hybrid_astar_planner.pb.h"

namespace rmp {
namespace path_planner {
/**
 * @brief Class for objects that plan using the A* algorithm
 */
class HybridAStarPathPlanner : public PathPlanner {
public:
  using QueueNode = std::pair<double, NodeHybrid::NodePtr>;

  /**
   * @brief Node comparison for priority queue sorting
   */
  struct NodeComparator {
    bool operator()(const QueueNode& a, const QueueNode& b) const {
      return a.first > b.first;
    }
  };

  using Queue = std::priority_queue<QueueNode, std::vector<QueueNode>, NodeComparator>;
  using Graph = std::unordered_map<uint64_t, NodeHybrid>;

public:
  /**
   * @brief Construct a new Hybrid A* object
   * @param costmap   the environment for path planning
   */
  HybridAStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Destory the Hybrid A* object
   */
  ~HybridAStarPathPlanner() = default;

  /**
   * @brief Creates a path between the start and goal points while expanding nodes.
   * @param start    The starting node in (x, y, theta).
   * @param goal     The goal node in (x, y, theta).
   * @param path     The resulting path in (x, y, theta).
   * @param expand   The expanded nodes visited during the search.
   * @return true if a path is created successfully, false otherwise.
   */
  bool plan(const common::geometry::Point3d& start, const common::geometry::Point3d& goal,
            common::geometry::Points3d* path, common::geometry::Points3d* expand);

  /**
   * @brief Get pose
   * @param Index Index of point
   * @return coordinates of point
   */
  static inline common::geometry::Point3d getPose(const uint64_t& index) {
    return common::geometry::Point3d(
        (index / motion_table_.num_angle_quantization) % motion_table_.map_width,  // x
        index / (motion_table_.num_angle_quantization * motion_table_.map_width),  // y
        index % motion_table_.num_angle_quantization);  // theta
  }

  /**
   * @brief Get index in <x, y, theta>
   * @param pose given pose to get index of
   * @return index
   */
  static inline uint64_t getIndex(const common::geometry::Point3d& pose) {
    return static_cast<uint64_t>(pose.theta()) +
           static_cast<uint64_t>(pose.x()) *
               static_cast<uint64_t>(motion_table_.num_angle_quantization) +
           static_cast<uint64_t>(pose.y()) *
               static_cast<uint64_t>(motion_table_.map_width) *
               static_cast<uint64_t>(motion_table_.num_angle_quantization);
  }

protected:
  /**
   * @brief Creates a path between the start and goal points while expanding nodes.
   * @param start    The starting node in (x, y, theta).
   * @param goal     The goal node in (x, y, theta).
   * @param path     The resulting path in (x, y, theta).
   * @param expand   The expanded nodes visited during the search.
   * @return true if a path is created successfully, false otherwise.
   */
  bool createPath(const common::geometry::Point3d& start,
                  const common::geometry::Point3d& goal, common::geometry::Points3d* path,
                  common::geometry::Points3d* expand);

  /**
   * @brief Computes the combined heuristic cost (obstacle and distance) for the given
   * node.
   * @param node The current node.
   * @param goal The goal node.
   * @return The heuristic cost for the given node.
   */
  double getHeuristicCost(const NodeHybrid::NodePtr& node,
                          const NodeHybrid::NodePtr& goal) const;

  /**
   * @brief Computes the obstacle heuristic cost for the given node.
   * @param node The current node.
   * @return The obstacle heuristic cost.
   */
  double getObstacleHeuristic(const NodeHybrid::NodePtr& node) const;

  /**
   * @brief Computes the distance heuristic cost between the given node and the goal.
   * @param node The current node.
   * @param goal The goal node.
   * @return The distance heuristic cost.
   */
  double getDistanceHeuristic(const NodeHybrid::NodePtr& node,
                              const NodeHybrid::NodePtr& goal) const;

  /**
   * @brief Precomputes the obstacle heuristic values for the entire costmap based on the
   * goal position.
   * @param goal The goal node.
   * @return true if the heuristic map is successfully computed, false otherwise.
   */
  bool precomputeObstacleHeuristic(const NodeHybrid::NodePtr& goal);

  /**
   * @brief Attempts an analytic expansion to connect the current node to the goal
   * directly.
   * @param node The current node.
   * @param goal The goal node.
   * @return A pointer to the goal node if the expansion is successful, nullptr otherwise.
   */
  NodeHybrid::NodePtr tryAnalyticExpansion(const NodeHybrid::NodePtr& node,
                                           const NodeHybrid::NodePtr& goal);

  /**
   * @brief Backtraces from the given node to reconstruct the path.
   * @param node The final node in the path.
   * @param path The reconstructed path consisting of 3D points (x, y, theta).
   * @return true if the path is successfully reconstructed, false otherwise.
   */
  bool backtracePath(NodeHybrid::NodePtr& node, common::geometry::Points3d* path);

  /**
   * @brief Retrieves the neighboring nodes of the given node.
   * @param node      The current node.
   * @param neighbors A vector to store the neighboring nodes.
   */
  void getNeighbors(const NodeHybrid::NodePtr& node,
                    std::vector<NodeHybrid::NodePtr>& neighbors);

  /**
   * @brief Checks if the given pose is in collision with any obstacles.
   * @param pose The pose to check in (x, y, theta).
   * @return true if the pose is in collision, false otherwise.
   */
  bool isCollision(const common::geometry::Point3d& node);

  /**
   * @brief Adds a node to the graph of visited nodes.
   * @param index The index of the node in the graph.
   * @return A pointer to the added node.
   */
  NodeHybrid::NodePtr addToGraph(const uint64_t& index);

  /**
   * @brief Clear graph of nodes searched
   */
  void clearGraph();

  /**
   * @brief Add a node to the open set
   * @param cost The cost to sort into the open set of the node
   * @param node Node pointer reference to add to open set
   */
  inline void addToQueue(const double& cost, NodeHybrid::NodePtr& node);

  /**
   * @brief Clear heuristic queue of nodes to search
   */
  inline void clearQueue();

  /**
   * @brief Checks if the given node has reached the goal.
   * @param node The current node.
   * @param goal The goal node.
   * @return true if the node is within the goal tolerance, false otherwise.
   */
  bool isReachGoal(const NodeHybrid::NodePtr& node,
                   const NodeHybrid::NodePtr& goal) const;

protected:
  pb::path_planner::HybriAstarPlanner hybrid_astar_config_;
  static HybridAStarMotionTable motion_table_;
  static std::vector<rmp::common::structure::Node<int>> grid_motions_;

  common::geometry::Point3d goal_;
  common::geometry::Points3d last_path_;

  Graph graph_;
  Queue queue_;
  std::pair<float, uint64_t> best_heuristic_node_;
  std::vector<std::vector<double>> obstacle_hmap_;
  std::vector<NodeHybrid::NodePtr> expansions_node_;
};
}  // namespace path_planner
}  // namespace rmp

#endif