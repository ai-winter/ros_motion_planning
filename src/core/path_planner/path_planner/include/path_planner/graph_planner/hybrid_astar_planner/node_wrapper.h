/**
 * *********************************************************
 *
 * @file: node_wrapper.h
 * @brief: Contains node wrapper for grid searching
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
#ifndef RMP_PATH_PLANNER_UTIL_NODE_WRAPPER_H_
#define RMP_PATH_PLANNER_UTIL_NODE_WRAPPER_H_

namespace rmp
{
namespace path_planner
{
/**
 * @brief NodeWrapper implementation for priority queue insertion
 */
template <typename NodeT>
class NodeWrapper
{
public:
  /**
   * @brief A constructor for NodeWrapper
   * @param index The index of this node for self-reference
   */
  explicit NodeWrapper(const uint64_t new_index) : graph_node_ptr(nullptr), index(new_index)
  {
  }

  /**
   * @brief Take a NodeWrapper and populate it with any necessary state cached in the queue for NodeT.
   * @param node NodeT ptr to populate metadata into NodeWrapper
   */
  void populateSearchNode(NodeT*& node);

  /**
   * @brief Take a NodeWrapper and populate it with any necessary state cached in the queue for NodeTs.
   * @param node Search node object to initialize internal node with state
   */
  void processSearchNode();

  typename NodeT::Pose pose;  // Used by NodeHybrid and NodeLattice
  NodeT* graph_node_ptr;
  // MotionPrimitive* prim_ptr;  // Used by NodeLattice
  uint64_t index;
  unsigned int motion_index;
  bool backward;
  TurnDirection turn_dir;
};
}  // namespace path_planner
}  // namespace rmp

#endif