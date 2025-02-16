/**
 * *********************************************************
 *
 * @file: node_wrapper.cpp
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
#include "path_planner/graph_planner/hybrid_astar_planner/node_hybrid.h"
#include "path_planner/graph_planner/hybrid_astar_planner/node_wrapper.h"

namespace rmp
{
namespace path_planner
{
template <typename Node2D>
void NodeWrapper<Node2D>::processSearchNode()
{
}

template <>
void NodeWrapper<NodeHybrid>::processSearchNode()
{
  // We only want to override the node's pose if it has not yet been visited
  // to prevent the case that a node has been queued multiple times and
  // a new branch is overriding one of lower cost already visited.
  if (!graph_node_ptr->is_visited())
  {
    graph_node_ptr->setPose(pose);
    graph_node_ptr->setMotionPrimitiveIndex(motion_index, turn_dir);
  }
}

template <>
void NodeWrapper<NodeHybrid>::populateSearchNode(NodeHybrid*& node)
{
  pose = node->pose();
  graph_node_ptr = node;
  motion_index = node->motion_primitive_index();
  turn_dir = node->turn_direction();
}

template class NodeWrapper<NodeHybrid>;
}  // namespace path_planner
}  // namespace rmp