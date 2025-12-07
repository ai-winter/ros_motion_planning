/**
 * *********************************************************
 *
 * @file: node_hybrid.h
 * @brief: hybrid node for Hybrid A* planning
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
#include "path_planner/graph_planner/hybrid_astar_planner/node_hybrid.h"

using namespace rmp::common::geometry;

namespace rmp
{
namespace path_planner
{
/**
 * @brief A constructor for NodeHybrid
 * @param index The index of this node for self-reference
 */
NodeHybrid::NodeHybrid(const uint64_t index)
  : parent(nullptr)
  , accumulated_cost_(std::numeric_limits<float>::max())
  , index_(index)
  , is_visited_(false)
  , motion_primitive_index_(std::numeric_limits<unsigned int>::max())
  , turn_dir_(TurnDirection::FORWARD)
  , pose_(Point3d())
{
}

/**
 * @brief A destructor for NodeHybrid
 */
NodeHybrid::~NodeHybrid()
{
  parent = nullptr;
}

void NodeHybrid::reset()
{
  parent = nullptr;
  accumulated_cost_ = std::numeric_limits<float>::max();
  is_visited_ = false;
  motion_primitive_index_ = std::numeric_limits<unsigned int>::max();
  pose_.setX(0.0);
  pose_.setY(0.0);
  pose_.setTheta(0.0);
}

/**
 * @brief operator== for comparisons
 * @param Node2D right hand side node reference
 * @return If cell indices are equal
 */
bool NodeHybrid::operator==(const NodeHybrid& other) const
{
  return index_ == other.index_;
}

/**
 * @brief Calculate transition cost considering motion continuity and penalties.
 * @param child Target child node
 * @param motion_table Contains cost parameters and penalties
 * @return Adjusted traversal cost between current node and child
 */
double NodeHybrid::getTraversalCost(const NodePtr& child, const HybridAStarMotionTable& motion_table) const
{
  // this is the first node
  if (motion_primitive_index_ == std::numeric_limits<unsigned int>::max())
  {
    return motion_table.projections[0].x();
  }

  double travel_cost = 0.0;
  const double travel_cost_raw =
      motion_table.travel_costs[child->motion_primitive_index()] * motion_table.travel_distance_reward;

  if (child->turn_direction() == TurnDirection::FORWARD || child->turn_direction() == TurnDirection::REVERSE)
  {
    travel_cost = travel_cost_raw;
  }
  else
  {
    if (turn_dir_ == child->turn_direction())
    {
      travel_cost = travel_cost_raw * motion_table.non_straight_penalty;
    }
    else
    {
      travel_cost = travel_cost_raw * (motion_table.non_straight_penalty + motion_table.change_penalty);
    }
  }

  if (child->turn_direction() == TurnDirection::REV_LEFT || child->turn_direction() == TurnDirection::REV_RIGHT ||
      child->turn_direction() == TurnDirection::REVERSE)
  {
    travel_cost *= motion_table.reverse_penalty;
  }

  return travel_cost;
}

}  // namespace path_planner
}  // namespace rmp