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
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_HYBRID_ASTAR_PLANNER_NODE_HYBRID_H_
#define RMP_PATH_PLANNER_GRAPH_PLANNER_HYBRID_ASTAR_PLANNER_NODE_HYBRID_H_

#include <memory>
#include <functional>

#include "common/geometry/point.h"
#include "path_planner/utils/motions.h"
#include "path_planner/graph_planner/hybrid_astar_planner/motion_table.h"

namespace rmp
{
namespace path_planner
{
class NodeHybrid
{
public:
  typedef NodeHybrid* NodePtr;

  /**
   * @brief A constructor for NodeHybrid
   * @param index The index of this node for self-reference
   */
  explicit NodeHybrid(const uint64_t index);

  /**
   * @brief A destructor for NodeHybrid
   */
  ~NodeHybrid();

  void reset();

  /**
   * @brief operator== for comparisons
   * @param Node2D right hand side node reference
   * @return If cell indices are equal
   */
  bool operator==(const NodeHybrid& other) const;

  /**
   * @brief Gets the accumulated cost at this node
   * @return accumulated cost
   */
  inline double accumulated_cost() const
  {
    return accumulated_cost_;
  }

  /**
   * @brief Sets the accumulated cost at this node
   * @param cost_in reference to accumulated cost
   */
  inline void setAccumulatedCost(const double& cost_in)
  {
    accumulated_cost_ = cost_in;
  }

  /**
   * @brief Gets if cell has been visited in search
   * @return If cell was visited
   */
  inline bool is_visited() const
  {
    return is_visited_;
  }

  /**
   * @brief Sets if cell has been visited in search
   */
  inline void visited()
  {
    is_visited_ = true;
  }

  /**
   * @brief Gets cell index
   */
  inline uint64_t index() const
  {
    return index_;
  }

  /**
   * @brief Gets node pose
   */
  inline rmp::common::geometry::Point3d pose() const
  {
    return pose_;
  };

  /**
   * @brief Sets the pose of current node
   */
  inline void setPose(const rmp::common::geometry::Point3d& pose)
  {
    pose_.setX(pose.x());
    pose_.setY(pose.y());
    pose_.setTheta(pose.theta());
  }

  /**
   * @brief Sets the motion primitive index used to achieve node in search
   * @param motion_primitive_index reference to motion primitive idx
   * @param turn_dir turning direction
   */
  inline void setMotionPrimitiveIndex(const unsigned int& motion_primitive_index, const TurnDirection& turn_dir)
  {
    motion_primitive_index_ = motion_primitive_index;
    turn_dir_ = turn_dir;
  }

  /**
   * @brief Gets the motion primitive index used to achieve node in search
   * @return reference to motion primitive idx
   */
  inline unsigned int motion_primitive_index() const
  {
    return motion_primitive_index_;
  }

  /**
   * @brief Gets the motion primitive turning direction used to achieve node in search
   * @return reference to motion primitive turning direction
   */
  inline TurnDirection turn_direction() const
  {
    return turn_dir_;
  }

  /**
   * @brief Calculate transition cost considering motion continuity and penalties.
   * @param child Target child node
   * @param motion_table Contains cost parameters and penalties
   * @return Adjusted traversal cost between current node and child
   */
  double getTraversalCost(const NodePtr& child, const HybridAStarMotionTable& motion_table) const;

public:
  NodePtr parent;  // the parent of this node

private:
  double accumulated_cost_;              // the accumulated cost at this node
  bool is_visited_;                      // if cell has been visited(closed) in search
  uint64_t index_;                       // cell index
  unsigned int motion_primitive_index_;  // motion primitive index
  TurnDirection turn_dir_;               // turn direction mode
  rmp::common::geometry::Point3d pose_;  // the pose of current node
};
}  // namespace path_planner
}  // namespace rmp

#endif