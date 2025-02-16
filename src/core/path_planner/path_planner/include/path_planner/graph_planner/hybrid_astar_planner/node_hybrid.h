/**
 * *********************************************************
 *
 * @file: node_hybrid.h
 * @brief: Contains node for hybrid grid searching
 * @author: Yang Haodong
 * @date: 2025-01-18
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_PATH_PLANNER_UTIL_NODE_HYBRID_H_
#define RMP_PATH_PLANNER_UTIL_NODE_HYBRID_H_

#include "common/geometry/point.h"
#include "common/geometry/curve/curve.h"
#include "common/geometry/collision_checker.h"
#include "path_planner/utils/motions.h"

namespace rmp
{
namespace path_planner
{
typedef std::pair<float, uint64_t> ObstacleHeuristicElement;
typedef std::vector<ObstacleHeuristicElement> ObstacleHeuristicQueue;

struct ObstacleHeuristicComparator
{
  bool operator()(const ObstacleHeuristicElement& a, const ObstacleHeuristicElement& b) const
  {
    return a.first > b.first;
  }
};

/**
 * @brief Search properties and penalties
 */
struct HybridSearchInfo
{
  int motion_model{ 2 };                   // Motion model. 2 for DUBIN and 3 for REEDS_SHEPP
  double goal_tolerance{ 0.125 };          // tolerance distance to goal [m]
  int dim_3_size{ 1 };                     // Auxiliary dimensions for search
  int max_iterations{ 10000 };             // Maximum number of iterations to use while expanding search
  int max_approach_iterations{ 1000 };     // Maximum number of iterations during search near the goal
  bool traverse_unknown{ true };           // Allow search in unknown space, good for navigation while mapping
  double cost_penalty{ 2.0 };              // penalty to apply to higher cost areas
  bool cache_obstacle_heuristic{ false };  // whether allow pre-computed obstacle heuristics
  double curve_sample_ratio{ 0.15 };       // sample ratio for curve generation
  double minimum_turning_radius{ 0.4 };    // Minimum turning radius
  double non_straight_penalty{ 1.20 };     // penalty to apply if motion is non-straight, must be => 1
  double change_penalty{ 0.0 };            // penalty to apply if motion is changing directions, must be >= 0
  double reverse_penalty{ 2.1 };           // penalty to apply if motion is reversing, must be => 1
  double retrospective_penalty{ 0.025 };   // penalty to prefer later maneuvers before earlier along the path.
  double lookup_table_dim{ 20.0 };         // size of the dubin/reeds-sheep distance window to cache [meters].
  double analytic_expansion_ratio{ 3.5 };  // The ratio to attempt analytic expansions during search for final approach
  double analytic_expansion_max_length{ 3.0 };  // The maximum length of the analytic expansion to be considered valid
  bool downsample_obstacle_heuristic{ true };
};

class NodeHybrid;

/**
 * @brief A table of motion primitives and related functions
 */
struct HybridMotionTable
{
  /**
   * @brief A constructor for HybridMotionTable
   */
  HybridMotionTable() = default;

  /**
   * @brief Initializing using Dubin model
   * @param size_x_in Size of costmap in X
   * @param size_y_in Size of costmap in Y
   * @param search_info Parameters for searching
   */
  void initDubin(unsigned int& size_x_in, unsigned int& size_y_in, HybridSearchInfo& search_info);

  /**
   * @brief Initializing using Reeds-Shepp model
   * @param size_x_in Size of costmap in X
   * @param size_y_in Size of costmap in Y
   * @param search_info Parameters for searching
   */
  void initReedsShepp(unsigned int& size_x_in, unsigned int& size_y_in, HybridSearchInfo& search_info);

  /**
   * @brief Get projections of motion models
   * @param node Ptr to NodeHybrid
   * @return A set of motion poses
   */
  MotionPoses getProjections(const NodeHybrid* node);

  /**
   * @brief Get the angular bin to use from a raw orientation
   * @param theta Angle in radians
   * @return bin index of closest angle to request
   */
  unsigned int getClosestAngularBin(const double& theta);

  /**
   * @brief Get the raw orientation from an angular bin
   * @param bin_idx Index of the bin
   * @return Raw orientation in radians
   */
  double getAngleFromBin(const unsigned int& bin_idx);

  /**
   * @brief Get the angle scaled across bins from a raw orientation
   * @param theta Angle in radians
   * @return angle scaled across bins
   */
  double getOrientationBin(const double& theta);

  MotionPoses projections;
  MotionModel motion_model = MotionModel::UNKNOWN;
  unsigned int num_angle_quantization;
  double min_turning_radius;
  double bin_size;

  // search properties
  unsigned int map_width;
  double change_penalty;
  double non_straight_penalty;
  double cost_penalty;
  double reverse_penalty;
  double travel_distance_reward;
  bool downsample_obstacle_heuristic;

  // motion primitives
  std::shared_ptr<rmp::common::geometry::Curve> curve_gen;
  std::vector<std::vector<double>> delta_xs;
  std::vector<std::vector<double>> delta_ys;
  std::vector<std::pair<double, double>> trig_values;
  std::vector<float> travel_costs;
};

// node for Hybrid path searching
class NodeHybrid
{
public:
  typedef NodeHybrid* NodePtr;
  typedef std::vector<NodePtr> NodeVector;
  typedef rmp::common::geometry::Point3d Pose;
  typedef rmp::common::geometry::Points3d Poses;

  /**
   * @brief A constructor for NodeHybrid
   * @param index The index of this node for self-reference
   */
  explicit NodeHybrid(const uint64_t index);

  /**
   * @brief A destructor for NodeHybrid
   */
  ~NodeHybrid();

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
   * @brief Gets the costmap cost at this node
   * @return costmap cost
   */
  inline double cell_cost() const
  {
    return cell_cost_;
  }

  /**
   * @brief Sets the costmap cost at this node
   * @param cost_in reference to cell cost
   */
  inline void setCellCost(const double& cost_in)
  {
    cell_cost_ = cost_in;
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
  inline unsigned int& motion_primitive_index()
  {
    return motion_primitive_index_;
  }

  /**
   * @brief Gets the motion primitive turning direction used to achieve node in search
   * @return reference to motion primitive turning direction
   */
  inline TurnDirection& turn_direction()
  {
    return turn_dir_;
  }

  /**
   * @brief Gets cell index
   */
  inline uint64_t index() const
  {
    return index_;
  }

  /**
   * @brief Get index in <x, y, theta>
   * @param pose given pose to get index of
   * @return index
   */
  static inline uint64_t getIndex(const Pose& pose)
  {
    return static_cast<uint64_t>(pose.theta()) +
           static_cast<uint64_t>(pose.x()) * static_cast<uint64_t>(motion_table.num_angle_quantization) +
           static_cast<uint64_t>(pose.y()) * static_cast<uint64_t>(motion_table.map_width) *
               static_cast<uint64_t>(motion_table.num_angle_quantization);
  }

  /**
   * @brief Gets node pose
   */
  inline Pose pose() const
  {
    return pose_;
  };

  /**
   * @brief Sets the pose of current node
   */
  inline void setPose(const Pose& pose)
  {
    pose_.setX(pose.x());
    pose_.setY(pose.y());
    pose_.setTheta(pose.theta());
  }

  /**
   * @brief operator== for comparisons
   * @param NodeHybrid right hand side node reference
   * @return If cell indices are equal
   */
  bool operator==(const NodeHybrid& rhs);

  /**
   * @brief Reset method for new search
   */
  void reset();

  /**
   * @brief Initialize the neighborhood to be used in A*
   * @param motion_model The allowed motions for the node
   * @param x_size The total x size to find neighbors
   * @param y_size The total y size to find neighbors
   * @param search_info Search parameters, unused by 2D node
   */
  static void initMotionModel(const MotionModel& motion_model, unsigned int& x_size, unsigned int& y_size,
                              HybridSearchInfo& search_info);

  /**
   * @brief Check if this node is valid
   * @param traverse_unknown If we can explore unknown nodes on the graph
   * @param collision_checker Pointer to collision checker object
   * @return whether this node is valid and collision free
   */
  bool isValid(const bool& traverse_unknown,
               const std::shared_ptr<rmp::common::geometry::CollisionChecker>& collision_checker);

  /**
   * @brief Get index
   * @param Index Index of point
   * @return coordinates of point
   */
  static inline Pose getCoords(const uint64_t& index)
  {
    return Pose((index / motion_table.num_angle_quantization) % motion_table.map_width,  // x
                index / (motion_table.num_angle_quantization * motion_table.map_width),  // y
                index % motion_table.num_angle_quantization);                            // theta
  }

  /**
   * @brief get traversal cost from this node to child node
   * @param child Node pointer to this node's child
   * @return traversal cost
   */
  double getTraversalCost(const NodePtr& child);

  /**
   * @brief Get cost of heuristic of node
   * @param node Node index current
   * @param node Node index of new
   * @return Heuristic cost between the nodes
   */
  static double getHeuristicCost(const Pose& node_coords, const Pose& goal_coords);

  /**
   * @brief Compute the Obstacle heuristic
   * @param node_coords Coordinates to get heuristic at
   * @param goal_coords Coordinates to compute heuristic to
   * @return heuristic Heuristic value
   */
  static double getObstacleHeuristic(const Pose& node_coords, const Pose& goal_coords, const double& cost_penalty);

  /**
   * @brief Compute the Distance heuristic
   * @param node_coords Coordinates to get heuristic at
   * @param goal_coords Coordinates to compute heuristic to
   * @param obstacle_heuristic Value of the obstacle heuristic to compute
   * additional motion heuristics if required
   * @return heuristic Heuristic value
   */
  static double getDistanceHeuristic(const Pose& node_coords, const Pose& goal_coords,
                                     const double& obstacle_heuristic);

  /**
   * @brief reset the obstacle heuristic state
   * @param costmap_ros Costmap to use
   * @param start_coords Coordinates to start heuristic expansion at
   * @param goal_coords Coordinates to goal heuristic expansion at
   */
  static void resetObstacleHeuristic(costmap_2d::Costmap2DROS* costmap_ros, const unsigned int& start_x,
                                     const unsigned int& start_y, const unsigned int& goal_x,
                                     const unsigned int& goal_y);

  /**
   * @brief Compute the SE2 distance heuristic
   * @param motion_model Motion model to use for state space
   * @param search_info Info containing minimum radius to use
   */
  static void precomputeDistanceHeuristic(const MotionModel& motion_model, const HybridSearchInfo& search_info);

  /**
   * @brief Retrieve all valid neighbors of a node.
   * @param NeighborGetter Functor for getting neighbors
   * @param collision_checker Collision checker to use
   * @param traverse_unknown If unknown costs are valid to traverse
   * @param neighbors Vector of neighbors to be filled
   */
  void getNeighbors(std::function<bool(const uint64_t&, NodePtr&)>& NeighborGetter,
                    const std::shared_ptr<rmp::common::geometry::CollisionChecker>& collision_checker,
                    bool traverse_unknown, NodeVector& neighbors);

  /**
   * @brief Set the starting pose for planning, as a node index
   * @param path Reference to a vector of indices of generated path
   * @return whether the path was able to be backtraced
   */
  bool backtracePath(Poses& path);

public:
  NodeHybrid* parent;  // the parent of this node
  static double travel_distance_cost;
  static HybridMotionTable motion_table;
  static costmap_2d::Costmap2DROS* costmap_ros;
  // Wavefront lookup for continuing to expand as needed
  static std::vector<double> obstacle_heuristic_lookup_table;
  static ObstacleHeuristicQueue obstacle_heuristic_queue;
  // Dubin / Reeds-Shepp lookup and size for dereferencing
  static std::vector<double> dist_heuristic_lookup_table;
  static double size_lookup;

private:
  double cell_cost_;                     // the costmap cost at this node
  double accumulated_cost_;              // the accumulated cost at this node
  bool is_visited_;                      // if cell has been visited(closed) in search
  uint64_t index_;                       // cell index
  unsigned int motion_primitive_index_;  // motion primitive index
  TurnDirection turn_dir_;               // turn direction mode
  bool is_valid_;                        // if cell is valid
  Pose pose_;                            // the pose of current node
};
}  // namespace path_planner
}  // namespace rmp

#endif