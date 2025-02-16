/**
 * *********************************************************
 *
 * @file: node_hybrid.cpp
 * @brief: Contains node for hybrid grid searching
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
#include "common/geometry/curve/dubins_curve.h"
#include "common/geometry/curve/reeds_shepp_curve.h"
#include "path_planner/graph_planner/hybrid_astar_planner/node_hybrid.h"

using namespace rmp::common::geometry;

namespace rmp
{
namespace path_planner
{
// defining static member for all instance to share
HybridMotionTable NodeHybrid::motion_table;
costmap_2d::Costmap2DROS* NodeHybrid::costmap_ros = nullptr;
std::vector<double> NodeHybrid::obstacle_heuristic_lookup_table;
ObstacleHeuristicQueue NodeHybrid::obstacle_heuristic_queue;
double NodeHybrid::size_lookup = 25;
double NodeHybrid::travel_distance_cost;
std::vector<double> NodeHybrid::dist_heuristic_lookup_table;

/**
 * @brief Initializing using Dubin model
 * @param size_x_in Size of costmap in X
 * @param size_y_in Size of costmap in Y
 * @param angle_quantization_in Size of costmap in bin sizes
 * @param search_info Parameters for searching
 */
void HybridMotionTable::initDubin(unsigned int& size_x_in, unsigned int& /*size_y_in*/, HybridSearchInfo& search_info)
{
  // if nothing changed, no need to re-compute primitives
  if (num_angle_quantization == search_info.dim_3_size && min_turning_radius == search_info.minimum_turning_radius &&
      motion_model == MotionModel::DUBIN)
  {
    return;
  }

  map_width = size_x_in;
  change_penalty = search_info.change_penalty;
  non_straight_penalty = search_info.non_straight_penalty;
  cost_penalty = search_info.cost_penalty;
  reverse_penalty = search_info.reverse_penalty;
  travel_distance_reward = 1.0f - search_info.retrospective_penalty;
  downsample_obstacle_heuristic = search_info.downsample_obstacle_heuristic;

  // update
  num_angle_quantization = search_info.dim_3_size;
  min_turning_radius = search_info.minimum_turning_radius;
  motion_model = MotionModel::DUBIN;
  // chord length must be greater than sqrt(2) to leave current cell
  // Thusly: sqrt(2) >= 2 * R * sin (angle / 2) namely angle <= 2.0 * asin(sqrt(2) / (2 * R))
  double angle = 2.0 * std::asin(std::sqrt(2.0) / (2 * min_turning_radius));
  bin_size = 2.0 * M_PI / static_cast<double>(num_angle_quantization);
  double increments = angle < bin_size ? 1.0 : std::ceil(angle / bin_size);
  angle = increments * bin_size;  // unit increment angle (rad)

  const double d_x = min_turning_radius * std::sin(angle);
  const double d_y = min_turning_radius - (min_turning_radius * std::cos(angle));
  const double d_dist = std::hypot(d_x, d_y);

  // Create dubins generator
  curve_gen = std::make_shared<DubinsCurve>(search_info.curve_sample_ratio, 1.0 / min_turning_radius);

  // Precompute motion primitives
  projections.clear();
  projections.emplace_back(d_dist, 0.0, 0.0, TurnDirection::FORWARD);      // Forward
  projections.emplace_back(d_x, d_y, increments, TurnDirection::LEFT);     // Left
  projections.emplace_back(d_x, -d_y, -increments, TurnDirection::RIGHT);  // Right
  delta_xs.resize(projections.size());
  delta_ys.resize(projections.size());
  trig_values.resize(num_angle_quantization);

  for (unsigned int i = 0; i != projections.size(); i++)
  {
    delta_xs[i].resize(num_angle_quantization);
    delta_ys[i].resize(num_angle_quantization);

    for (unsigned int j = 0; j != num_angle_quantization; j++)
    {
      double cos_theta = cos(bin_size * j);
      double sin_theta = sin(bin_size * j);
      if (i == 0)
      {
        // if first iteration, cache the trig values for later
        trig_values[j] = { cos_theta, sin_theta };
      }
      delta_xs[i][j] = projections[i].x_ * cos_theta - projections[i].y_ * sin_theta;
      delta_ys[i][j] = projections[i].x_ * sin_theta + projections[i].y_ * cos_theta;
    }
  }

  // Precompute travel costs for each motion primitive
  travel_costs.resize(projections.size());
  for (unsigned int i = 0; i != projections.size(); i++)
  {
    const TurnDirection turn_dir = projections[i].turn_dir_;
    if (turn_dir != TurnDirection::FORWARD && turn_dir != TurnDirection::REVERSE)
    {
      const double arc_angle = projections[i].theta_ * bin_size;  // +/- unit increment angle
      const double turning_rad = 0.5 * d_dist / (std::sin(arc_angle * 0.5));
      travel_costs[i] = turning_rad * arc_angle;
    }
    else
    {
      travel_costs[i] = d_dist;
    }
  }
}

/**
 * @brief Get projections of motion models
 * @param node Ptr to NodeHybrid
 * @return A set of motion poses
 */
MotionPoses HybridMotionTable::getProjections(const NodeHybrid* node)
{
  MotionPoses projection_list;
  projection_list.reserve(projections.size());
  for (unsigned int i = 0; i != projections.size(); i++)
  {
    const MotionPose& proj_motion_model = projections[i];
    const double& node_heading = node->pose().theta();
    double new_heading = node_heading + proj_motion_model.theta_;

    if (new_heading < 0.0)
    {
      new_heading += static_cast<double>(num_angle_quantization);
    }

    if (new_heading >= static_cast<double>(num_angle_quantization))
    {
      new_heading -= static_cast<double>(num_angle_quantization);
    }

    projection_list.emplace_back(delta_xs[i][node_heading] + node->pose().x(),
                                 delta_ys[i][node_heading] + node->pose().y(), new_heading,
                                 proj_motion_model.turn_dir_);
  }

  return projection_list;
}

double HybridMotionTable::getAngleFromBin(const unsigned int& bin_idx)
{
  return bin_idx * bin_size;
}

double HybridMotionTable::getOrientationBin(const double& theta)
{
  double num_angle_quantizations = 2.0 * M_PI / bin_size;
  double orientation_bin = std::round(theta / bin_size);
  while (orientation_bin < 0.0)
  {
    orientation_bin += num_angle_quantizations;
  }
  if (orientation_bin >= num_angle_quantizations)
  {
    orientation_bin -= num_angle_quantizations;
  }
  return orientation_bin;
};

/**
 * @brief A constructor for NodeHybrid
 * @param index The index of this node for self-reference
 */
NodeHybrid::NodeHybrid(const uint64_t index)
  : parent(nullptr)
  , cell_cost_(std::numeric_limits<float>::quiet_NaN())
  , accumulated_cost_(std::numeric_limits<float>::max())
  , index_(index)
  , is_visited_(false)
  , motion_primitive_index_(std::numeric_limits<unsigned int>::max())
  , is_valid_(false)
  , pose_(getCoords(index))
{
}

/**
 * @brief A destructor for NodeHybrid
 */
NodeHybrid::~NodeHybrid()
{
  parent = nullptr;
}

/**
 * @brief operator== for comparisons
 * @param Node2D right hand side node reference
 * @return If cell indices are equal
 */
bool NodeHybrid::operator==(const NodeHybrid& rhs)
{
  return index_ == rhs.index_;
}

/**
 * @brief Reset method for new search
 */
void NodeHybrid::reset()
{
  parent = nullptr;
  cell_cost_ = std::numeric_limits<float>::quiet_NaN();
  accumulated_cost_ = std::numeric_limits<float>::max();
  is_visited_ = false;
  motion_primitive_index_ = std::numeric_limits<unsigned int>::max();
  is_valid_ = false;
  pose_.setX(0.0);
  pose_.setY(0.0);
  pose_.setTheta(0.0);
}

/**
 * @brief Initialize the neighborhood to be used in Hybrid A*
 * @param motion_model The allowed motions for the node
 * @param x_size The total x size to find neighbors
 * @param y_size The total y size to find neighbors
 * @param search_info Search parameters, unused by 2D node
 */
void NodeHybrid::initMotionModel(const MotionModel& motion_model, unsigned int& x_size, unsigned int& y_size,
                                 HybridSearchInfo& search_info)
{
  // find the motion model selected
  switch (motion_model)
  {
    case MotionModel::DUBIN:
      motion_table.initDubin(x_size, y_size, search_info);
      break;
    case MotionModel::REEDS_SHEPP:
      motion_table.initReedsShepp(x_size, y_size, search_info);
      break;
    default:
      R_ERROR << "Invalid motion model for Hybrid A*. Please select between\n"
              << " Dubin (Ackermann forward only),\n"
              << " Reeds-Shepp (Ackermann forward and back).";
  }

  travel_distance_cost = motion_table.projections[0].x_;
}

/**
 * @brief Check if this node is valid
 * @param traverse_unknown If we can explore unknown nodes on the graph
 * @param collision_checker Pointer to collision checker object
 * @return whether this node is valid and collision free
 */
bool NodeHybrid::isValid(const bool& traverse_unknown, const std::shared_ptr<CollisionChecker>& collision_checker)
{
  // Already found, we can return the result
  if (!std::isnan(cell_cost_))
  {
    return is_valid_;
  }

  // TODO: Consider footprint
  uint64_t idx = static_cast<uint64_t>(pose_.x()) +
                 static_cast<uint64_t>(pose_.y()) * static_cast<uint64_t>(motion_table.map_width);
  is_valid_ = !collision_checker->inCollision(idx, traverse_unknown);
  cell_cost_ = collision_checker->getCost(idx);
  return is_valid_;
}

/**
 * @brief get traversal cost from this node to child node
 * @param child Node pointer to this node's child
 * @return traversal cost
 */
double NodeHybrid::getTraversalCost(const NodePtr& child)
{
  const double normalized_cost = child->cell_cost() / 252.0;

  // this is the first node
  if (motion_primitive_index_ == std::numeric_limits<unsigned int>::max())
  {
    return NodeHybrid::travel_distance_cost;
  }

  const TurnDirection& child_turn_dir = child->turn_direction();
  double travel_cost_raw = motion_table.travel_costs[child->motion_primitive_index()];
  double travel_cost = 0.0;
  travel_cost_raw *= (motion_table.travel_distance_reward + motion_table.cost_penalty * normalized_cost);

  if (child_turn_dir == TurnDirection::FORWARD || child_turn_dir == TurnDirection::REVERSE)
  {
    travel_cost = travel_cost_raw;
  }
  else
  {
    if (turn_dir_ == child_turn_dir)
    {
      // Turning motion but keeps in same direction: encourages to commit to turning if starting it
      travel_cost = travel_cost_raw * motion_table.non_straight_penalty;
    }
    else
    {
      // Turning motion and changing direction: penalizes wiggling
      travel_cost = travel_cost_raw * (motion_table.non_straight_penalty + motion_table.change_penalty);
    }
  }

  if (child_turn_dir == TurnDirection::REV_RIGHT || child_turn_dir == TurnDirection::REV_LEFT ||
      child_turn_dir == TurnDirection::REVERSE)
  {
    travel_cost *= motion_table.reverse_penalty;
  }

  return travel_cost;
}

inline double distanceHeuristic2D(const uint64_t idx, const unsigned int size_x, const unsigned int target_x,
                                  const unsigned int target_y)
{
  int dx = static_cast<int>(idx % size_x) - static_cast<int>(target_x);
  int dy = static_cast<int>(idx / size_x) - static_cast<int>(target_y);
  return std::hypot(dx, dy);
}

/**
 * @brief Get cost of heuristic of node
 * @param node Node index current
 * @param node Node index of new
 * @return Heuristic cost between the nodes
 */
double NodeHybrid::getHeuristicCost(const Pose& node_coords, const Pose& goal_coords)
{
  const double obstacle_heuristic = getObstacleHeuristic(node_coords, goal_coords, motion_table.cost_penalty);
  const double dist_heuristic = getDistanceHeuristic(node_coords, goal_coords, obstacle_heuristic);
  return std::max(obstacle_heuristic, dist_heuristic);
}

/**
 * @brief Compute the Obstacle heuristic
 * @param node_coords Coordinates to get heuristic at
 * @param goal_coords Coordinates to compute heuristic to
 * @return heuristic Heuristic value
 */
double NodeHybrid::getObstacleHeuristic(const Pose& node_coords, const Pose& goal_coords, const double& cost_penalty)
{
  auto costmap = costmap_ros->getCostmap();
  unsigned int size_x = motion_table.downsample_obstacle_heuristic ?
                            std::ceil(static_cast<double>(costmap->getSizeInCellsX()) * 0.5) :
                            costmap->getSizeInCellsX();
  unsigned int size_y = motion_table.downsample_obstacle_heuristic ?
                            std::ceil(static_cast<double>(costmap->getSizeInCellsY()) * 0.5) :
                            costmap->getSizeInCellsY();
  unsigned int start_x =
      motion_table.downsample_obstacle_heuristic ? std::floor(node_coords.x() * 0.5) : std::floor(node_coords.x());
  unsigned int start_y =
      motion_table.downsample_obstacle_heuristic ? std::floor(node_coords.y() * 0.5) : std::floor(node_coords.y());
  const unsigned int start_index = start_y * size_x + start_x;
  const double& requested_node_cost = obstacle_heuristic_lookup_table[start_index];

  // If already expanded, return the cost
  if (requested_node_cost > 0.0f)
  {
    return motion_table.downsample_obstacle_heuristic ? 2.0 * requested_node_cost : requested_node_cost;
  }

  // If not, expand until it is included
  // start_x and start_y have changed since last call, we need to recompute 2D distance heuristic and reprioritize queue
  for (auto& n : obstacle_heuristic_queue)
  {
    n.first = -obstacle_heuristic_lookup_table[n.second] + distanceHeuristic2D(n.second, size_x, start_x, start_y);
  }
  std::make_heap(obstacle_heuristic_queue.begin(), obstacle_heuristic_queue.end(), ObstacleHeuristicComparator{});

  const float sqrt2 = sqrtf(2.0f);
  double c_cost, cost, travel_cost, new_cost, existing_cost;

  const std::vector<int> neighborhood = { 1,
                                          -1,
                                          static_cast<int>(size_x),
                                          -static_cast<int>(size_x),
                                          static_cast<int>(size_x) + 1,
                                          static_cast<int>(size_x) - 1,
                                          -static_cast<int>(size_x) + 1,
                                          -static_cast<int>(size_x) - 1 };

  while (!obstacle_heuristic_queue.empty())
  {
    unsigned int idx = obstacle_heuristic_queue.front().second;
    std::pop_heap(obstacle_heuristic_queue.begin(), obstacle_heuristic_queue.end(), ObstacleHeuristicComparator{});
    obstacle_heuristic_queue.pop_back();
    c_cost = obstacle_heuristic_lookup_table[idx];
    if (c_cost > 0.0f)
    {
      continue;
    }
    c_cost = -c_cost;
    obstacle_heuristic_lookup_table[idx] = c_cost;  // set a positive value to close the cell

    // find neighbors
    for (unsigned int i = 0; i != neighborhood.size(); i++)
    {
      unsigned int new_idx = static_cast<unsigned int>(static_cast<int>(idx) + neighborhood[i]);

      // if neighbor path is better and non-lethal, set new cost and add to queue
      if (new_idx < size_x * size_y)
      {
        if (motion_table.downsample_obstacle_heuristic)
        {
          // Get costmap values as if downsampled
          unsigned int y_offset = (new_idx / size_x) * 2;
          unsigned int x_offset = (new_idx - ((new_idx / size_x) * size_x)) * 2;
          cost = costmap->getCost(x_offset, y_offset);
          for (unsigned int k = 0; k < 2u; ++k)
          {
            unsigned int mxd = x_offset + k;
            if (mxd >= costmap->getSizeInCellsX())
            {
              continue;
            }
            for (unsigned int j = 0; j < 2u; ++j)
            {
              unsigned int myd = y_offset + j;
              if (myd >= costmap->getSizeInCellsY())
              {
                continue;
              }
              if (k == 0 && j == 0)
              {
                continue;
              }
              cost = std::min(cost, static_cast<double>(costmap->getCost(mxd, myd)));
            }
          }
        }
        else
        {
          cost = static_cast<double>(costmap->getCharMap()[new_idx]);
        }

        if (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
          continue;
        }

        unsigned int my = new_idx / size_x;
        unsigned int mx = new_idx - (my * size_x);

        if (mx >= size_x - 3 || mx <= 3)
        {
          continue;
        }
        if (my >= size_y - 3 || my <= 3)
        {
          continue;
        }

        existing_cost = obstacle_heuristic_lookup_table[new_idx];
        if (existing_cost <= 0.0f)
        {
          travel_cost = ((i <= 3) ? 1.0f : sqrt2) * (1.0f + (cost_penalty * cost / 252.0f));

          new_cost = c_cost + travel_cost;
          if (existing_cost == 0.0f || -existing_cost > new_cost)
          {
            // the negative value means the cell is in the open set
            obstacle_heuristic_lookup_table[new_idx] = -new_cost;
            obstacle_heuristic_queue.emplace_back(new_cost + distanceHeuristic2D(new_idx, size_x, start_x, start_y),
                                                  new_idx);
            std::push_heap(obstacle_heuristic_queue.begin(), obstacle_heuristic_queue.end(),
                           ObstacleHeuristicComparator{});
          }
        }
      }
    }

    if (idx == start_index)
    {
      break;
    }
  }

  return motion_table.downsample_obstacle_heuristic ? 2.0 * requested_node_cost : requested_node_cost;
}

/**
 * @brief Compute the Distance heuristic
 * @param node_coords Coordinates to get heuristic at
 * @param goal_coords Coordinates to compute heuristic to
 * @param obstacle_heuristic Value of the obstacle heuristic to compute
 * additional motion heuristics if required
 * @return heuristic Heuristic value
 */
double NodeHybrid::getDistanceHeuristic(const Pose& node_coords, const Pose& goal_coords,
                                        const double& obstacle_heuristic)
{
  // rotate and translate node_coords such that goal_coords relative is (0,0,0)
  // This angle is negative since we are de-rotating the current node
  // by the goal angle; cos(-th) = cos(th) & sin(-th) = -sin(th)
  const auto& trig_vals = motion_table.trig_values[goal_coords.theta()];
  const double cos_th = trig_vals.first;
  const double sin_th = -trig_vals.second;
  const double dx = node_coords.x() - goal_coords.x();
  const double dy = node_coords.y() - goal_coords.y();

  double dtheta_bin = node_coords.theta() - goal_coords.theta();
  if (dtheta_bin < 0)
  {
    dtheta_bin += motion_table.num_angle_quantization;
  }
  if (dtheta_bin > motion_table.num_angle_quantization)
  {
    dtheta_bin -= motion_table.num_angle_quantization;
  }

  Pose node_relative(std::round(dx * cos_th - dy * sin_th), std::round(dx * sin_th + dy * cos_th),
                     std::round(dtheta_bin));

  // Check if the relative node coordinate is within the localized window around the goal
  // to apply the distance heuristic. Since the lookup table is contains only the positive
  // X axis, we mirror the Y and theta values across the X axis to find the heuristic values.
  double motion_heuristic = 0.0;
  const int floored_size = std::floor(size_lookup / 2.0);
  const int ceiling_size = std::ceil(size_lookup / 2.0);
  const double mirrored_relative_y = std::fabs(node_relative.y());
  if (std::fabs(node_relative.x()) < floored_size && mirrored_relative_y < floored_size)
  {
    // Need to mirror angle if Y coordinate was mirrored
    int theta_pos =
        node_relative.y() < 0.0 ? motion_table.num_angle_quantization - node_relative.theta() : node_relative.theta();
    const int x_pos = node_relative.x() + floored_size;
    const int y_pos = static_cast<int>(mirrored_relative_y);
    const int index = x_pos * ceiling_size * motion_table.num_angle_quantization +
                      y_pos * motion_table.num_angle_quantization + theta_pos;
    motion_heuristic = dist_heuristic_lookup_table[index];
  }
  else if (obstacle_heuristic <= 0.0)
  {
    // If no obstacle heuristic value, must have some H to use
    Points3d motion_path;
    Point3d from(node_coords.x(), node_coords.y(), node_coords.theta() * motion_table.num_angle_quantization);
    Point3d to(goal_coords.x(), goal_coords.y(), goal_coords.theta() * motion_table.num_angle_quantization);
    if (!motion_table.curve_gen->generation(from, to, motion_path))
    {
      R_ERROR << "No obstacle heuristic value and generate motion path failed";
    }
    motion_heuristic = motion_table.curve_gen->distance(motion_path);
  }

  return motion_heuristic;
}

/**
 * @brief reset the obstacle heuristic state
 * @param costmap_ros Costmap to use
 * @param start_coords Coordinates to start heuristic expansion at
 * @param goal_coords Coordinates to goal heuristic expansion at
 */
void NodeHybrid::resetObstacleHeuristic(costmap_2d::Costmap2DROS* costmap_ros_i, const unsigned int& start_x,
                                        const unsigned int& start_y, const unsigned int& goal_x,
                                        const unsigned int& goal_y)
{
  // Downsample costmap 2x to compute a sparse obstacle heuristic for speed up reason.
  costmap_ros = costmap_ros_i;
  auto costmap = costmap_ros->getCostmap();

  // Clear lookup table
  unsigned int size = 0u;
  unsigned int size_x = 0u;
  if (motion_table.downsample_obstacle_heuristic)
  {
    size_x = std::ceil(static_cast<double>(costmap->getSizeInCellsX()) * 0.5);
    size = size_x * std::ceil(static_cast<double>(costmap->getSizeInCellsY()) * 0.5);
  }
  else
  {
    size_x = costmap->getSizeInCellsX();
    size = size_x * costmap->getSizeInCellsY();
  }

  if (obstacle_heuristic_lookup_table.size() == size)
  {
    // must reset all values
    std::fill(obstacle_heuristic_lookup_table.begin(), obstacle_heuristic_lookup_table.end(), 0.0);
  }
  else
  {
    unsigned int obstacle_size = obstacle_heuristic_lookup_table.size();
    obstacle_heuristic_lookup_table.resize(size, 0.0);
    // must reset values for non-constructed indices
    std::fill_n(obstacle_heuristic_lookup_table.begin(), obstacle_size, 0.0);
  }

  obstacle_heuristic_queue.clear();
  obstacle_heuristic_queue.reserve(size);

  // Set initial goal point to queue from. Divided by 2 due to downsampled costmap.
  unsigned int goal_index;
  if (motion_table.downsample_obstacle_heuristic)
  {
    goal_index = floor(goal_y / 2.0f) * size_x + floor(goal_x / 2.0f);
  }
  else
  {
    goal_index = floor(goal_y) * size_x + floor(goal_x);
  }

  obstacle_heuristic_queue.emplace_back(distanceHeuristic2D(goal_index, size_x, start_x, start_y), goal_index);

  // initialize goal cell with a very small value to differentiate it from 0.0 (~uninitialized)
  // the negative value means the cell is in the open set
  obstacle_heuristic_lookup_table[goal_index] = -0.00001f;
}

void NodeHybrid::precomputeDistanceHeuristic(const MotionModel& motion_model, const HybridSearchInfo& search_info)
{
  // Dubin or Reeds-Shepp shortest distances
  if (motion_model == MotionModel::DUBIN)
  {
    motion_table.curve_gen =
        std::make_shared<DubinsCurve>(search_info.curve_sample_ratio, 1.0 / search_info.minimum_turning_radius);
  }
  else if (motion_model == MotionModel::REEDS_SHEPP)
  {
    motion_table.curve_gen =
        std::make_shared<ReedsSheppCurve>(search_info.curve_sample_ratio, 1.0 / search_info.minimum_turning_radius);
  }
  else
  {
    R_ERROR << "Node attempted to precompute distance heuristics with invalid motion model!";
  }

  Point3d to(0.0, 0.0, 0.0);
  size_lookup = search_info.lookup_table_dim;
  unsigned int index = 0;
  double angular_bin_size = 2 * M_PI / static_cast<double>(search_info.dim_3_size);

  // Create a lookup table of Dubin/Reeds-Shepp distances in a window around the goal
  // to help drive the search towards admissible approaches. Deu to symmetries in the
  // Heuristic space, we need to only store 2 of the 4 quadrants and simply mirror
  // around the X axis any relative node lookup. This reduces memory overhead and increases
  // the size of a window a platform can store in memory.
  dist_heuristic_lookup_table.resize((size_lookup + 1) * (floor(size_lookup / 2.0) + 1) * search_info.dim_3_size);
  for (float x = ceil(-size_lookup / 2.0); x <= floor(size_lookup / 2.0); x += 1.0)
  {
    for (float y = 0.0; y <= floor(size_lookup / 2.0); y += 1.0)
    {
      for (int heading = 0; heading != search_info.dim_3_size; heading++)
      {
        Points3d motion_path;
        Point3d from(x, y, heading * angular_bin_size);
        motion_table.curve_gen->generation(from, to, motion_path);
        dist_heuristic_lookup_table[index] = motion_table.curve_gen->distance(motion_path);
        index++;
      }
    }
  }
}

void NodeHybrid::getNeighbors(std::function<bool(const uint64_t&, NodePtr&)>& NeighborGetter,
                              const std::shared_ptr<CollisionChecker>& collision_checker, bool traverse_unknown,
                              NodeVector& neighbors)
{
  NodePtr neighbor = nullptr;
  const MotionPoses motion_projections = motion_table.getProjections(this);
  for (unsigned int i = 0; i != motion_projections.size(); i++)
  {
    uint64_t index =
        NodeHybrid::getIndex({ motion_projections[i].x_, motion_projections[i].y_, motion_projections[i].theta_ });
    if (NeighborGetter(index, neighbor) && !neighbor->is_visited())
    {
      neighbor->setPose({ motion_projections[i].x_, motion_projections[i].y_, motion_projections[i].theta_ });
      if (neighbor->isValid(traverse_unknown, collision_checker))
      {
        neighbor->setMotionPrimitiveIndex(i, motion_projections[i].turn_dir_);
        neighbors.push_back(neighbor);
      }
    }
  }
}

/**
 * @brief Set the starting pose for planning, as a node index
 * @param path Reference to a vector of indices of generated path
 * @return whether the path was able to be backtraced
 */
bool NodeHybrid::backtracePath(Poses& path)
{
  if (!parent)
  {
    return false;
  }

  NodePtr current_node = this;
  while (current_node->parent)
  {
    path.push_back(current_node->pose());
    // Convert angle to radians
    path.back().setTheta(NodeHybrid::motion_table.getAngleFromBin(path.back().theta()));
    current_node = current_node->parent;
  }

  // add the start pose
  path.push_back(current_node->pose());
  // Convert angle to radians
  path.back().setTheta(NodeHybrid::motion_table.getAngleFromBin(path.back().theta()));

  return true;
}

}  // namespace path_planner
}  // namespace rmp