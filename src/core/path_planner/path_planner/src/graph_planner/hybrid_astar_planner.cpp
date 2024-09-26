/**
 * *********************************************************
 *
 * @file: hybrid_astar_planner.cpp
 * @brief: Contains the Hybrid A* planner class
 * @author: Yang Haodong
 * @date: 2024-01-03
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <iostream>
#include <queue>
#include <unordered_set>

#include "common/math/math_helper.h"
#include "path_planner/graph_planner/hybrid_astar_planner.h"

namespace rmp
{
namespace path_planner
{
namespace
{
constexpr double kPenaltyTurning = 1.05;
constexpr double kPenaltyCod = 1.5;
constexpr double kPenaltyReversing = 1.5;
constexpr int kHeadings = 72;
constexpr double kDeltaHeading = (2 * M_PI / kHeadings);

// double R = 1.0;
constexpr double R = 1.3;

// // 20 deg 0.349065 rad
constexpr double alpha = 14 * M_PI / 180;

const std::vector<HybridAStarPathPlanner::Node> motions = {
  { 0, 1, 1.0 },          { 1, 0, 1.0 },           { 0, -1, 1.0 },          { -1, 0, 1.0 },
  { 1, 1, std::sqrt(2) }, { 1, -1, std::sqrt(2) }, { -1, 1, std::sqrt(2) }, { -1, -1, std::sqrt(2) },
};
}  // namespace

/**
 * @brief Constructor for 3d Node class
 * @param x   x value
 * @param y   y value
 * @param g   g value, cost to get to this node
 * @param h   h value, heuritic cost of this node
 * @param id  node's id
 * @param pid node's parent's id
 */
HybridAStarPathPlanner::HybridNode::HybridNode(double x, double y, double t, double g, double h, int id, int pid,
                                               int prim)
  : Node(x, y, g, h, id, pid), theta_(t), prim_(prim)
{
}

double HybridAStarPathPlanner::HybridNode::theta() const
{
  return theta_;
}

void HybridAStarPathPlanner::HybridNode::set_theta(double theta)
{
  theta_ = theta;
}

/**
 * @brief Overloading operator + for Node class
 * @param n another Node
 * @return Node with current node's and input node n's values added
 */
HybridAStarPathPlanner::HybridNode HybridAStarPathPlanner::HybridNode::operator+(const HybridNode& n) const
{
  HybridNode result;

  result.x_ = x_ + n.x_ * cos(theta_) - n.y_ * sin(theta_);
  result.y_ = y_ + n.x_ * sin(theta_) + n.y_ * cos(theta_);
  result.theta_ = rmp::common::math::mod2pi(theta_ + n.theta_);
  result.prim_ = n.prim_;

  // forward driving
  if (prim_ < 3)
  {
    // penalize turning
    if (n.prim_ != prim_)
    {
      // penalize change of direction
      if (n.prim_ > 2)
        result.set_g(g() + n.x_ * kPenaltyTurning * kPenaltyCod);
      else
        result.set_g(g() + n.x_ * kPenaltyTurning);
    }
    else
      result.set_g(g() + n.x_);
  }
  // reverse driving
  else
  {
    // penalize turning and reversing
    if (n.prim_ != prim_)
    {
      // penalize change of direction
      if (n.prim_ < 3)
        result.set_g(g() + n.x_ * kPenaltyTurning * kPenaltyCod * kPenaltyReversing);
      else
        result.set_g(g() + n.x_ * kPenaltyTurning * kPenaltyReversing);
    }
    else
      result.set_g(g() + n.x_ * kPenaltyReversing);
  }

  return result;
}

/**
 * @brief Overloading operator == for Node class
 * @param n another Node
 * @return true if current node equals node n, else false
 */
bool HybridAStarPathPlanner::HybridNode::operator==(const HybridNode& n) const
{
  return (x_ == n.x_) && (y_ == n.y_) &&
         ((std::abs(theta_ - n.theta_) <= kDeltaHeading) ||
          (std::abs(theta_ - n.theta_) >= (2 * M_PI - kDeltaHeading)));
}

/**
 * @brief Overloading operator != for Node class
 * @param n another Node
 * @return true if current node equals node n, else false
 */
bool HybridAStarPathPlanner::HybridNode::operator!=(const HybridNode& n) const
{
  return !operator==(n);
}

/**
 * @brief Get permissible motion
 * @return Node vector of permissible motions
 */
std::vector<HybridAStarPathPlanner::HybridNode> HybridAStarPathPlanner::HybridNode::getMotion()
{
  // R, alpha
  double dy[] = { 0, -R * (1 - cos(alpha)), R * (1 - cos(alpha)) };
  double dx[] = { alpha * R, R * sin(alpha), R * sin(alpha) };
  double dt[] = { 0, alpha, -alpha };

  return {
    HybridNode(dx[0], dy[0], dt[0], 0, 0, 0, 0, 0),   HybridNode(dx[1], dy[1], dt[1], 0, 0, 0, 0, 1),
    HybridNode(dx[2], dy[2], dt[2], 0, 0, 0, 0, 2),   HybridNode(-dx[0], dy[0], -dt[0], 0, 0, 0, 0, 3),
    HybridNode(-dx[1], dy[1], -dt[1], 0, 0, 0, 0, 4), HybridNode(-dx[2], dy[2], -dt[2], 0, 0, 0, 0, 5),
  };
}

/**
 * @brief Construct a new Hybrid A* object
 * @param costmap   the environment for path planning
 * @param is_reverse whether reverse operation is allowed
 * @param max_curv   maximum curvature of model
 */
HybridAStarPathPlanner::HybridAStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, bool is_reverse, double max_curv)
  : PathPlanner(costmap_ros)
  , is_reverse_(is_reverse)
  , max_curv_(max_curv)
  , dubins_gen_(std::make_unique<rmp::common::geometry::DubinsCurve>(1.5, max_curv))
  , a_star_planner_(std::make_unique<AStarPathPlanner>(costmap_ros))
  , goal_(HybridNode())
{
}

/**
 * @brief Hybrid A* implementation
 * @param start          start node
 * @param goal           goal node
 * @param path           optimal path consists of Node
 * @param expand         containing the node been search during the process
 * @return true if path found, else false
 */
bool HybridAStarPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  // intialization
  path.clear();
  expand.clear();
  HybridNode start_node(start.x(), start.y(), start.theta());
  HybridNode goal_node(goal.x(), goal.y(), goal.theta());
  updateIndex(start_node);
  updateIndex(goal_node);

  // update heuristic map
  if (goal_ != goal_node)
  {
    goal_.set_x(goal.x());
    goal_.set_y(goal.y());
    goal_.set_theta(goal.theta());
    double gx, gy;
    world2Map(goal.x(), goal.y(), gx, gy);
    HybridNode h_start(gx, gy, 0, 0, 0, grid2Index(gx, gy), 0);
    genHeurisiticMap(h_start);
  }

  // possible directions and motions
  int dir = is_reverse_ ? 6 : 3;
  const std::vector<HybridNode> motions = HybridNode::getMotion();

  // open list and closed list
  std::priority_queue<HybridNode, std::vector<HybridNode>, HybridNode::compare_cost> open_list;
  std::unordered_map<int, HybridNode> closed_list;

  open_list.push(start_node);

  // main process
  while (!open_list.empty())
  {
    // pop current node from open list
    HybridNode current = open_list.top();
    open_list.pop();

    // current node does not exist in closed list
    if (closed_list.find(current.id()) != closed_list.end())
      continue;

    closed_list.insert(std::make_pair(current.id(), current));
    expand.emplace_back(current.x(), current.y());

    // goal shot
    std::vector<HybridNode> path_dubins;
    if (std::hypot(current.x() - goal.x(), current.y() - goal.y()) < 50)
    {
      if (dubinsShot(current, goal_node, path_dubins))
      {
        const auto& backtrace = _convertClosedListToPath(closed_list, start_node, current);
        for (auto iter = backtrace.rbegin(); iter != backtrace.rend(); ++iter)
        {
          path.emplace_back(iter->x(), iter->y());
        }
        for (const auto& dubins_pt : path_dubins)
        {
          path.emplace_back(dubins_pt.x(), dubins_pt.y());
        }
        return true;
      }
    }

    // explore neighbor of current node
    for (size_t i = 0; i < dir; i++)
    {
      // explore a new node
      HybridNode node_new = current + motions[i];
      updateIndex(node_new);

      // node_new in closed list
      if (closed_list.find(node_new.id()) != closed_list.end())
        continue;

      // next node hit the boundary or obstacle
      // prevent planning failed when the current within inflation
      if ((_worldToIndex(node_new.x(), node_new.y()) < 0) || (_worldToIndex(node_new.x(), node_new.y()) >= map_size_) ||
          (node_new.theta() / kDeltaHeading >= kHeadings) ||
          (costmap_->getCharMap()[_worldToIndex(node_new.x(), node_new.y())] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
           costmap_->getCharMap()[_worldToIndex(node_new.x(), node_new.y())] >=
               costmap_->getCharMap()[_worldToIndex(current.x(), current.y())]))
        continue;

      node_new.set_pid(current.id());
      updateHeuristic(node_new);

      open_list.push(node_new);
    }
  }

  // candidate A* path
  return a_star_planner_->plan(start, goal, path, expand);
}

/**
 * @brief Try using Dubins curves to connect the start and goal
 * @param start          start node
 * @param goal           goal node
 * @param path           dubins path between start and goal
 * @return true if shot successfully, else false
 */
bool HybridAStarPathPlanner::dubinsShot(const HybridNode& start, const HybridNode& goal, std::vector<HybridNode>& path)
{
  double sx, sy, gx, gy;
  world2Map(start.x(), start.y(), sx, sy);
  world2Map(goal.x(), goal.y(), gx, gy);
  rmp::common::geometry::Points3d poes = { { sx, sy, start.theta() }, { gx, gy, goal.theta() } };
  rmp::common::geometry::Points2d path_dubins;

  if (dubins_gen_->run(poes, path_dubins))
  {
    path.clear();
    for (auto const& p : path_dubins)
    {
      if (costmap_->getCharMap()[grid2Index(p.x(), p.y())] >= costmap_2d::LETHAL_OBSTACLE * factor_)
        return false;
      else
        path.emplace_back(p.x(), p.y());
    }
    return true;
  }
  else
    return false;
}

/**
 * @brief update index of hybrid node
 * @param node hybrid node to update
 */
void HybridAStarPathPlanner::updateIndex(HybridNode& node)
{
  node.set_id(static_cast<int>(node.theta() / kDeltaHeading) + _worldToIndex(node.x(), node.y()));
}

/**
 * @brief update the h-value of hybrid node
 * @param node hybrid node to update
 */
void HybridAStarPathPlanner::updateHeuristic(HybridNode& node)
{
  // Dubins cost function
  double cost_dubins = 0.0;
  // std::vector<std::tuple<double, double, double>> poes = { { node.x_, node.y_, node.t_ },
  //                                                          { goal_.x_, goal_.y_, goal_.t_ } };
  // std::vector<std::pair<double, double>> path_dubins;
  // if (dubins_gen_.run(poes, path_dubins))
  //   cost_dubins = dubins_gen_.len(path_dubins);

  // 2D search cost function
  double cost_2d = h_map_[_worldToIndex(node.x(), node.y())].g() * costmap_->getResolution();
  node.set_h(std::max(cost_2d, cost_dubins));
}

/**
 * @brief generate heurisitic map using A* algorithm, each matric of map is the distance between it and start.
 * @param start start node
 */
void HybridAStarPathPlanner::genHeurisiticMap(const Node& start)
{
  // open list and closed list
  std::priority_queue<Node, std::vector<Node>, Node::compare_cost> open_list;
  std::unordered_map<int, Node> open_set;

  open_list.push(start);
  open_set.emplace(start.id(), start);

  // main process
  while (!open_list.empty())
  {
    // pop current node from open list
    auto current = open_list.top();
    open_list.pop();

    h_map_.emplace(current.id(), current);

    // explore neighbor of current node
    for (const auto& motion : motions)
    {
      // explore a new node
      auto node_new = current + motion;
      node_new.set_g(current.g() + motion.g());
      node_new.set_id(grid2Index(node_new.x(), node_new.y()));

      // node_new in closed list
      if (h_map_.find(node_new.id()) != h_map_.end())
        continue;

      // next node hit the boundary or obstacle
      // prevent planning failed when the current within inflation
      if ((node_new.id() < 0) || (node_new.id() >= map_size_))
        continue;

      if (open_set.find(node_new.id()) != open_set.end())
      {
        if (open_set[node_new.id()].g() > node_new.g())
          open_set[node_new.id()].set_g(node_new.g());
      }
      else
      {
        open_list.push(node_new);
        open_set.emplace(node_new.id(), node_new);
      }
    }
  }
}

/**
 * @brief Tranform from world map(x, y) to grid index(i)
 * @param wx world map x
 * @param wy world map y
 * @return index
 */
int HybridAStarPathPlanner::_worldToIndex(double wx, double wy)
{
  double gx, gy;
  world2Map(wx, wy, gx, gy);
  return grid2Index(gx, gy);
}

/**
 * @brief Convert closed list to path
 * @param closed_list closed list
 * @param start       start node
 * @param goal        goal node
 * @return vector containing path nodes
 */
std::vector<HybridAStarPathPlanner::HybridNode> HybridAStarPathPlanner::_convertClosedListToPath(
    std::unordered_map<int, HybridNode>& closed_list, const HybridNode& start, const HybridNode& goal)
{
  double cur_x, cur_y;
  std::vector<HybridNode> path;
  auto current = closed_list.find(goal.id());
  while (current->second != start)
  {
    world2Map(current->second.x(), current->second.y(), cur_x, cur_y);
    path.emplace_back(cur_x, cur_y);
    auto it = closed_list.find(current->second.pid());
    if (it != closed_list.end())
      current = it;
    else
      return {};
  }
  world2Map(start.x(), start.y(), cur_x, cur_y);
  path.emplace_back(cur_x, cur_y);
  return path;
}
}  // namespace path_planner

}  // namespace rmp