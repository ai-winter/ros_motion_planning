/**
 * *********************************************************
 *
 * @file: hybrid_a_star.cpp
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

#include "hybrid_a_star.h"

namespace global_planner
{
/**
 * @brief Constructor for 3d Node class
 * @param x   x value
 * @param y   y value
 * @param g   g value, cost to get to this node
 * @param h   h value, heuritic cost of this node
 * @param id  node's id
 * @param pid node's parent's id
 */
HybridAStar::HybridNode::HybridNode(double x, double y, double t, double g, double h, int id, int pid, int prim)
  : Node(x, y, g, h, id, pid), x_(x), y_(y), t_(t), prim_(prim)
{
}

/**
 * @brief Overloading operator + for Node class
 * @param n another Node
 * @return Node with current node's and input node n's values added
 */
HybridAStar::HybridNode HybridAStar::HybridNode::operator+(const HybridNode& n) const
{
  HybridNode result;

  result.x_ = x_ + n.x_ * cos(t_) - n.y_ * sin(t_);
  result.y_ = y_ + n.x_ * sin(t_) + n.y_ * cos(t_);
  result.t_ = helper::mod2pi(t_ + n.t_);
  result.prim_ = n.prim_;

  // forward driving
  if (prim_ < 3)
  {
    // penalize turning
    if (n.prim_ != prim_)
    {
      // penalize change of direction
      if (n.prim_ > 2)
        result.g_ = g_ + n.x_ * PENALTY_TURNING * PENALTY_COD;
      else
        result.g_ = g_ + n.x_ * PENALTY_TURNING;
    }
    else
      result.g_ = g_ + n.x_;
  }
  // reverse driving
  else
  {
    // penalize turning and reversing
    if (n.prim_ != prim_)
    {
      // penalize change of direction
      if (n.prim_ < 3)
        result.g_ = g_ + n.x_ * PENALTY_TURNING * PENALTY_COD * PENALTY_REVERSING;
      else
        result.g_ = g_ + n.x_ * PENALTY_TURNING * PENALTY_REVERSING;
    }
    else
      result.g_ = g_ + n.x_ * PENALTY_REVERSING;
  }

  return result;
}

/**
 * @brief Overloading operator == for Node class
 * @param n another Node
 * @return true if current node equals node n, else false
 */
bool HybridAStar::HybridNode::operator==(const HybridNode& n) const
{
  return (x_ == n.x_) && (y_ == n.y_) &&
         ((std::abs(t_ - n.t_) <= DELTA_HEADING) || (std::abs(t_ - n.t_) >= (2 * M_PI - DELTA_HEADING)));
}

/**
 * @brief Overloading operator != for Node class
 * @param n another Node
 * @return true if current node equals node n, else false
 */
bool HybridAStar::HybridNode::operator!=(const HybridNode& n) const
{
  return !operator==(n);
}

/**
 * @brief Get permissible motion
 * @return Node vector of permissible motions
 */
std::vector<HybridAStar::HybridNode> HybridAStar::HybridNode::getMotion()
{
  // double R = 1.0;
  double R = 1.3;

  // // 20 deg 0.349065 rad
  double alpha = 14 * M_PI / 180;

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
 * @param nx         pixel number in costmap x direction
 * @param ny         pixel number in costmap y direction
 * @param resolution costmap resolution
 * @param is_reverse whether reverse operation is allowed
 * @param max_curv   maximum curvature of model
 */
HybridAStar::HybridAStar(int nx, int ny, double resolution, bool is_reverse, double max_curv)
  : GlobalPlanner(nx, ny, resolution), is_reverse_(is_reverse), max_curv_(max_curv)
{
  dubins_gen_.setStep(1.5);
  dubins_gen_.setMaxCurv(max_curv_);
  goal_ = HybridNode();
  a_star_planner_ = new AStar(nx, ny, resolution);
}

/**
 * @brief Destory the Hybrid A* object
 */
HybridAStar::~HybridAStar()
{
  delete costmap_;
  delete a_star_planner_;
}

/**
 * @brief Hybrid A* implementation
 * @param global_costmap global costmap
 * @param start          start node
 * @param goal           goal node
 * @param path           optimal path consists of Node
 * @param expand         containing the node been search during the process
 * @return true if path found, else false
 */
bool HybridAStar::plan(const unsigned char* global_costmap, const Node& start, const Node& goal,
                       std::vector<Node>& path, std::vector<Node>& expand)
{
  return false;
}
bool HybridAStar::plan(const unsigned char* global_costmap, HybridNode& start, HybridNode& goal,
                       std::vector<Node>& path, std::vector<Node>& expand)

{
  // intialization
  path.clear();
  expand.clear();
  costmap_ = global_costmap;
  updateIndex(start);
  updateIndex(goal);

  // update heuristic map
  if (goal_ != goal)
  {
    goal_ = goal;
    double gx, gy;
    world2Map(goal.x_, goal.y_, gx, gy);
    Node h_start(gx, gy, 0, 0, grid2Index(gx, gy), 0);
    genHeurisiticMap(h_start);
  }

  // possible directions and motions
  int dir = is_reverse_ ? 6 : 3;
  const std::vector<HybridNode> motions = HybridNode::getMotion();

  // open list and closed list
  std::priority_queue<HybridNode, std::vector<HybridNode>, HybridNode::compare_cost> open_list;
  std::unordered_map<int, HybridNode> closed_list;

  open_list.push(start);

  // main process
  while (!open_list.empty())
  {
    // pop current node from open list
    HybridNode current = open_list.top();
    open_list.pop();

    // current node does not exist in closed list
    if (closed_list.find(current.id_) != closed_list.end())
      continue;

    closed_list.insert(std::make_pair(current.id_, current));
    expand.emplace_back(current.x_, current.y_, 0, 0, _worldToIndex(current.x_, current.y_));

    // goal shot
    std::vector<Node> path_dubins;
    if (std::hypot(current.x_ - goal.x_, current.y_ - goal.y_) < 50)
    {
      if (dubinsShot(current, goal, path_dubins))
      {
        path = _convertClosedListToPath(closed_list, start, current);
        std::reverse(path.begin(), path.end());
        path.insert(path.end(), path_dubins.begin(), path_dubins.end());
        std::reverse(path.begin(), path.end());
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
      if (closed_list.find(node_new.id_) != closed_list.end())
        continue;

      // next node hit the boundary or obstacle
      // prevent planning failed when the current within inflation
      if ((_worldToIndex(node_new.x_, node_new.y_) < 0) || (_worldToIndex(node_new.x_, node_new.y_) >= ns_) ||
          (node_new.t_ / DELTA_HEADING >= HEADINGS) ||
          (global_costmap[_worldToIndex(node_new.x_, node_new.y_)] >= lethal_cost_ * factor_ &&
           global_costmap[_worldToIndex(node_new.x_, node_new.y_)] >=
               global_costmap[_worldToIndex(current.x_, current.y_)]))
        continue;

      node_new.pid_ = current.id_;
      updateHeuristic(node_new);

      open_list.push(node_new);
    }
  }

  // candidate A* path
  double sx, sy, gx, gy;
  world2Map(start.x_, start.y_, sx, sy);
  world2Map(goal.x_, goal.y_, gx, gy);
  return a_star_planner_->plan(global_costmap, Node(sx, sy, 0, 0, grid2Index(sx, sy)),
                               Node(gx, gy, 0, 0, grid2Index(gx, gy)), path, expand);
}

/**
 * @brief Try using Dubins curves to connect the start and goal
 * @param start          start node
 * @param goal           goal node
 * @param path           dubins path between start and goal
 * @return true if shot successfully, else false
 */
bool HybridAStar::dubinsShot(const HybridNode& start, const HybridNode& goal, std::vector<Node>& path)
{
  double sx, sy, gx, gy;
  world2Map(start.x_, start.y_, sx, sy);
  world2Map(goal.x_, goal.y_, gx, gy);
  std::vector<std::tuple<double, double, double>> poes = { { sx, sy, start.t_ }, { gx, gy, goal.t_ } };
  std::vector<std::pair<double, double>> path_dubins;

  if (dubins_gen_.run(poes, path_dubins))
  {
    path.clear();
    for (auto const& p : path_dubins)
    {
      if (costmap_[grid2Index(p.first, p.second)] >= lethal_cost_ * factor_)
        return false;
      else
        path.emplace_back(p.first, p.second);
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
void HybridAStar::updateIndex(HybridNode& node)
{
  node.id_ = static_cast<int>(node.t_ / DELTA_HEADING) + _worldToIndex(node.x_, node.y_);
}

/**
 * @brief update the h-value of hybrid node
 * @param node hybrid node to update
 */
void HybridAStar::updateHeuristic(HybridNode& node)
{
  // Dubins cost function
  double cost_dubins = 0.0;
  // std::vector<std::tuple<double, double, double>> poes = { { node.x_, node.y_, node.t_ },
  //                                                          { goal_.x_, goal_.y_, goal_.t_ } };
  // std::vector<std::pair<double, double>> path_dubins;
  // if (dubins_gen_.run(poes, path_dubins))
  //   cost_dubins = dubins_gen_.len(path_dubins);

  // 2D search cost function
  double cost_2d = h_map_[_worldToIndex(node.x_, node.y_)].g_ * resolution_;
  node.h_ = std::max(cost_2d, cost_dubins);
}

/**
 * @brief generate heurisitic map using A* algorithm, each matric of map is the distance between it and start.
 * @param start start node
 */
void HybridAStar::genHeurisiticMap(const Node& start)
{
  // open list and closed list
  std::priority_queue<Node, std::vector<Node>, Node::compare_cost> open_list;
  std::unordered_map<int, Node> open_set;

  open_list.push(start);
  open_set.emplace(start.id_, start);

  // get all possible motions
  const std::vector<Node> motions = Node::getMotion();

  // main process
  while (!open_list.empty())
  {
    // pop current node from open list
    Node current = open_list.top();
    open_list.pop();

    h_map_.emplace(current.id_, current);

    // explore neighbor of current node
    for (const auto& motion : motions)
    {
      // explore a new node
      Node node_new = current + motion;
      node_new.id_ = grid2Index(node_new.x_, node_new.y_);

      // node_new in closed list
      if (h_map_.find(node_new.id_) != h_map_.end())
        continue;

      // next node hit the boundary or obstacle
      // prevent planning failed when the current within inflation
      if ((node_new.id_ < 0) || (node_new.id_ >= ns_))
        continue;

      if (open_set.find(node_new.id_) != open_set.end())
      {
        if (open_set[node_new.id_].g_ > node_new.g_)
          open_set[node_new.id_].g_ = node_new.g_;
      }
      else
      {
        open_list.push(node_new);
        open_set.emplace(node_new.id_, node_new);
      }
    }
  }
}

/**
 * @brief Tranform from world map(x, y) to grid map(x, y)
 * @param gx grid map x
 * @param gy grid map y
 * @param wx world map x
 * @param wy world map y
 */
void HybridAStar::_worldToGrid(double wx, double wy, int& gx, int& gy)
{
  double mx, my;
  world2Map(wx, wy, mx, my);
  map2Grid(mx, my, gx, gy);
}

/**
 * @brief Tranform from world map(x, y) to grid index(i)
 * @param wx world map x
 * @param wy world map y
 * @return index
 */
int HybridAStar::_worldToIndex(double wx, double wy)
{
  int gx, gy;
  _worldToGrid(wx, wy, gx, gy);
  return grid2Index(gx, gy);
}

/**
 * @brief Convert closed list to path
 * @param closed_list closed list
 * @param start       start node
 * @param goal        goal node
 * @return vector containing path nodes
 */
std::vector<Node> HybridAStar::_convertClosedListToPath(std::unordered_map<int, HybridNode>& closed_list,
                                                        const HybridNode& start, const HybridNode& goal)
{
  int cur_x, cur_y;
  std::vector<Node> path;
  auto current = closed_list.find(goal.id_);
  while (current->second != start)
  {
    _worldToGrid(current->second.x_, current->second.y_, cur_x, cur_y);
    path.emplace_back(cur_x, cur_y);
    auto it = closed_list.find(current->second.pid_);
    if (it != closed_list.end())
      current = it;
    else
      return {};
  }
  _worldToGrid(start.x_, start.y_, cur_x, cur_y);
  path.emplace_back(cur_x, cur_y);
  return path;
}

}  // namespace global_planner