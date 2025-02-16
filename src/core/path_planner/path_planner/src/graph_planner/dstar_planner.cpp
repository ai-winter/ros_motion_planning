/**
 * *********************************************************
 *
 * @file: dstar_planner.cpp
 * @brief: Contains the D* planner class
 * @author: Zhanyu Guo, Haodong Yang
 * @date: 2023-03-19
 * @version: 1.0
 *
 * Copyright (c) 2024, Zhanyu Guo, Haodong Yang.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <costmap_2d/cost_values.h>

#include "path_planner/graph_planner/dstar_planner.h"

namespace rmp
{
namespace path_planner
{
namespace
{
// local costmap window size (in grid, 3.5m / 0.05 = 70)
constexpr int win_size = 70;
}  // namespace

/**
 * @brief Construct a new DStar object
 * @param costmap   the environment for path planning
 * @param obstacle_factor obstacle factor(greater means obstacles)
 */
DStarPathPlanner::DStarPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor)
  : PathPlanner(costmap_ros, obstacle_factor)
{
  curr_global_costmap_ = new unsigned char[map_size_];
  last_global_costmap_ = new unsigned char[map_size_];
  goal_.set_x(std::numeric_limits<int>::max());
  goal_.set_y(std::numeric_limits<int>::max());
  initMap();
}

/**
 * @brief Init map
 */
void DStarPathPlanner::initMap()
{
  map_ = new DNodePtr*[nx_];
  for (int i = 0; i < nx_; ++i)
  {
    map_[i] = new DNodePtr[ny_];
    for (int j = 0; j < ny_; ++j)
    {
      map_[i][j] = new DNode(i, j, std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
                             grid2Index(i, j), -1, DNode::NEW, std::numeric_limits<double>::max());
    }
  }
}

/**
 * @brief Reset the system
 */
void DStarPathPlanner::reset()
{
  open_list_.clear();

  for (int i = 0; i < nx_; ++i)
    for (int j = 0; j < ny_; ++j)
      delete map_[i][j];

  for (int i = 0; i < nx_; ++i)
    delete[] map_[i];

  delete[] map_;

  initMap();
}

/**
 * @brief Insert node_ptr into the open_list with h_new
 * @param node_ptr DNode pointer of the DNode to be inserted
 * @param h_new    new h value
 */
void DStarPathPlanner::insert(DNodePtr node_ptr, double h_new)
{
  if (node_ptr->t() == DNode::NEW)
    node_ptr->setK(h_new);
  else if (node_ptr->t() == DNode::OPEN)
    node_ptr->setK(std::min(node_ptr->k(), h_new));
  else if (node_ptr->t() == DNode::CLOSED)
    node_ptr->setK(std::min(node_ptr->g(), h_new));

  node_ptr->set_g(h_new);
  node_ptr->setTag(DNode::OPEN);
  open_list_.insert(std::make_pair(node_ptr->k(), node_ptr));
}

/**
 * @brief Check if there is collision between n1 and n2
 * @param n1 DNode pointer of one DNode
 * @param n2 DNode pointer of the other DNode
 * @return true if collision, else false
 */
bool DStarPathPlanner::isCollision(DNodePtr n1, DNodePtr n2)
{
  return curr_global_costmap_[n1->id()] > costmap_2d::LETHAL_OBSTACLE * obstacle_factor_ ||
         curr_global_costmap_[n2->id()] > costmap_2d::LETHAL_OBSTACLE * obstacle_factor_;
}

/**
 * @brief Get neighbour DNodePtrs of node_ptr
 * @param node_ptr   DNode to expand
 * @param neighbours neigbour DNodePtrs in vector
 */
void DStarPathPlanner::getNeighbours(DNodePtr node_ptr, std::vector<DNodePtr>& neighbours)
{
  int x = node_ptr->x(), y = node_ptr->y();
  for (int i = -1; i <= 1; ++i)
  {
    for (int j = -1; j <= 1; ++j)
    {
      if (i == 0 && j == 0)
        continue;

      int x_n = x + i, y_n = y + j;
      if (x_n < 0 || x_n > nx_ - 1 || y_n < 0 || y_n > ny_ - 1)
        continue;

      DNodePtr neigbour_ptr = map_[x_n][y_n];
      neighbours.push_back(neigbour_ptr);
    }
  }
}

/**
 * @brief Get the cost between n1 and n2, return INF if collision
 * @param n1 DNode pointer of one DNode
 * @param n2 DNode pointer of the other DNode
 * @return cost between n1 and n2
 */
double DStarPathPlanner::getCost(DNodePtr n1, DNodePtr n2)
{
  if (isCollision(n1, n2))
    return std::numeric_limits<double>::max();

  return std::hypot(n1->x() - n2->x(), n1->y() - n2->y());
}

/**
 * @brief Main process of D*
 * @return k_min
 */
double DStarPathPlanner::processState()
{
  if (open_list_.empty())
    return -1;

  double k_old = open_list_.begin()->first;
  DNodePtr x = open_list_.begin()->second;
  open_list_.erase(open_list_.begin());
  x->setTag(DNode::CLOSED);
  expand_.emplace_back((*x).x(), (*x).y());

  std::vector<DNodePtr> neigbours;
  getNeighbours(x, neigbours);

  // RAISE state, try to reduce k value by neibhbours
  if (k_old < x->g())
  {
    for (DNodePtr y : neigbours)
    {
      if (y->t() != DNode::NEW && y->g() <= k_old && x->g() > y->g() + getCost(y, x))
      {
        x->set_pid(y->id());
        x->set_g(y->g() + getCost(y, x));
      }
    }
  }

  // LOWER state, cost reductions
  if (k_old == x->g())
  {
    for (DNodePtr y : neigbours)
    {
      if (y->t() == DNode::NEW || ((y->pid() == x->id()) && (y->g() != x->g() + getCost(x, y))) ||
          ((y->pid() != x->id()) && (y->g() > x->g() + getCost(x, y))))
      {
        y->set_pid(x->id());
        insert(y, x->g() + getCost(x, y));
      }
    }
  }
  else
  {
    // RAISE state
    for (DNodePtr y : neigbours)
    {
      if (y->t() == DNode::NEW || ((y->pid() == x->id()) && (y->g() != x->g() + getCost(x, y))))
      {
        y->set_pid(x->id());
        insert(y, x->g() + getCost(x, y));
      }
      else if (y->pid() != x->id() && (y->g() > x->g() + getCost(x, y)))
      {
        insert(x, x->g());
      }
      else if (y->pid() != x->id() && (x->g() > y->g() + getCost(y, x)) && y->t() == DNode::CLOSED && (y->g() > k_old))
      {
        insert(y, y->g());
      }
    }
  }
  return open_list_.begin()->first;
}

/**
 * @brief Extract the expanded Nodes (CLOSED)
 * @param expand expanded Nodes in vector
 */
void DStarPathPlanner::extractExpand(std::vector<DNode>& expand)
{
  for (int i = 0; i < nx_; ++i)
  {
    for (int j = 0; j < ny_; ++j)
    {
      DNodePtr node_ptr = map_[i][j];
      if (node_ptr->t() == DNode::CLOSED)
        expand.push_back(*node_ptr);
    }
  }
}

/**
 * @brief Extract path for map
 * @param start start node
 * @param goal  goal node
 */
void DStarPathPlanner::extractPath(const DNode& start, const DNode& goal)
{
  DNodePtr node_ptr = map_[static_cast<unsigned int>(start.x())][static_cast<unsigned int>(start.y())];
  while (node_ptr->x() != goal.x() || node_ptr->y() != goal.y())
  {
    // convert to world frame
    double wx, wy;
    costmap_->mapToWorld((*node_ptr).x(), (*node_ptr).y(), wx, wy);
    path_.emplace_back(wx, wy);

    int x, y;
    index2Grid(node_ptr->pid(), x, y);
    node_ptr = map_[x][y];
  }
}

/**
 * @brief Get the closest DNode of the path to current state
 * @param current current state
 * @return the closest DNode
 */
DStarPathPlanner::DNode DStarPathPlanner::getState(const DNode& current)
{
  DNode state(path_[0].x(), path_[0].y());
  double dis_min = std::hypot(state.x() - current.x(), state.y() - current.y());
  int idx_min = 0;
  for (size_t i = 1; i < path_.size(); i++)
  {
    double dis = std::hypot(path_[i].x() - current.x(), path_[i].y() - current.y());
    if (dis < dis_min)
    {
      dis_min = dis;
      idx_min = static_cast<int>(i);
    }
  }
  state.set_x(path_[idx_min].x());
  state.set_y(path_[idx_min].y());

  return state;
}

/**
 * @brief Modify the map when collision occur between x and y in path, and then do processState()
 * @param x DNode pointer of one DNode
 */
void DStarPathPlanner::modify(DNodePtr x)
{
  if (x->t() == DNode::CLOSED)
    insert(x, x->g());
}

/**
 * @brief D* implementation
 * @param start          start node
 * @param goal           goal node
 * @param expand         containing the node been search during the process
 * @return tuple contatining a bool as to whether a path was found, and the path
 */
bool DStarPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  double m_start_x, m_start_y, m_goal_x, m_goal_y;
  if ((!validityCheck(start.x(), start.y(), m_start_x, m_start_y)) ||
      (!validityCheck(goal.x(), goal.y(), m_goal_x, m_goal_y)))
  {
    return false;
  }

  // update costmap
  memcpy(last_global_costmap_, curr_global_costmap_, map_size_);
  memcpy(curr_global_costmap_, costmap_->getCharMap(), map_size_);

  expand_.clear();

  // new goal set
  if (goal_.x() != m_goal_x || goal_.y() != m_goal_y)
  {
    reset();
    goal_.set_x(m_goal_x);
    goal_.set_y(m_goal_y);
    goal_.set_id(grid2Index(goal_.x(), goal_.y()));

    DNodePtr start_ptr = map_[static_cast<unsigned int>(m_start_x)][static_cast<unsigned int>(m_start_y)];
    DNodePtr goal_ptr = map_[static_cast<unsigned int>(m_goal_x)][static_cast<unsigned int>(m_goal_y)];

    goal_ptr->set_g(0.0);
    insert(goal_ptr, 0);
    while (1)
    {
      double k_min = processState();
      if (k_min == -1 || start_ptr->t() == DNode::CLOSED)
        break;
    }

    path_.clear();
    extractPath({ static_cast<int>(m_start_x), static_cast<int>(m_start_y) }, goal_);

    expand = expand_;
    path = path_;

    return true;
  }
  else
  {
    // get current state from path, argmin Euler distance
    DNode state = getState({ static_cast<int>(m_start_x), static_cast<int>(m_start_y) });

    // prepare-repair
    for (int i = -win_size / 2; i < win_size / 2; ++i)
    {
      for (int j = -win_size / 2; j < win_size / 2; ++j)
      {
        int x_n = state.x() + i, y_n = state.y() + j;
        if (x_n < 0 || x_n > nx_ - 1 || y_n < 0 || y_n > ny_ - 1)
          continue;

        DNodePtr x = map_[x_n][y_n];
        std::vector<DNodePtr> neigbours;
        getNeighbours(x, neigbours);

        int idx = grid2Index(x_n, y_n);
        if (curr_global_costmap_[idx] != last_global_costmap_[idx])
        {
          modify(x);
          for (DNodePtr y : neigbours)
          {
            modify(y);
          }
        }
      }
    }

    // repair-replan
    DNodePtr x = map_[static_cast<unsigned int>(state.x())][static_cast<unsigned int>(state.y())];
    while (1)
    {
      double k_min = processState();
      if (k_min >= x->g() || k_min == -1)
        break;
    }

    path_.clear();
    extractPath(state, goal_);

    expand = expand_;
    path = path_;

    return true;
  }
}
}  // namespace path_planner
}  // namespace rmp