/**
 * *********************************************************
 *
 * @file: dstar_lite_planner.cpp
 * @brief: Contains the D* lite planner class
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
#include "path_planner/graph_planner/dstar_lite_planner.h"

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
 * @brief Construct a new DStarLite object
 * @param costmap   the environment for path planning
 * @param obstacle_factor obstacle factor(greater means obstacles)
 */
DStarLitePathPlanner::DStarLitePathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor)
  : PathPlanner(costmap_ros, obstacle_factor)
{
  curr_global_costmap_ = new unsigned char[map_size_];
  last_global_costmap_ = new unsigned char[map_size_];
  start_.set_x(std::numeric_limits<int>::max());
  start_.set_y(std::numeric_limits<int>::max());
  goal_.set_x(std::numeric_limits<int>::max());
  goal_.set_y(std::numeric_limits<int>::max());
  initMap();
}

DStarLitePathPlanner::~DStarLitePathPlanner()
{
  delete curr_global_costmap_;
  delete last_global_costmap_;
}

/**
 * @brief Init map
 */
void DStarLitePathPlanner::initMap()
{
  map_ = new LNodePtr*[nx_];
  for (int i = 0; i < nx_; i++)
  {
    map_[i] = new LNodePtr[ny_];
    for (int j = 0; j < ny_; j++)
    {
      map_[i][j] =
          new LNode(i, j, std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), grid2Index(i, j), -1,
                    std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
      map_[i][j]->setIterator(open_list_.end());  // allocate empty memory
    }
  }
}

/**
 * @brief Reset the system
 */
void DStarLitePathPlanner::reset()
{
  open_list_.clear();
  km_ = 0.0;

  for (int i = 0; i < nx_; i++)
    for (int j = 0; j < ny_; j++)
      delete map_[i][j];

  for (int i = 0; i < nx_; i++)
    delete[] map_[i];

  delete[] map_;

  initMap();
}

/**
 * @brief Get heuristics between n1 and n2
 *
 * @param n1  LNode pointer of on LNode
 * @param n2  LNode pointer of the other LNode
 * @return heuristics between n1 and n2
 */
double DStarLitePathPlanner::getH(LNodePtr n1, LNodePtr n2)
{
  return std::hypot(n1->x() - n2->x(), n1->y() - n2->y());
}

/**
 * @brief Calculate the key of s
 *
 * @param s LNode pointer
 * @return the key value
 */
double DStarLitePathPlanner::calculateKey(LNodePtr s)
{
  return std::min(s->g(), s->rhs()) + 0.9 * (getH(s, start_ptr_) + km_);
}

/**
 * @brief Check if there is collision between n1 and n2
 *
 * @param n1  DNode pointer of one DNode
 * @param n2  DNode pointer of the other DNode
 * @return true if collision, else false
 */
bool DStarLitePathPlanner::isCollision(LNodePtr n1, LNodePtr n2)
{
  return (curr_global_costmap_[n1->id()] > costmap_2d::LETHAL_OBSTACLE * obstacle_factor_) ||
         (curr_global_costmap_[n2->id()] > costmap_2d::LETHAL_OBSTACLE * obstacle_factor_);
}

/**
 * @brief Get neighbour LNodePtrs of nodePtr
 *
 * @param node_ptr    DNode to expand
 * @param neighbours  neigbour LNodePtrs in vector
 */
void DStarLitePathPlanner::getNeighbours(LNodePtr u, std::vector<LNodePtr>& neighbours)
{
  int x = u->x(), y = u->y();
  for (int i = -1; i <= 1; i++)
  {
    for (int j = -1; j <= 1; j++)
    {
      if (i == 0 && j == 0)
        continue;

      int x_n = x + i, y_n = y + j;
      if (x_n < 0 || x_n > nx_ - 1 || y_n < 0 || y_n > ny_ - 1)
        continue;
      LNodePtr neigbour_ptr = map_[x_n][y_n];

      if (isCollision(u, neigbour_ptr))
        continue;

      neighbours.push_back(neigbour_ptr);
    }
  }
}

/**
 * @brief Get the cost between n1 and n2, return INF if collision
 *
 * @param n1 LNode pointer of one LNode
 * @param n2 LNode pointer of the other LNode
 * @return cost between n1 and n2
 */
double DStarLitePathPlanner::getCost(LNodePtr n1, LNodePtr n2)
{
  if (isCollision(n1, n2))
    return std::numeric_limits<double>::max();
  return std::hypot(n1->x() - n2->x(), n1->y() - n2->y());
}

/**
 * @brief Update vertex u
 *
 * @param u LNode pointer to update
 */
void DStarLitePathPlanner::updateVertex(LNodePtr u)
{
  // u != goal
  if (u->x() != goal_.x() || u->y() != goal_.y())
  {
    std::vector<LNodePtr> neigbours;
    getNeighbours(u, neigbours);

    // min_{s\in pred(u)}(g(s) + c(s, u))
    u->setRhs(std::numeric_limits<double>::max());
    for (LNodePtr s : neigbours)
    {
      if (s->g() + getCost(s, u) < u->rhs())
      {
        u->setRhs(s->g() + getCost(s, u));
      }
    }
  }

  // u in openlist, remove u
  if (u->iter() != open_list_.end())
  {
    open_list_.erase(u->iter());
    u->setIterator(open_list_.end());
  }

  // g(u) != rhs(u)
  if (u->g() != u->rhs())
  {
    u->setKey(calculateKey(u));
    u->setIterator(open_list_.insert(std::make_pair(u->key(), u)));
  }
}

/**
 * @brief Main process of D* lite
 */
void DStarLitePathPlanner::computeShortestPath()
{
  while (1)
  {
    if (open_list_.empty())
      break;

    double k_old = open_list_.begin()->first;
    LNodePtr u = open_list_.begin()->second;
    open_list_.erase(open_list_.begin());
    u->setIterator(open_list_.end());
    expand_.emplace_back((*u).x(), (*u).y());

    // start reached
    if (u->key() >= calculateKey(start_ptr_) && start_ptr_->rhs() == start_ptr_->g())
      break;

    // affected by obstacles
    if (k_old < calculateKey(u))
    {
      u->setKey(calculateKey(u));
      u->setIterator(open_list_.insert(std::make_pair(u->key(), u)));
    }
    // Locally over-consistent -> Locally consistent
    else if (u->g() > u->rhs())
    {
      u->set_g(u->rhs());
    }
    // Locally under-consistent -> Locally over-consistent
    else
    {
      u->set_g(std::numeric_limits<double>::max());
      updateVertex(u);
    }

    std::vector<LNodePtr> neigbours;
    getNeighbours(u, neigbours);
    for (LNodePtr s : neigbours)
      updateVertex(s);
  }
}

/**
 * @brief Extract path for map
 *
 * @param start start node
 * @param goal  goal node
 * @return flag true if extract successfully else do not
 */
bool DStarLitePathPlanner::extractPath(const LNode& start, const LNode& goal)
{
  Points3d path_temp;
  LNodePtr node_ptr = map_[static_cast<unsigned int>(start.x())][static_cast<unsigned int>(start.y())];
  int count = 0;
  while (node_ptr->x() != goal.x() || node_ptr->y() != goal.y())
  {
    // convert to world frame
    double wx, wy;
    costmap_->mapToWorld((*node_ptr).x(), (*node_ptr).y(), wx, wy);
    path_temp.emplace_back(wx, wy);

    // argmin_{s\in pred(u)}
    std::vector<LNodePtr> neigbours;
    getNeighbours(node_ptr, neigbours);
    double min_cost = std::numeric_limits<double>::max();
    ;
    LNodePtr next_node_ptr;
    for (LNodePtr node_n_ptr : neigbours)
    {
      if (node_n_ptr->g() < min_cost)
      {
        min_cost = node_n_ptr->g();
        next_node_ptr = node_n_ptr;
      }
    }
    node_ptr = next_node_ptr;

    // TODO: it happens to cannnot find a path to start sometimes...
    // use counter to solve it templately
    if (count++ > 1000)
      return false;
  }
  path_ = path_temp;
  return true;
}

/**
 * @brief Get the closest Node of the path to current state
 *
 * @param current current state
 * @return the closest Node
 */
DStarLitePathPlanner::LNode DStarLitePathPlanner::getState(const LNode& current)
{
  LNode state(path_[0].x(), path_[0].y());
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
 * @brief D* lite implementation
 * @param start   start node
 * @param goal    goal node
 * @param expand  containing the node been search during the process
 * @return tuple contatining a bool as to whether a path was found, and the path
 */
bool DStarLitePathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
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
  if (goal_.x() != goal.x() || goal_.y() != goal.y())
  {
    reset();
    start_.set_x(m_start_x);
    start_.set_y(m_start_y);
    start_.set_id(grid2Index(m_start_x, m_start_y));
    goal_.set_x(m_goal_x);
    goal_.set_y(m_goal_y);
    goal_.set_id(grid2Index(m_goal_x, m_goal_y));
    start_ptr_ = map_[static_cast<unsigned int>(m_start_x)][static_cast<unsigned int>(m_start_y)];
    goal_ptr_ = map_[static_cast<unsigned int>(m_goal_x)][static_cast<unsigned int>(m_goal_y)];
    last_ptr_ = start_ptr_;

    goal_ptr_->setRhs(0.0);
    goal_ptr_->setKey(calculateKey(goal_ptr_));
    goal_ptr_->setIterator(open_list_.insert(std::make_pair(goal_ptr_->key(), goal_ptr_)));

    computeShortestPath();

    extractPath(start_, goal_);

    expand = expand_;
    path = path_;

    return true;
  }
  else
  {
    start_.set_x(m_start_x);
    start_.set_y(m_start_y);
    start_.set_id(grid2Index(m_start_x, m_start_y));
    start_ptr_ = map_[static_cast<unsigned int>(m_start_x)][static_cast<unsigned int>(m_start_y)];
    for (int i = -win_size / 2; i < win_size / 2; i++)
    {
      for (int j = -win_size / 2; j < win_size / 2; j++)
      {
        int x_n = m_start_x + i, y_n = m_start_y + j;
        if (x_n < 0 || x_n > nx_ - 1 || y_n < 0 || y_n > ny_ - 1)
          continue;

        int idx = grid2Index(x_n, y_n);
        if (curr_global_costmap_[idx] != last_global_costmap_[idx])
        {
          km_ = km_ + getH(last_ptr_, start_ptr_);
          last_ptr_ = start_ptr_;

          LNodePtr u = map_[x_n][y_n];
          std::vector<LNodePtr> neigbours;
          getNeighbours(u, neigbours);
          updateVertex(u);
          for (LNodePtr s : neigbours)
          {
            updateVertex(s);
          }
        }
      }
    }
    computeShortestPath();

    path_.clear();
    extractPath(start_, goal_);

    expand = expand_;
    path = path_;

    return true;
  }
}
}  // namespace path_planner
}  // namespace rmp