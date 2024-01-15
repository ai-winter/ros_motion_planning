/**
 * *********************************************************
 *
 * @file: d_star_lite.cpp
 * @brief: Contains the D* lite planner class
 * @author: Zhanyu Guo
 * @date: 2023-03-19
 * @version: 1.0
 *
 * Copyright (c) 2024, Zhanyu Guo. 
 * All rights reserved.
 * 
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "d_star_lite.h"

namespace global_planner
{
/**
 * @brief Construct a new DStarLite object
 *
 * @param nx          pixel number in costmap x direction
 * @param ny          pixel number in costmap y direction
 * @param resolution  costmap resolution
 */
DStarLite::DStarLite(int nx, int ny, double resolution) : GlobalPlanner(nx, ny, resolution)
{
  curr_global_costmap_ = new unsigned char[ns_];
  last_global_costmap_ = new unsigned char[ns_];
  start_.x_ = start_.y_ = goal_.x_ = goal_.y_ = INF;
  factor_ = 0.4;
  initMap();
}

/**
 * @brief Init map
 */
void DStarLite::initMap()
{
  map_ = new LNodePtr*[nx_];
  for (int i = 0; i < nx_; i++)
  {
    map_[i] = new LNodePtr[ny_];
    for (int j = 0; j < ny_; j++)
    {
      map_[i][j] = new LNode(i, j, INF, INF, grid2Index(i, j), -1, INF, INF);
      map_[i][j]->open_it = open_list_.end();  // allocate empty memory
    }
  }
}

/**
 * @brief Reset the system
 */
void DStarLite::reset()
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
double DStarLite::getH(LNodePtr n1, LNodePtr n2)
{
  return std::hypot(n1->x_ - n2->x_, n1->y_ - n2->y_);
}

/**
 * @brief Calculate the key of s
 *
 * @param s LNode pointer
 * @return the key value
 */
double DStarLite::calculateKey(LNodePtr s)
{
  return std::min(s->g_, s->rhs) + 0.9 * (getH(s, start_ptr_) + km_);
}

/**
 * @brief Check if there is collision between n1 and n2
 *
 * @param n1  DNode pointer of one DNode
 * @param n2  DNode pointer of the other DNode
 * @return true if collision, else false
 */
bool DStarLite::isCollision(LNodePtr n1, LNodePtr n2)
{
  return (curr_global_costmap_[n1->id_] > lethal_cost_ * factor_) ||
         (curr_global_costmap_[n2->id_] > lethal_cost_ * factor_);
}

/**
 * @brief Get neighbour LNodePtrs of nodePtr
 *
 * @param node_ptr    DNode to expand
 * @param neighbours  neigbour LNodePtrs in vector
 */
void DStarLite::getNeighbours(LNodePtr u, std::vector<LNodePtr>& neighbours)
{
  int x = u->x_, y = u->y_;
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
double DStarLite::getCost(LNodePtr n1, LNodePtr n2)
{
  if (isCollision(n1, n2))
    return INF;
  return std::hypot(n1->x_ - n2->x_, n1->y_ - n2->y_);
}

/**
 * @brief Update vertex u
 *
 * @param u LNode pointer to update
 */
void DStarLite::updateVertex(LNodePtr u)
{
  // u != goal
  if (u->x_ != goal_.x_ || u->y_ != goal_.y_)
  {
    std::vector<LNodePtr> neigbours;
    getNeighbours(u, neigbours);

    // min_{s\in pred(u)}(g(s) + c(s, u))
    u->rhs = INF;
    for (LNodePtr s : neigbours)
    {
      if (s->g_ + getCost(s, u) < u->rhs)
      {
        u->rhs = s->g_ + getCost(s, u);
      }
    }
  }

  // u in openlist, remove u
  if (u->open_it != open_list_.end())
  {
    open_list_.erase(u->open_it);
    u->open_it = open_list_.end();
  }

  // g(u) != rhs(u)
  if (u->g_ != u->rhs)
  {
    u->key = calculateKey(u);
    u->open_it = open_list_.insert(std::make_pair(u->key, u));
  }
}

/**
 * @brief Main process of D* lite
 */
void DStarLite::computeShortestPath()
{
  while (1)
  {
    if (open_list_.empty())
      break;

    double k_old = open_list_.begin()->first;
    LNodePtr u = open_list_.begin()->second;
    open_list_.erase(open_list_.begin());
    u->open_it = open_list_.end();
    expand_.push_back(*u);

    // start reached
    if (u->key >= calculateKey(start_ptr_) && start_ptr_->rhs == start_ptr_->g_)
      break;

    // affected by obstacles
    if (k_old < calculateKey(u))
    {
      u->key = calculateKey(u);
      u->open_it = open_list_.insert(std::make_pair(u->key, u));
    }
    // Locally over-consistent -> Locally consistent
    else if (u->g_ > u->rhs)
    {
      u->g_ = u->rhs;
    }
    // Locally under-consistent -> Locally over-consistent
    else
    {
      u->g_ = INF;
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
bool DStarLite::extractPath(const Node& start, const Node& goal)
{
  std::vector<Node> path_temp;
  LNodePtr node_ptr = map_[start.x_][start.y_];
  int count = 0;
  while (node_ptr->x_ != goal.x_ || node_ptr->y_ != goal.y_)
  {
    path_temp.push_back(*node_ptr);

    // argmin_{s\in pred(u)}
    std::vector<LNodePtr> neigbours;
    getNeighbours(node_ptr, neigbours);
    double min_cost = INF;
    LNodePtr next_node_ptr;
    for (LNodePtr node_n_ptr : neigbours)
    {
      if (node_n_ptr->g_ < min_cost)
      {
        min_cost = node_n_ptr->g_;
        next_node_ptr = node_n_ptr;
      }
    }
    node_ptr = next_node_ptr;

    // TODO: it happens to cannnot find a path to start sometimes...
    // use counter to solve it templately
    if (count++ > 1000)
      return false;
  }
  std::reverse(path_temp.begin(), path_temp.end());
  path_ = path_temp;
  return true;
}

/**
 * @brief Get the closest Node of the path to current state
 *
 * @param current current state
 * @return the closest Node
 */
Node DStarLite::getState(const Node& current)
{
  Node state(path_[0].x_, path_[0].y_);
  double dis_min = std::hypot(state.x_ - current.x_, state.y_ - current.y_);
  int idx_min = 0;
  for (int i = 1; i < path_.size(); i++)
  {
    double dis = std::hypot(path_[i].x_ - current.x_, path_[i].y_ - current.y_);
    if (dis < dis_min)
    {
      dis_min = dis;
      idx_min = i;
    }
  }
  state.x_ = path_[idx_min].x_;
  state.y_ = path_[idx_min].y_;

  return state;
}

/**
 * @brief D* lite implementation
 * @param global_costmap   costmap
 * @param start   start node
 * @param goal    goal node
 * @param expand  containing the node been search during the process
 * @return tuple contatining a bool as to whether a path was found, and the path
 */
bool DStarLite::plan(const unsigned char* global_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
                     std::vector<Node>& expand)
{
  // update costmap
  memcpy(last_global_costmap_, curr_global_costmap_, ns_);
  memcpy(curr_global_costmap_, global_costmap, ns_);

  expand_.clear();

  // new goal set
  if (goal_.x_ != goal.x_ || goal_.y_ != goal.y_)
  {
    reset();
    goal_ = goal;
    start_ = start;

    start_ptr_ = map_[start.x_][start.y_];
    goal_ptr_ = map_[goal.x_][goal.y_];
    last_ptr_ = start_ptr_;

    goal_ptr_->rhs = 0.0;
    goal_ptr_->key = calculateKey(goal_ptr_);
    goal_ptr_->open_it = open_list_.insert(std::make_pair(goal_ptr_->key, goal_ptr_));

    computeShortestPath();

    // path_.clear();
    extractPath(start, goal);

    expand = expand_;

    path = path_;

    return true;
  }
  else
  {
    start_ = start;
    start_ptr_ = map_[start.x_][start.y_];

    for (int i = -WINDOW_SIZE / 2; i < WINDOW_SIZE / 2; i++)
    {
      for (int j = -WINDOW_SIZE / 2; j < WINDOW_SIZE / 2; j++)
      {
        int x_n = start.x_ + i, y_n = start.y_ + j;
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
    extractPath(start, goal);

    expand = expand_;

    path = path_;

    return true;
  }
}

}  // namespace global_planner