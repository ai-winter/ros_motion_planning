/**
 * *********************************************************
 *
 * @file: lpa_star.cpp
 * @brief: Contains the LPA* planner class
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
#include "lpa_star.h"

#define WINDOW_SIZE 70  // local costmap window size (in grid, 3.5m / 0.05 = 70)

namespace global_planner
{
/**
 * @brief Construct a new LPAStar object
 * @param costmap the environment for path planning
 */
LPAStar::LPAStar(costmap_2d::Costmap2D* costmap) : GlobalPlanner(costmap)
{
  curr_global_costmap_ = new unsigned char[map_size_];
  last_global_costmap_ = new unsigned char[map_size_];
  start_.set_x(INF);
  start_.set_y(INF);
  goal_.set_x(INF);
  goal_.set_y(INF);
  initMap();
}

/**
 * @brief Init map
 */
void LPAStar::initMap()
{
  auto nx = costmap_->getSizeInCellsX();
  auto ny = costmap_->getSizeInCellsY();

  map_ = new LNodePtr*[nx];
  for (int i = 0; i < nx; i++)
  {
    map_[i] = new LNodePtr[ny];
    for (int j = 0; j < ny; j++)
    {
      map_[i][j] = new LNode(i, j, INF, INF, grid2Index(i, j), -1, INF, INF);
      map_[i][j]->open_it = open_list_.end();  // allocate empty memory
    }
  }
}

/**
 * @brief Reset the system
 */
void LPAStar::reset()
{
  auto nx = costmap_->getSizeInCellsX();
  auto ny = costmap_->getSizeInCellsY();

  open_list_.clear();

  for (int i = 0; i < nx; i++)
    for (int j = 0; j < ny; j++)
      delete map_[i][j];

  for (int i = 0; i < nx; i++)
    delete[] map_[i];

  delete[] map_;

  initMap();
}

/**
 * @brief Get heuristics between n1 and n2
 * @param n1 LNode pointer of on LNode
 * @param n2 LNode pointer of the other LNode
 * @return heuristics between n1 and n2
 */
double LPAStar::getH(LNodePtr n1, LNodePtr n2)
{
  return std::hypot(n1->x() - n2->x(), n1->y() - n2->y());
}

/**
 * @brief Calculate the key of s
 * @param s LNode pointer
 * @return the key value
 */
double LPAStar::calculateKey(LNodePtr s)
{
  return std::min(s->g(), s->rhs) + 0.9 * getH(s, goal_ptr_);
}

/**
 * @brief Check if there is collision between n1 and n2
 * @param n1 LNode pointer of one LNode
 * @param n2 LNode pointer of the other LNode
 * @return true if collision, else false
 */
bool LPAStar::isCollision(LNodePtr n1, LNodePtr n2)
{
  return (curr_global_costmap_[n1->id()] > costmap_2d::LETHAL_OBSTACLE * factor_) ||
         (curr_global_costmap_[n2->id()] > costmap_2d::LETHAL_OBSTACLE * factor_);
}

/**
 * @brief Get neighbour LNodePtrs of nodePtr
 * @param node_ptr   LNode to expand
 * @param neighbours neigbour LNodePtrs in vector
 */
void LPAStar::getNeighbours(LNodePtr node_ptr, std::vector<LNodePtr>& neighbours)
{
  auto nx = costmap_->getSizeInCellsX();
  auto ny = costmap_->getSizeInCellsY();

  int x = node_ptr->x(), y = node_ptr->y();
  for (int i = -1; i <= 1; i++)
  {
    for (int j = -1; j <= 1; j++)
    {
      if (i == 0 && j == 0)
        continue;

      int x_n = x + i, y_n = y + j;
      if (x_n < 0 || x_n >= nx || y_n < 0 || y_n >= ny)
        continue;

      LNodePtr neigbour_ptr = map_[x_n][y_n];
      if (isCollision(node_ptr, neigbour_ptr))
        continue;

      neighbours.push_back(neigbour_ptr);
    }
  }
}

/**
 * @brief Get the cost between n1 and n2, return INF if collision
 * @param n1 LNode pointer of one LNode
 * @param n2 LNode pointer of the other LNode
 * @return cost between n1 and n2
 */
double LPAStar::getCost(LNodePtr n1, LNodePtr n2)
{
  if (isCollision(n1, n2))
    return INF;

  return std::hypot(n1->x() - n2->x(), n1->y() - n2->y());
}

/**
 * @brief Update vertex u
 * @param u LNode pointer to update
 */
void LPAStar::updateVertex(LNodePtr u)
{
  // u != start
  if (u->x() != start_.x() || u->y() != start_.y())
  {
    std::vector<LNodePtr> neigbours;
    getNeighbours(u, neigbours);

    // min_{s\in pred(u)}(g(s) + c(s, u))
    u->rhs = INF;
    for (LNodePtr s : neigbours)
      if (s->g() + getCost(s, u) < u->rhs)
        u->rhs = s->g() + getCost(s, u);
  }

  // u in openlist, remove u
  if (u->open_it != open_list_.end())
  {
    open_list_.erase(u->open_it);
    u->open_it = open_list_.end();
  }

  // g(u) != rhs(u)
  if (u->g() != u->rhs)
  {
    u->key = calculateKey(u);
    u->open_it = open_list_.insert(std::make_pair(u->key, u));
  }
}

/**
 * @brief Main process of LPA*
 */
void LPAStar::computeShortestPath()
{
  while (true)
  {
    if (open_list_.empty())
      break;

    LNodePtr u = open_list_.begin()->second;
    open_list_.erase(open_list_.begin());
    u->open_it = open_list_.end();
    expand_.push_back(*u);

    // goal reached
    if (u->key >= calculateKey(goal_ptr_) && goal_ptr_->rhs == goal_ptr_->g())
      break;

    // Locally over-consistent -> Locally consistent
    if (u->g() > u->rhs)
    {
      u->set_g(u->rhs);
    }
    // Locally under-consistent -> Locally over-consistent
    else
    {
      u->set_g(INF);
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
 * @param start start node
 * @param goal  goal node
 * @return flag true if extract successfully else do not
 */
bool LPAStar::extractPath(const Node& start, const Node& goal)
{
  std::vector<Node> path_temp;
  LNodePtr node_ptr = map_[goal.x()][goal.y()];
  int count = 0;
  while (node_ptr->x() != start.x() || node_ptr->y() != start.y())
  {
    path_temp.push_back(*node_ptr);

    // argmin_{s\in pred(u)}
    std::vector<LNodePtr> neigbours;
    getNeighbours(node_ptr, neigbours);
    double min_cost = INF;
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

    // TODO: it happens to cannnot find a path to start sometimes..
    // use counter to solve it templately
    if (count++ > 1000)
      return false;
  }
  path_ = path_temp;
  return true;
}

/**
 * @brief Get the closest Node of the path to current state
 * @param current current state
 * @return the closest Node
 */
Node LPAStar::getState(const Node& current)
{
  Node state(path_[0].x(), path_[0].y());
  double dis_min = std::hypot(state.x() - current.x(), state.y() - current.y());
  int idx_min = 0;
  for (int i = 1; i < path_.size(); i++)
  {
    double dis = std::hypot(path_[i].x() - current.x(), path_[i].y() - current.y());
    if (dis < dis_min)
    {
      dis_min = dis;
      idx_min = i;
    }
  }
  state.set_x(path_[idx_min].x());
  state.set_y(path_[idx_min].y());

  return state;
}

/**
 * @brief LPA* implementation
 * @param start   start node
 * @param goal    goal node
 * @param expand  containing the node been search during the process
 * @return tuple contatining a bool as to whether a path was found, and the path
 */
bool LPAStar::plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand)
{
  // update costmap
  memcpy(last_global_costmap_, curr_global_costmap_, map_size_);
  memcpy(curr_global_costmap_, costmap_->getCharMap(), map_size_);

  expand_.clear();

  // new start or goal set
  if (start_.x() != start.x() || start_.y() != start.y() || goal_.x() != goal.x() || goal_.y() != goal.y())
  {
    reset();
    start_ = start;
    goal_ = goal;
    start_ptr_ = map_[start.x()][start.y()];
    goal_ptr_ = map_[goal.x()][goal.y()];

    start_ptr_->rhs = 0.0;
    start_ptr_->key = calculateKey(start_ptr_);
    start_ptr_->open_it = open_list_.insert(std::make_pair(start_ptr_->key, start_ptr_));

    computeShortestPath();

    // path_.clear();
    extractPath(start, goal);

    expand = expand_;
    path = path_;

    return true;
  }
  // NOTE: Unlike D* or D* lite, we cannot use history after the robot moves,
  // because we only get the optimal path from original start to goal after environment changed,
  // but not the current state to goal. To this end, we have to reset and replan.
  // Therefore, it is even worse than using A* algorithm.
  else
  {
    Node state = getState(start);

    for (int i = -WINDOW_SIZE / 2; i < WINDOW_SIZE / 2; ++i)
    {
      for (int j = -WINDOW_SIZE / 2; j < WINDOW_SIZE / 2; ++j)
      {
        int x_n = state.x() + i, y_n = state.y() + j;
        if (x_n < 0 || x_n > costmap_->getSizeInCellsX() - 1 || y_n < 0 || y_n > costmap_->getSizeInCellsY() - 1)
          continue;

        int idx = grid2Index(x_n, y_n);
        if (curr_global_costmap_[idx] != last_global_costmap_[idx])
        {
          LNodePtr u = map_[x_n][y_n];
          std::vector<LNodePtr> neigbours;
          getNeighbours(u, neigbours);
          updateVertex(u);
          for (LNodePtr s : neigbours)
            updateVertex(s);
        }
      }
    }
    computeShortestPath();

    path_.clear();
    extractPath(start_, goal);
    state = getState(start);
    std::vector<Node>::iterator state_it = std::find(path_.begin(), path_.end(), state);
    path_.assign(path_.begin(), state_it);

    path = path_;

    return true;
  }
}
}  // namespace global_planner