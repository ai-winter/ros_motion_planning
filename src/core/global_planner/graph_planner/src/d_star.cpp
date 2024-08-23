/**
 * *********************************************************
 *
 * @file: d_star.cpp
 * @brief: Contains the D* planner class
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
#include "d_star.h"

#define WINDOW_SIZE 70  // local costmap window size (in grid, 3.5m / 0.05 = 70)

namespace global_planner
{
/**
 * @brief Construct a new DStar object
 * @param costmap the environment for path planning
 */
DStar::DStar(costmap_2d::Costmap2D* costmap) : GlobalPlanner(costmap)
{
  curr_global_costmap_ = new unsigned char[map_size_];
  last_global_costmap_ = new unsigned char[map_size_];
  goal_.set_x(INF);
  goal_.set_y(INF);
  initMap();
}

/**
 * @brief Init map
 */
void DStar::initMap()
{
  auto nx = costmap_->getSizeInCellsX();
  auto ny = costmap_->getSizeInCellsY();

  map_ = new DNodePtr*[nx];
  for (int i = 0; i < nx; i++)
  {
    map_[i] = new DNodePtr[ny];
    for (int j = 0; j < ny; j++)
      map_[i][j] = new DNode(i, j, INF, INF, grid2Index(i, j), -1, DNode::NEW, INF);
  }
}

/**
 * @brief Reset the system
 */
void DStar::reset()
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
 * @brief Insert node_ptr into the open_list with h_new
 * @param node_ptr DNode pointer of the DNode to be inserted
 * @param h_new    new h value
 */
void DStar::insert(DNodePtr node_ptr, double h_new)
{
  if (node_ptr->t_ == DNode::NEW)
    node_ptr->k_ = h_new;
  else if (node_ptr->t_ == DNode::OPEN)
    node_ptr->k_ = std::min(node_ptr->k_, h_new);
  else if (node_ptr->t_ == DNode::CLOSED)
    node_ptr->k_ = std::min(node_ptr->g(), h_new);

  node_ptr->set_g(h_new);
  node_ptr->t_ = DNode::OPEN;
  open_list_.insert(std::make_pair(node_ptr->k_, node_ptr));
}

/**
 * @brief Check if there is collision between n1 and n2
 * @param n1 DNode pointer of one DNode
 * @param n2 DNode pointer of the other DNode
 * @return true if collision, else false
 */
bool DStar::isCollision(DNodePtr n1, DNodePtr n2)
{
  return curr_global_costmap_[n1->id()] > costmap_2d::LETHAL_OBSTACLE * factor_ ||
         curr_global_costmap_[n2->id()] > costmap_2d::LETHAL_OBSTACLE * factor_;
}

/**
 * @brief Get neighbour DNodePtrs of node_ptr
 * @param node_ptr   DNode to expand
 * @param neighbours neigbour DNodePtrs in vector
 */
void DStar::getNeighbours(DNodePtr node_ptr, std::vector<DNodePtr>& neighbours)
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
double DStar::getCost(DNodePtr n1, DNodePtr n2)
{
  if (isCollision(n1, n2))
    return INF;

  return std::hypot(n1->x() - n2->x(), n1->y() - n2->y());
}

/**
 * @brief Main process of D*
 * @return k_min
 */
double DStar::processState()
{
  if (open_list_.empty())
    return -1;

  double k_old = open_list_.begin()->first;
  DNodePtr x = open_list_.begin()->second;
  open_list_.erase(open_list_.begin());
  x->t_ = DNode::CLOSED;
  expand_.push_back(*x);

  std::vector<DNodePtr> neigbours;
  getNeighbours(x, neigbours);

  // RAISE state, try to reduce k value by neibhbours
  if (k_old < x->g())
  {
    for (DNodePtr y : neigbours)
    {
      if (y->t_ != DNode::NEW && y->g() <= k_old && x->g() > y->g() + getCost(y, x))
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
      if (y->t_ == DNode::NEW || ((y->pid() == x->id()) && (y->g() != x->g() + getCost(x, y))) ||
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
      if (y->t_ == DNode::NEW || ((y->pid() == x->id()) && (y->g() != x->g() + getCost(x, y))))
      {
        y->set_pid(x->id());
        insert(y, x->g() + getCost(x, y));
      }
      else if (y->pid() != x->id() && (y->g() > x->g() + getCost(x, y)))
      {
        insert(x, x->g());
      }
      else if (y->pid() != x->id() && (x->g() > y->g() + getCost(y, x)) && y->t_ == DNode::CLOSED && (y->g() > k_old))
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
void DStar::extractExpand(std::vector<Node>& expand)
{
  auto nx = costmap_->getSizeInCellsX();
  auto ny = costmap_->getSizeInCellsY();

  for (int i = 0; i < nx; i++)
  {
    for (int j = 0; j < ny; j++)
    {
      DNodePtr node_ptr = map_[i][j];
      if (node_ptr->t_ == DNode::CLOSED)
        expand.push_back(*node_ptr);
    }
  }
}

/**
 * @brief Extract path for map
 * @param start start node
 * @param goal  goal node
 */
void DStar::extractPath(const Node& start, const Node& goal)
{
  DNodePtr node_ptr = map_[start.x()][start.y()];
  while (node_ptr->x() != goal.x() || node_ptr->y() != goal.y())
  {
    path_.push_back(*node_ptr);

    int x, y;
    index2Grid(node_ptr->pid(), x, y);
    node_ptr = map_[x][y];
  }
  std::reverse(path_.begin(), path_.end());
}

/**
 * @brief Get the closest Node of the path to current state
 * @param current current state
 * @return the closest Node
 */
Node DStar::getState(const Node& current)
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
 * @brief Modify the map when collision occur between x and y in path, and then do processState()
 * @param x DNode pointer of one DNode
 */
void DStar::modify(DNodePtr x)
{
  if (x->t_ == DNode::CLOSED)
    insert(x, x->g());
}

/**
 * @brief D* implementation
 * @param start  start node
 * @param goal   goal node
 * @param expand containing the node been search during the process
 * @return tuple contatining a bool as to whether a path was found, and the path
 */
bool DStar::plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand)
{
  // update costmap
  memcpy(last_global_costmap_, curr_global_costmap_, map_size_);
  memcpy(curr_global_costmap_, costmap_->getCharMap(), map_size_);

  expand_.clear();

  // new goal set
  if (goal_.x() != goal.x() || goal_.y() != goal.y())
  {
    reset();
    goal_ = goal;

    DNodePtr start_ptr = map_[start.x()][start.y()];
    DNodePtr goal_ptr = map_[goal.x()][goal.y()];

    goal_ptr->set_g(0.0);
    insert(goal_ptr, 0);
    while (1)
    {
      double k_min = processState();
      if (k_min == -1 || start_ptr->t_ == DNode::CLOSED)
        break;
    }

    path_.clear();
    extractPath(start, goal);

    expand = expand_;
    path = path_;

    return true;
  }
  else
  {
    // get current state from path, argmin Euler distance
    Node state = getState(start);

    auto nx = costmap_->getSizeInCellsX();
    auto ny = costmap_->getSizeInCellsY();

    // prepare-repair
    for (int i = -WINDOW_SIZE / 2; i < WINDOW_SIZE / 2; i++)
    {
      for (int j = -WINDOW_SIZE / 2; j < WINDOW_SIZE / 2; j++)
      {
        int x_n = state.x() + i, y_n = state.y() + j;
        if (x_n < 0 || x_n >= nx || y_n < 0 || y_n >= ny)
          continue;

        DNodePtr x = map_[x_n][y_n];
        std::vector<DNodePtr> neigbours;
        getNeighbours(x, neigbours);

        int idx = grid2Index(x_n, y_n);
        if (curr_global_costmap_[idx] != last_global_costmap_[idx])
        {
          modify(x);
          for (DNodePtr y : neigbours)
            modify(y);
        }
      }
    }

    // repair-replan
    DNodePtr x = map_[state.x()][state.y()];
    while (true)
    {
      double k_min = processState();
      if (k_min >= x->g() || k_min == -1)
        break;
    }

    path_.clear();
    extractPath(state, goal);

    expand = expand_;
    path = path_;

    return true;
  }
}
}  // namespace global_planner
