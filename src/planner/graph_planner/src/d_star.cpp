#include "d_star.h"

namespace d_star_planner
{
/**
 * @brief Construct a new DStar object
 *
 * @param nx          pixel number in costmap x direction
 * @param ny          pixel number in costmap y direction
 * @param resolution  costmap resolution
 */
DStar::DStar(int nx, int ny, double resolution) : global_planner::GlobalPlanner(nx, ny, resolution)
{
  curr_global_costmap_ = new unsigned char[ns_];
  last_global_costmap_ = new unsigned char[ns_];
  goal_.x_ = goal_.y_ = INF;
  factor_ = 0.35;
  this->initMap();
}

/**
 * @brief Init map
 */
void DStar::initMap()
{
  map_ = new DNodePtr*[nx_];
  for (int i = 0; i < nx_; i++)
  {
    map_[i] = new DNodePtr[ny_];
    for (int j = 0; j < ny_; j++)
    {
      map_[i][j] = new DNode(i, j, INF, INF, this->grid2Index(i, j), -1, NEW, INF);
    }
  }
}

/**
 * @brief Reset the system
 */
void DStar::reset()
{
  open_list_.clear();

  for (int i = 0; i < nx_; i++)
    for (int j = 0; j < ny_; j++)
      delete map_[i][j];

  for (int i = 0; i < nx_; i++)
    delete[] map_[i];

  delete[] map_;

  this->initMap();
}

/**
 * @brief Insert node_ptr into the open_list with h_new
 *
 * @param node_ptr  DNode pointer of the DNode to be inserted
 * @param h_new     new h value
 */
void DStar::insert(DNodePtr node_ptr, double h_new)
{
  if (node_ptr->t == NEW)
    node_ptr->k = h_new;
  else if (node_ptr->t == OPEN)
    node_ptr->k = std::min(node_ptr->k, h_new);
  else if (node_ptr->t == CLOSED)
    node_ptr->k = std::min(node_ptr->g_, h_new);

  node_ptr->g_ = h_new;
  node_ptr->t = OPEN;
  open_list_.insert(std::make_pair(node_ptr->k, node_ptr));
}

/**
 * @brief Check if there is collision between n1 and n2
 *
 * @param n1  DNode pointer of one DNode
 * @param n2  DNode pointer of the other DNode
 * @return true if collision, else false
 */
bool DStar::isCollision(DNodePtr n1, DNodePtr n2)
{
  return curr_global_costmap_[n1->id_] > lethal_cost_ * factor_ || curr_global_costmap_[n2->id_] > lethal_cost_ * factor_;
}

/**
 * @brief Get neighbour DNodePtrs of node_ptr
 *
 * @param node_ptr     DNode to expand
 * @param neighbours  neigbour DNodePtrs in vector
 */
void DStar::getNeighbours(DNodePtr node_ptr, std::vector<DNodePtr>& neighbours)
{
  int x = node_ptr->x_, y = node_ptr->y_;
  for (int i = -1; i <= 1; i++)
  {
    for (int j = -1; j <= 1; j++)
    {
      if (i == 0 && j == 0)
        continue;

      int x_n = x + i, y_n = y + j;
      if (x_n < 0 || x_n > nx_ || y_n < 0 || y_n > ny_)
        continue;
      DNodePtr neigbour_ptr = map_[x_n][y_n];

      // if (this->isCollision(node_ptr, neigbour_ptr))
      //   continue;

      neighbours.push_back(neigbour_ptr);
    }
  }
}

/**
 * @brief Get the cost between n1 and n2, return INF if collision
 *
 * @param n1 DNode pointer of one DNode
 * @param n2 DNode pointer of the other DNode
 * @return cost between n1 and n2
 */
double DStar::getCost(DNodePtr n1, DNodePtr n2)
{
  if (this->isCollision(n1, n2))
    return INF;
  return std::hypot(n1->x_ - n2->x_, n1->y_ - n2->y_);
}

/**
 * @brief Main process of D*
 *
 * @return k_min
 */
double DStar::processState()
{
  if (open_list_.empty())
    return -1;

  double k_old = open_list_.begin()->first;
  DNodePtr x = open_list_.begin()->second;
  open_list_.erase(open_list_.begin());
  x->t = CLOSED;
  expand_.push_back(*x);

  std::vector<DNodePtr> neigbours;
  this->getNeighbours(x, neigbours);

  // RAISE state, try to reduce k value by neibhbours
  if (k_old < x->g_)
  {
    for (DNodePtr y : neigbours)
    {
      if (y->t != NEW && y->g_ <= k_old && x->g_ > y->g_ + this->getCost(y, x))
      {
        x->pid_ = y->id_;
        x->g_ = y->g_ + this->getCost(y, x);
      }
    }
  }

  // LOWER state, cost reductions
  if (k_old == x->g_)
  {
    for (DNodePtr y : neigbours)
    {
      if (y->t == NEW || ((y->pid_ == x->id_) && (y->g_ != x->g_ + this->getCost(x, y))) ||
          ((y->pid_ != x->id_) && (y->g_ > x->g_ + this->getCost(x, y))))
      {
        y->pid_ = x->id_;
        this->insert(y, x->g_ + this->getCost(x, y));
      }
    }
  }
  else
  {
    // RAISE state
    for (DNodePtr y : neigbours)
    {
      if (y->t == NEW || ((y->pid_ == x->id_) && (y->g_ != x->g_ + this->getCost(x, y))))
      {
        y->pid_ = x->id_;
        this->insert(y, x->g_ + this->getCost(x, y));
      }
      else if (y->pid_ != x->id_ && (y->g_ > x->g_ + this->getCost(x, y)))
      {
        this->insert(x, x->g_);
      }
      else if (y->pid_ != x->id_ && (x->g_ > y->g_ + this->getCost(y, x)) && y->t == CLOSED && (y->g_ > k_old))
      {
        this->insert(y, y->g_);
      }
    }
  }
  return open_list_.begin()->first;
}

/**
 * @brief Extract the expanded Nodes (CLOSED)
 *
 * @param expand expanded Nodes in vector
 */
void DStar::extractExpand(std::vector<Node>& expand)
{
  for (int i = 0; i < nx_; i++)
  {
    for (int j = 0; j < ny_; j++)
    {
      DNodePtr node_ptr = map_[i][j];
      if (node_ptr->t == CLOSED)
        expand.push_back(*node_ptr);
    }
  }
}

/**
 * @brief Extract path for map
 *
 * @param start start node
 * @param goal  goal node
 */
void DStar::extractPath(const Node& start, const Node& goal)
{
  DNodePtr node_ptr = map_[start.x_][start.y_];
  while (node_ptr->x_ != goal.x_ || node_ptr->y_ != goal.y_)
  {
    path_.push_back(*node_ptr);

    int x, y;
    this->index2Grid(node_ptr->pid_, x, y);
    node_ptr = map_[x][y];
  }
  std::reverse(path_.begin(), path_.end());
}

/**
 * @brief Get the closest Node of the path to current state
 *
 * @param current current state
 * @return the closest Node
 */
Node DStar::getState(const Node& current)
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
 * @brief Modify the map when collision occur between x and y in path, and then do processState()
 *
 * @param x DNode pointer of one DNode
 * @param y DNode pointer of the other DNode
 */
void DStar::modify(DNodePtr x, DNodePtr y)
{
  if (x->t == CLOSED)
    this->insert(x, x->g_);
}

/**
 * @brief D* implementation
 * @param gloal_costmap   costmap
 * @param start   start node
 * @param goal    goal node
 * @param expand  containing the node been search during the process
 * @return tuple contatining a bool as to whether a path was found, and the path
 */
bool DStar::plan(const unsigned char* gloal_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
                 std::vector<Node>& expand)
{
  // update costmap
  memcpy(last_global_costmap_, curr_global_costmap_, ns_);
  memcpy(curr_global_costmap_, gloal_costmap, ns_);

  expand_.clear();

  // new goal set
  if (goal_.x_ != goal.x_ || goal_.y_ != goal.y_)
  {
    this->reset();
    goal_ = goal;

    DNodePtr start_ptr = map_[start.x_][start.y_];
    DNodePtr goal_ptr = map_[goal.x_][goal.y_];

    goal_ptr->g_ = 0;
    this->insert(goal_ptr, 0);
    while (1)
    {
      double k_min = this->processState();
      if (k_min == -1 || start_ptr->t == CLOSED)
        break;
    }

    path_.clear();
    this->extractPath(start, goal);

    // expand.clear();
    // this->extractExpand(expand);
    expand = expand_;

    path = path_;

    return true;
  }
  else
  {
    // get current state from path, argmin Euler distance
    Node state = this->getState(start);

    // prepare-repair
    for (int i = -WINDOW_SIZE / 2; i < WINDOW_SIZE / 2; i++)
    {
      for (int j = -WINDOW_SIZE / 2; j < WINDOW_SIZE / 2; j++)
      {
        int x_n = state.x_ + i, y_n = state.y_ + j;
        if (x_n < 0 || x_n > nx_ || y_n < 0 || y_n > ny_)
          continue;

        DNodePtr x = map_[x_n][y_n];
        std::vector<DNodePtr> neigbours;
        this->getNeighbours(x, neigbours);

        int idx = this->grid2Index(x_n, y_n);
        for (DNodePtr y : neigbours)
        {
          if (curr_global_costmap_[idx] != last_global_costmap_[idx])
          {
            this->modify(x, y);
            this->modify(y, x);
          }
        }
      }
    }

    // repair-replan
    DNodePtr x = map_[state.x_][state.y_];
    while (1)
    {
      double k_min = this->processState();
      if (k_min >= x->g_ || k_min == -1)
        break;
    }

    // // walk forward N points, once collision, modify
    // for (int i = 0; i < WINDOW_SIZE; i++)
    // {
    //   // goal reached
    //   if (x->pid_ == -1)
    //     break;

    //   int x_val, y_val;
    //   this->index2Grid(x->pid_, x_val, y_val);
    //   y = map_[x_val][y_val];
    //   if (this->isCollision(x, y))
    //   {
    //     // ROS_WARN("Collision on original path, modified.");
    //     // std::vector<DNodePtr> neigbours;
    //     // this->getNeighbours(y, neigbours);
    //     // for (DNodePtr n : neigbours)
    //     // {
    //     //   this->modify(n, y);
    //     // }
    //     this->modify(x, y);
    //     while (1)
    //     {
    //       double k_min = this->processState();
    //       if (k_min >= x->g_ || k_min == -1)
    //         break;
    //     }
    //     // continue;
    //     break;
    //   }
    //   x = y;
    // }

    path_.clear();
    this->extractPath(state, goal);

    // expand.clear();
    // this->extractExpand(expand);
    expand = expand_;

    path = path_;

    return true;
  }
}
}  // namespace d_star_planner
