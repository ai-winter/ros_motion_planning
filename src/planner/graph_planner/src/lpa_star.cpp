#include "lpa_star.h"

namespace lpa_star_planner
{
/**
 * @brief Class for objects that plan using the LPA* algorithm
 */
LPAStar::LPAStar(int nx, int ny, double resolution) : global_planner::GlobalPlanner(nx, ny, resolution)
{
  curr_global_costmap_ = new unsigned char[ns_];
  last_global_costmap_ = new unsigned char[ns_];
  start_.x = start_.y = goal_.x = goal_.y = INF;
  factor_ = 0.4;
  this->initMap();
}

/**
 * @brief Init map
 */
void LPAStar::initMap()
{
  map_ = new LNodePtr*[nx_];
  for (int i = 0; i < nx_; i++)
  {
    map_[i] = new LNodePtr[ny_];
    for (int j = 0; j < ny_; j++)
    {
      map_[i][j] = new LNode(i, j, INF, INF, this->grid2Index(i, j), -1, INF, INF);
      map_[i][j]->open_it = open_list_.end();  // allocate empty memory
    }
  }
}

/**
 * @brief Reset the system
 */
void LPAStar::reset()
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
 * @brief Get heuristics between n1 and n2
 *
 * @param n1  LNode pointer of on LNode
 * @param n2  LNode pointer of the other LNode
 * @return heuristics between n1 and n2
 */
double LPAStar::getH(LNodePtr n1, LNodePtr n2)
{
  return std::hypot(n1->x - n2->x, n1->y - n2->y);
}

/**
 * @brief Calculate the key of s
 *
 * @param s LNode pointer
 * @return the key value
 */
double LPAStar::calculateKey(LNodePtr s)
{
  return std::min(s->cost, s->rhs) + 0.9 * this->getH(s, goal_ptr_);
}

/**
 * @brief Check if there is collision between n1 and n2
 *
 * @param n1  DNode pointer of one DNode
 * @param n2  DNode pointer of the other DNode
 * @return true if collision, else false
 */
bool LPAStar::isCollision(LNodePtr n1, LNodePtr n2)
{
  return (curr_global_costmap_[n1->id] > lethal_cost_ * factor_) ||
         (curr_global_costmap_[n2->id] > lethal_cost_ * factor_);
}

/**
 * @brief Get neighbour LNodePtrs of nodePtr
 *
 * @param node_ptr    DNode to expand
 * @param neighbours  neigbour LNodePtrs in vector
 */
void LPAStar::getNeighbours(LNodePtr u, std::vector<LNodePtr>& neighbours)
{
  int x = u->x, y = u->y;
  for (int i = -1; i <= 1; i++)
  {
    for (int j = -1; j <= 1; j++)
    {
      if (i == 0 && j == 0)
        continue;

      int x_n = x + i, y_n = y + j;
      if (x_n < 0 || x_n > nx_ || y_n < 0 || y_n > ny_)
        continue;
      LNodePtr neigbour_ptr = map_[x_n][y_n];

      if (this->isCollision(u, neigbour_ptr))
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
double LPAStar::getCost(LNodePtr n1, LNodePtr n2)
{
  if (this->isCollision(n1, n2))
    return INF;
  return std::hypot(n1->x - n2->x, n1->y - n2->y);
}

/**
 * @brief Update vertex u
 *
 * @param u LNode pointer to update
 */
void LPAStar::updateVertex(LNodePtr u)
{
  // u != start
  if (u->x != start_.x || u->y != start_.y)
  {
    std::vector<LNodePtr> neigbours;
    this->getNeighbours(u, neigbours);

    // min_{s\in pred(u)}(g(s) + c(s, u))
    u->rhs = INF;
    for (LNodePtr s : neigbours)
    {
      if (s->cost + this->getCost(s, u) < u->rhs)
      {
        u->rhs = s->cost + this->getCost(s, u);
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
  if (u->cost != u->rhs)
  {
    u->key = this->calculateKey(u);
    u->open_it = open_list_.insert(std::make_pair(u->key, u));
  }
}

/**
 * @brief Main process of LPA*
 */
void LPAStar::computeShortestPath()
{
  while (1)
  {
    if (open_list_.empty())
      break;

    LNodePtr u = open_list_.begin()->second;
    open_list_.erase(open_list_.begin());
    u->open_it = open_list_.end();
    expand_.push_back(*u);

    // goal reached
    if (u->key >= this->calculateKey(goal_ptr_) && goal_ptr_->rhs == goal_ptr_->cost)
      break;

    // Locally over-consistent -> Locally consistent
    if (u->cost > u->rhs)
    {
      u->cost = u->rhs;
    }
    // Locally under-consistent -> Locally over-consistent
    else
    {
      u->cost = INF;
      this->updateVertex(u);
    }

    std::vector<LNodePtr> neigbours;
    this->getNeighbours(u, neigbours);
    for (LNodePtr s : neigbours)
      this->updateVertex(s);
  }
}

/**
 * @brief Extract path for map
 *
 * @param start start node
 * @param goal  goal node
 */
void LPAStar::extractPath(const Node& start, const Node& goal)
{
  LNodePtr node_ptr = map_[goal.x][goal.y];
  int count = 0;
  while (node_ptr->x != start.x || node_ptr->y != start.y)
  {
    path_.push_back(*node_ptr);

    // argmin_{s\in pred(u)}
    std::vector<LNodePtr> neigbours;
    this->getNeighbours(node_ptr, neigbours);
    double min_cost = INF;
    LNodePtr next_node_ptr;
    for (LNodePtr node_n_ptr : neigbours)
    {
      if (node_n_ptr->cost < min_cost)
      {
        min_cost = node_n_ptr->cost;
        next_node_ptr = node_n_ptr;
      }
    }
    node_ptr = next_node_ptr;

    // TODO: it happens to cannnot find a path to start sometimes...
    // use counter to solve it templately
    if (count++ > 1000)
      break;
  }
}

/**
 * @brief Get the closest Node of the path to current state
 *
 * @param current current state
 * @return the closest Node
 */
Node LPAStar::getState(const Node& current)
{
  Node state(path_[0].x, path_[0].y);
  double dis_min = std::hypot(state.x - current.x, state.y - current.y);
  int idx_min = 0;
  for (int i = 1; i < path_.size(); i++)
  {
    double dis = std::hypot(path_[i].x - current.x, path_[i].y - current.y);
    if (dis < dis_min)
    {
      dis_min = dis;
      idx_min = i;
    }
  }
  state.x = path_[idx_min].x;
  state.y = path_[idx_min].y;

  return state;
}

/**
 * @brief LPA* implementation
 * @param costs   costmap
 * @param start   start node
 * @param goal    goal node
 * @param expand  containing the node been search during the process
 * @return tuple contatining a bool as to whether a path was found, and the path
 */
std::tuple<bool, std::vector<Node>> LPAStar::plan(const unsigned char* costs, const Node& start, const Node& goal,
                                                  std::vector<Node>& expand)
{
  // update costmap
  memcpy(last_global_costmap_, curr_global_costmap_, ns_);
  memcpy(curr_global_costmap_, costs, ns_);

  expand_.clear();

  // new start or goal set
  if (start_.x != start.x || start_.y != start.y || goal_.x != goal.x || goal_.y != goal.y)
  {
    this->reset();
    start_ = start;
    goal_ = goal;
    start_ptr_ = map_[start.x][start.y];
    goal_ptr_ = map_[goal.x][goal.y];

    start_ptr_->rhs = 0.0;
    start_ptr_->key = this->calculateKey(start_ptr_);
    start_ptr_->open_it = open_list_.insert(std::make_pair(start_ptr_->key, start_ptr_));

    this->computeShortestPath();

    path_.clear();
    this->extractPath(start, goal);

    expand = expand_;

    return { true, path_ };
  }
  // NOTE: Unlike D* or D* lite, we cannot use history after the robot moves,
  // because we only get the optimal path from original start to goal after environment changed,
  // but not the current state to goal. To this end, we have to reset and replan.
  // Therefore, it is even worse than using A* algorithm.
  else
  {
    Node state = this->getState(start);

    for (int i = -WINDOW_SIZE / 2; i < WINDOW_SIZE / 2; i++)
    {
      for (int j = -WINDOW_SIZE / 2; j < WINDOW_SIZE / 2; j++)
      {
        int x_n = state.x + i, y_n = state.y + j;
        if (x_n < 0 || x_n > nx_ || y_n < 0 || y_n > ny_)
          continue;

        int idx = this->grid2Index(x_n, y_n);
        if (curr_global_costmap_[idx] != last_global_costmap_[idx])
        {
          LNodePtr u = map_[x_n][y_n];
          std::vector<LNodePtr> neigbours;
          this->getNeighbours(u, neigbours);
          this->updateVertex(u);
          for (LNodePtr s : neigbours)
          {
            this->updateVertex(s);
          }
        }
      }
    }
    this->computeShortestPath();

    path_.clear();
    this->extractPath(start_, goal);
    state = this->getState(start);
    std::vector<Node>::iterator state_it = std::find(path_.begin(), path_.end(), state);
    path_.assign(path_.begin(), state_it);

    return { true, path_ };
  }
}
}  // namespace lpa_star_planner