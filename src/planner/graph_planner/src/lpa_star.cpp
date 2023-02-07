#include "lpa_star.h"

namespace lpa_star_planner
{
LPAStar::LPAStar(int nx, int ny, double resolution) : global_planner::GlobalPlanner(nx, ny, resolution)
{
  global_costmap_ = new unsigned char[ns_];
  initMap();
}

void LPAStar::initMap()
{
  map_ = new LNodePtr*[nx_];
  for (int i = 0; i < nx_; i++)
  {
    map_[i] = new LNodePtr[ny_];
    for (int j = 0; j < ny_; j++)
      map_[i][j] = new LNode(i, j, INF, INF, this->grid2Index(i, j), -1, INF, INF);
  }
}

void LPAStar::reset()
{
  open_list_.clear();

  for (int i = 0; i < nx_; i++)
    for (int j = 0; j < ny_; j++)
      delete map_[i][j];

  for (int i = 0; i < nx_; i++)
    delete[] map_[i];

  delete[] map_;

  initMap();
}

double LPAStar::calculateKey(LNodePtr node_ptr)
{
  return std::min(node_ptr->cost, node_ptr->rhs) + this->getH(node_ptr);
}

double LPAStar::getH(LNodePtr node_ptr)
{
  return std::hypot(node_ptr->x - goal_.x, node_ptr->y - goal_.y);
}

bool LPAStar::isCollision(LNodePtr n1, LNodePtr n2)
{
  return global_costmap_[n1->id] > lethal_cost_ * factor_ || global_costmap_[n2->id] > lethal_cost_ * factor_;
}

void LPAStar::getNeighbours(LNodePtr node_ptr, std::vector<LNodePtr>& neighbours)
{
  int x = node_ptr->x, y = node_ptr->y;
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
      if (this->isCollision(node_ptr, neigbour_ptr))
        continue;

      neighbours.push_back(neigbour_ptr);
    }
  }
}

double LPAStar::getCost(LNodePtr n1, LNodePtr n2)
{
  if (this->isCollision(n1, n2))
    return INF;
  return std::hypot(n1->x - n2->x, n1->y - n2->y);
}

void LPAStar::updateVertex(LNodePtr node_ptr)
{
  std::vector<LNodePtr> neigbours;
  this->getNeighbours(node_ptr, neigbours);

  // greed correction
  for (int i = 0; i < (int)neigbours.size(); i++)
  {
    LNodePtr node_n_ptr = neigbours[i];
    if (node_n_ptr->cost + this->getCost(node_n_ptr, node_ptr) < node_ptr->rhs)
    {
      node_ptr->rhs = node_n_ptr->cost + this->getCost(node_n_ptr, node_ptr);
      node_ptr->pid = node_n_ptr->id;
    }
  }

  // Locally unconsistent nodes should be added into OPEN set (set U)
  if (node_ptr->cost != node_ptr->rhs)
  {
    node_ptr->key = this->calculateKey(node_ptr);
    open_list_.insert(std::make_pair(node_ptr->key, node_ptr));
  }
}

void LPAStar::computeShortestPath()
{
  while (1)
  {
    if (open_list_.empty())
      break;

    LNodePtr u = open_list_.begin()->second;
    if (u->key >= this->calculateKey(goal_ptr_) && goal_ptr_->rhs == goal_ptr_->cost)
      break;

    open_list_.erase(open_list_.begin());
    expand_.push_back(*u);

    // Locally over-consistent -> Locally consistent
    if (u->cost > u->rhs)
    {
      u->cost = u->rhs;
    }
    else
    {
      u->cost = INF;
      this->updateVertex(u);
    }

    std::vector<LNodePtr> neigbours;
    this->getNeighbours(u, neigbours);
    for (int i = 0; i < (int)neigbours.size(); i++)
    {
      this->updateVertex(neigbours[i]);
    }
  }
}

std::tuple<bool, std::vector<Node>> LPAStar::plan(const unsigned char* costs, const Node& start, const Node& goal,
                                                  std::vector<Node>& expand)
{
  // update costmap
  memcpy(global_costmap_, costs, ns_);
  expand_ = expand;

  if (goal_.x != goal.x || goal_.y != goal.y)
  {
    this->reset();
    goal_ = goal;

    start_ptr_ = map_[start.x][start.y];
    goal_ptr_ = map_[goal.x][goal.y];

    start_ptr_->rhs = 0;
    start_ptr_->key = this->calculateKey(start_ptr_);
    open_list_.insert(std::make_pair(start_ptr_->key, start_ptr_));

    computeShortestPath();

    path_.clear();
    // this->extractPath();
    
  }
}

}  // namespace lpa_star_planner