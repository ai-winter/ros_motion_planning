#ifndef D_STAR_LITE_H
#define D_STAR_LITE_H

#include <ros/ros.h>

#include <map>
#include <algorithm>

#include "global_planner.h"

#define INF 10000       // infinity, a big enough number
#define WINDOW_SIZE 70  // local costmap window size (in grid, 3.5m / 0.05 = 70)

namespace d_star_lite_planner
{
class LNode;
typedef LNode* LNodePtr;

class DStarLite : public global_planner::GlobalPlanner
{
public:
  DStarLite(int nx, int ny, double resolution);

  void initMap();

  void reset();

  double getH(LNodePtr s);

  double calculateKey(LNodePtr s);

  bool isCollision(LNodePtr n1, LNodePtr n2);

  void getNeighbours(LNodePtr u, std::vector<LNodePtr>& neighbours);

  double getCost(LNodePtr n1, LNodePtr n2);

  void updateVertex(LNodePtr u);

  void computeShortestPath();

  void extractPath(const Node& start, const Node& goal);

  Node getState(const Node& current);

  std::tuple<bool, std::vector<Node>> plan(const unsigned char* costs, const Node& start, const Node& goal,
                                           std::vector<Node>& expand);

public:
  // global costmap
  unsigned char* curr_global_costmap_;
  unsigned char* last_global_costmap_;
  // grid pointer map
  LNodePtr** map_;
  // open list, ascending order
  std::multimap<double, LNodePtr> open_list_;
  // path
  std::vector<Node> path_;
  // expand
  std::vector<Node> expand_;
  // start and goal
  Node start_, goal_;
  // start and goal ptr
  LNodePtr start_ptr_, goal_ptr_, last_ptr_, curr_ptr_;
  // correction
  double km_;
};

class LNode : public Node
{
public:
  LNode(const int x = 0, const int y = 0, const double cost = INF, const double h_cost = INF, const int id = 0,
        const int pid = -1, const double rhs = INF, const double key = INF)
    : Node(x, y, cost, h_cost, id, pid), rhs(rhs), key(key)
  {
  }

public:
  // minimum cost moving from start(value)
  double rhs;
  // priority
  double key;
  // iterator
  std::multimap<double, LNodePtr>::iterator open_it;
};

}  // namespace d_star_lite_planner

#endif