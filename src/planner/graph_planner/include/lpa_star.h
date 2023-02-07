#ifndef LPA_STAR_H
#define LPA_STAR_H

#include <ros/ros.h>

#include <map>
#include <algorithm>

#include "global_planner.h"

#define INF 10000  // infinity, a big enough number

namespace lpa_star_planner
{
class LNode;
typedef LNode* LNodePtr;

class LPAStar : public global_planner::GlobalPlanner
{
public:
  LPAStar(int nx, int ny, double resolution);

  void initMap();

  void reset();

  double calculateKey(LNodePtr node_ptr);

  double getH(LNodePtr node_ptr);

  bool isCollision(LNodePtr n1, LNodePtr n2);

  void getNeighbours(LNodePtr node_ptr, std::vector<LNodePtr>& neighbours);

  double getCost(LNodePtr n1, LNodePtr n2);

  void updateVertex(LNodePtr node_ptr);

  void computeShortestPath();

  std::tuple<bool, std::vector<Node>> plan(const unsigned char* costs, const Node& start, const Node& goal,
                                           std::vector<Node>& expand);

public:
  // global costmap
  unsigned char* global_costmap_;
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
  LNodePtr start_ptr_, goal_ptr_;
};

class LNode : public Node
{
public:
  LNode(const int x = 0, const int y = 0, const double cost = INF, const double h_cost = INF, const int id = 0,
        const int pid = -1, const double rhs = INF, const double key = INF)
    : Node(x, y, cost, h_cost, id, pid)
  {
  }

public:
  // minimum cost moving from start(value)
  double rhs;
  // priority
  double key;
};
}  // namespace lpa_star_planner

#endif