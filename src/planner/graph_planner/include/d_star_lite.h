#ifndef D_STAR_LITE_H
#define D_STAR_LITE_H

#include <ros/ros.h>

#include <map>
#include <algorithm>

#include "global_planner.h"
#include "lpa_star.h"

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

};

class LNode : public lpa_star_planner::LNode
{
public:
  LNode(const int x = 0, const int y = 0, const double cost = INF, const double h_cost = INF, const int id = 0,
        const int pid = -1, const double rhs = INF, const double key = INF)
    : lpa_star_planner::LNode(x, y, cost, h_cost, id, pid, rhs, key)
  {
  }
};

}  // namespace d_star_lite_planner

#endif