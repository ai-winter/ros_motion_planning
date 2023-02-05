#ifndef D_STAR_H
#define D_STAR_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <map>
#include <algorithm>

#include "global_planner.h"

#define inf 10000  // infinity, a big enough number

enum Tag
{
  NEW = 0,
  OPEN = 1,
  CLOSED = 2
};

namespace d_star_planner
{
class DNode;
typedef DNode* DNodePtr;

/**
 * @brief Class for objects that plan using the D* algorithm
 */
class DStar : public global_planner::GlobalPlanner
{
public:
  /**
   * @brief Construct a new DStar object
   *
   * @param nx          pixel number in costmap x direction
   * @param ny          pixel number in costmap y direction
   * @param resolution  costmap resolution
   */
  DStar(int nx, int ny, double resolution);

  /**
   * @brief Init DNodeMap
   */
  void initMap();

  /**
   * @brief Reset the system
   */
  void reset();

  /**
   * @brief Insert nodePtr into the open_list with h_new
   *
   * @param nodePtr DNode pointer of the DNode to be inserted
   * @param h_new   new h value
   */
  void insert(DNodePtr nodePtr, double h_new);

  /**
   * @brief Judge if there is collision between n1 and n2
   *
   * @param n1  DNode pointer of one DNode
   * @param n2  DNode pointer of the other DNode
   * @return true if collision
   */
  bool isCollision(DNodePtr n1, DNodePtr n2);

  /**
   * @brief Get neighbour DNodePtrs of nodePtr
   *
   * @param nodePtr     DNode to expand
   * @param neighbours  neigbour DNodePtrs in vector
   */
  void getNeighbours(DNodePtr nodePtr, std::vector<DNodePtr>& neighbours);

  /**
   * @brief Get the cost between n1 and n2, return inf if collision
   *
   * @param n1 DNode pointer of one DNode
   * @param n2 DNode pointer of the other DNode
   * @return cost between n1 and n2
   */
  double cost(DNodePtr n1, DNodePtr n2);

  /**
   * @brief Main process of D*
   *
   * @return k_min
   */
  double processState();

  /**
   * @brief Extract the expanded Nodes (CLOSED)
   *
   * @param expand expanded Nodes in vector
   */
  void extractExpand(std::vector<Node>& expand);

  /**
   * @brief Extract path for DNodeMap
   *
   * @param start start node
   * @param goal  goal node
   */
  void extractPath(const Node& start, const Node& goal);

  /**
   * @brief Get the closest Node of the path to current state
   *
   * @param current current state
   * @return the closest Node
   */
  Node getState(const Node& current);

  /**
   * @brief Modify the map when collision occur between x and y in path, and then do processState()
   *
   * @param x DNode pointer of one DNode
   * @param y DNode pointer of the other DNode
   */
  void modify(DNodePtr x, DNodePtr y);

  /**
   * @brief D* implementation
   * @param costs   costmap
   * @param start   start node
   * @param goal    goal node
   * @param expand  containing the node been search during the process
   * @return tuple contatining a bool as to whether a path was found, and the path
   */
  std::tuple<bool, std::vector<Node>> plan(const unsigned char* costs, const Node& start, const Node& goal,
                                           std::vector<Node>& expand);

public:
  // global costmap
  unsigned char* global_costmap;
  // grid pointer map
  DNodePtr** DNodeMap;
  // open list, ascending order
  std::multimap<double, DNodePtr> open_list;
  // path
  std::vector<Node> path;
  // last goal
  Node goal_;
  // forward distance, like range of the sensor (in grid)
  int N_;
};

class DNode : public Node
{
public:
  /**
   * @brief Construct a new DNode object
   * @param x       X value
   * @param y       Y value
   * @param cost    Cost to get to this node
   * @param h_cost  Heuritic cost of this node
   * @param id      Node's id
   * @param pid     Node's parent's id
   * @param tag     Node's tag among enum Tag
   * @param k       Node's k_min in history
   */
  DNode(const int x = 0, const int y = 0, const double cost = inf, const double h_cost = inf, const int id = 0,
        const int pid = -1, const int tag = NEW, const double k = inf)
    : Node(x, y, cost, h_cost, id, pid), tag(tag), k(k)
  {
  }

public:
  // Node's tag among enum Tag
  int tag;
  // Node's k_min in history
  double k;
  // Node's iterator from multimap
  std::multimap<double, DNodePtr>::iterator nodeMapIt;
};
}  // namespace d_star_planner

#endif
