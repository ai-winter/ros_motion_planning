/**
/***********************************************************
 *
 * @file: utils.h
 * @breif: Contains common/commonly used funtions and classes
 * @author: Yang Haodong
 * @update: 2022-10-24
 * @version: 2.1
 *
 * Copyright (c) 2022， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef UTILS_H
#define UTILS_H

#include <limits>
#include <queue>
#include <unordered_set>
#include <vector>

/************************** Some useful macro-definition *************************/
// lethal cost
#define LETHAL_COST 253
// neutral cost
#define NEUTRAL_COST 50
// obstacle factor
#define OBSTACLE_FACTOR 0.5
/*********************************************************************************/

/*********************** Some useful classes and structures **********************/
/**
 * @brief Node class
 * @param x X value
 * @param y Y value
 * @param cost Cost to get to this node
 * @param h_cost Heuritic cost of this node
 * @param id Node's id
 * @param pid Node's parent's id
 */
class Node
{
public:
  // grid x coordinate
  int x;
  // grid y coordinate
  int y;
  // cost to reach this node
  double cost;
  // heuristic cost to reach the goal/
  double h_cost;
  // Node id
  int id;
  // Node's parent's id
  int pid;

  /**
   * @brief Constructor for Node class
   * @param x X value
   * @param y Y value
   * @param cost Cost to get to this node
   * @param h_cost Heuritic cost of this node
   * @param id Node's id
   * @param pid Node's parent's id
   */
  Node(const int x = 0, const int y = 0, const double cost = 0, const double h_cost = 0, const int id = 0,
       const int pid = 0)
    : x(x), y(y), cost(cost), h_cost(h_cost), id(id), pid(pid)
  {
  }

  /**
   * @brief Overloading operator + for Node class
   * @param p node
   * @return Node with current node's and input node p's values added
   */
  Node operator+(const Node& p) const;

  /**
   * @brief Overloading operator - for Node class
   * @param p node
   * @return Node with current node's and input node p's values subtracted
   */
  Node operator-(const Node& p) const;

  /**
   * @brief Overloading operator == for Node class
   * @param p node
   * @return bool whether current node equals input node
   */
  bool operator==(const Node& p) const;

  /**
   * @brief Overloading operator != for Node class
   * @param p node
   * @return bool whether current node equals input node
   */
  bool operator!=(const Node& p) const;
};

/**
 * @brief 2-d Node in plane, useful in kd-tree search
 * @param x X value
 * @param y Y value
 * @param cost Cost to get to this node
 * @param h_cost Heuritic cost of this node
 * @param id Node's id
 * @param pid Node's parent's id
 * @param dim dimentions, default is 2
 */
class PlaneNode : public Node, public std::array<int, 2>
{
public:
  PlaneNode(const int x = 0, const int y = 0, const double cost = 0, const double h_cost = 0, const int id = 0,
            const int pid = 0)
    : Node(x, y, cost, h_cost, id, pid)
  {
    (*this)[0] = x;
    (*this)[1] = y;
  }
  PlaneNode(const Node& n) : PlaneNode(n.x, n.y, n.cost, n.h_cost, n.id, n.pid)
  {
  }
  static const int dim = 2;
};

/**
 * @brief Hash for node struct that returns node id
 */
class NodeIdAsHash
{
public:
  /**
   * @brief Overlaod () operator to calculate the hash of a Node
   * @param n Node for which the hash is to be calculated
   * @return hash value
   * @details the hash returned is the node id
   */
  size_t operator()(const Node& n) const;
};

/**
 * @brief Struct created to encapsulate function compare cost between 2 nodes.
 * Used in with multiple algorithms and classes
 */
struct compare_cost
{
  /**
   * @brief Compare cost between 2 nodes
   * @param p1 Node 1
   * @param p2 Node 2
   * @return Returns whether cost to get to node 1 is greater than the cost to
   * get to node 2
   */
  bool operator()(const Node& p1, const Node& p2) const;
};

/**
 * @brief Struct created to encapsulate function compare cost between 2 nodes.
 * Used in with multiple algorithms and classes
 */
struct compare_coordinates
{
  /**
   * @brief Compare cost between 2 nodes
   * @param p1 Node 1
   * @param p2 Node 2
   * @return Returns whether cost to get to node 1 is greater than the cost to
   * get to node 2
   */
  bool operator()(const Node& p1, const Node& p2) const;
};
/*********************************************************************************/

/*************************** Some useful functions ********************************/
/**
 * @brief Get permissible motion primatives for the bot
 * @return vector of permissible motions
 */
std::vector<Node> getMotion();

/**
 * @brief compare coordinates between 2 nodes
 * @param p1 node 1
 * @param p2 node 2
 * @return whether the two nodes are for the same coordinates
 */
bool compareCoordinates(const Node& p1, const Node& p2);

/*********************************************************************************/

#endif  // UTILS_H
