/**
 * *********************************************************
 *
 * @file: nodes.h
 * @brief: Contains common/commonly used nodes data strcutre
 * @author: Yang Haodong
 * @date: 2023-07-21
 * @version: 2.1
 *
 * Copyright (c) 2024, Yang Haodong. 
 * All rights reserved.
 * 
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef NODES_H
#define NODES_H

#define INF 10000  // infinity, a big enough number

#include <cmath>
#include <array>
#include <vector>
#include <map>

/**
 * @brief Basic Node class
 */
class Node
{
public:
  /**
   * @brief Constructor for Node class
   * @param x   x value
   * @param y   y value
   * @param g   g value, cost to get to this node
   * @param h   h value, heuritic cost of this node
   * @param id  node's id
   * @param pid node's parent's id
   */
  Node(int x = 0, int y = 0, double g = 0.0, double h = 0.0, int id = 0, int pid = 0);

  /**
   * @brief Overloading operator + for Node class
   * @param n another Node
   * @return Node with current node's and input node n's values added
   */
  Node operator+(const Node& n) const;

  /**
   * @brief Overloading operator - for Node class
   * @param n another Node
   * @return Node with current node's and input node n's values subtracted
   */
  Node operator-(const Node& n) const;

  /**
   * @brief Overloading operator == for Node class
   * @param n another Node
   * @return true if current node equals node n, else false
   */
  bool operator==(const Node& n) const;

  /**
   * @brief Overloading operator != for Node class
   * @param n another Node
   * @return true if current node equals node n, else false
   */
  bool operator!=(const Node& n) const;

  /**
   * @brief Get permissible motion
   * @return  Node vector of permissible motions
   */
  static std::vector<Node> getMotion();

  /**
   * @brief Struct created to encapsulate function compare cost between 2 Nodes.
   *        Used in with multiple algorithms and classes
   */
  struct compare_cost
  {
    /**
     * @brief Compare cost between 2 nodes
     * @param n1 one Node
     * @param n2 another Node
     * @return true if the cost to get to n1 is greater than n2, else false
     */
    bool operator()(const Node& n1, const Node& n2) const;
  };

  /**
   * @brief Struct created to encapsulate function compare coordinates between 2 Nodes.
   *        Used in with multiple algorithms and classes
   */
  struct compare_coordinates
  {
    /**
     * @brief Compare coordinates between 2 nodes
     * @param n1 one Node
     * @param n2 another Node
     * @return true if n1 equals n2, else false
     */
    bool operator()(const Node& n1, const Node& n2) const;
  };

public:
  int x_, y_;     // x and y value
  double g_, h_;  // g value, cost to reach this node. h value, heuristic cost to reach the goal
  int id_, pid_;  // Node's index and parent's index
};

/* =====================================================================================
 * Some other nodes defined by user.
   =====================================================================================*/

/**
 * @brief 2-d Node in plane, used in kd-tree search
 */
class PlaneNode : public Node, public std::array<int, 2>
{
public:
  /**
   * @brief Construct a new Plane Node object
   * @param x   x value
   * @param y   y value
   * @param g   g value, cost to get to this node
   * @param h   h value, heuritic cost of this node
   * @param id  node's id
   * @param pid node's parent's id
   */
  PlaneNode(int x = 0, int y = 0, double g = 0.0, double h = 0.0, int id = 0, int pid = 0) : Node(x, y, g, h, id, pid)
  {
    (*this)[0] = x;
    (*this)[1] = y;
  }

  /**
   * @brief Construct a new Plane Node object
   * @param n another Node
   */
  PlaneNode(const Node& n) : PlaneNode(n.x_, n.y_, n.g_, n.h_, n.id_, n.pid_)
  {
  }

  static const int dim = 2;
};

class DNode : public Node
{
public:
  enum Tag
  {
    NEW = 0,
    OPEN = 1,
    CLOSED = 2
  };
  /**
   * @brief Construct a new DNode object
   * @param x   X value
   * @param y   Y value
   * @param g   Cost to get to this node
   * @param h   Heuritic cost of this node
   * @param id  Node's id
   * @param pid Node's parent's id
   * @param t   Node's tag among enum Tag
   * @param k   Node's k_min in history
   */
  DNode(const int x = 0, const int y = 0, const double g = INF, const double h = INF, const int id = 0,
        const int pid = -1, const int t = NEW, const double k = INF)
    : Node(x, y, g, h, id, pid), t_(t), k_(k)
  {
  }

public:
  int t_;     // Node's tag among enum Tag
  double k_;  // Node's k_min in history
};

class LNode : public Node
{
public:
  /**
   * @brief Construct a new LNode object
   * @param x      X value
   * @param y      Y value
   * @param cost   Cost to get to this node
   * @param h_cost Heuritic cost of this node
   * @param id     Node's id
   * @param pid    Node's parent's id
   * @param rhs    Node's right hand side
   * @param key    Node's key value
   */
  LNode(const int x = 0, const int y = 0, const double cost = INF, const double h_cost = INF, const int id = 0,
        const int pid = -1, const double rhs = INF, const double key = INF)
    : Node(x, y, cost, h_cost, id, pid), rhs(rhs), key(key)
  {
  }

public:
  double rhs;                                       // minimum cost moving from start(value)
  double key;                                       // priority
  std::multimap<double, LNode*>::iterator open_it;  // iterator
};



#endif  // NODES_H