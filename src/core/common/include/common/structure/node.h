/**
 * *********************************************************
 *
 * @file: node.h
 * @brief: Contains common/commonly used nodes data strcutre
 * @author: Yang Haodong
 * @date: 2024-09-22
 * @version: 3.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_COMMON_STRUCTURE_NODE_H_
#define RMP_COMMON_STRUCTURE_NODE_H_

#include <cmath>
#include <array>
#include <vector>
#include <map>

namespace rmp
{
namespace common
{
namespace structure
{
/**
 * @brief Basic Node class
 */
template <typename T>
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
  Node(T x = 0, T y = 0, double g = 0.0, double h = 0.0, int id = 0, int pid = 0)
    : x_(x), y_(y), g_(g), h_(h), id_(id), pid_(pid){};
  Node(const Node& n) : x_(n.x()), y_(n.y()), g_(n.g()), h_(n.h()), id_(n.id()), pid_(n.pid()){};

  /**
   * @brief get the property of node
   */
  T x() const
  {
    return x_;
  };
  T y() const
  {
    return y_;
  };
  double g() const
  {
    return g_;
  };
  double h() const
  {
    return h_;
  };
  int id() const
  {
    return id_;
  };
  int pid() const
  {
    return pid_;
  };

  /**
   * @brief set the property of node
   */
  void set_x(T x)
  {
    x_ = x;
  };
  void set_y(T y)
  {
    y_ = y;
  };
  void set_g(double g)
  {
    g_ = g;
  };
  void set_h(double h)
  {
    h_ = h;
  };
  void set_id(int id)
  {
    id_ = id;
  };
  void set_pid(int pid)
  {
    pid_ = pid;
  };

  /**
   * @brief Overloading operator + for Node class
   * @param n another Node
   * @return Node with current node's and input node n's values added
   */
  Node operator+(const Node& n) const
  {
    return Node(x_ + n.x(), y_ + n.y());
  };

  /**
   * @brief Overloading operator - for Node class
   * @param n another Node
   * @return Node with current node's and input node n's values subtracted
   */
  Node operator-(const Node& n) const
  {
    return Node(x_ - n.x(), y_ - n.y());
  };

  /**
   * @brief Overloading operator == for Node class
   * @param n another Node
   * @return true if current node equals node n, else false
   */
  bool operator==(const Node& n) const
  {
    return x_ == n.x() && y_ == n.y();
  };

  /**
   * @brief Overloading operator != for Node class
   * @param n another Node
   * @return true if current node equals node n, else false
   */
  bool operator!=(const Node& n) const
  {
    return !operator==(n);
  };

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
    bool operator()(const Node& n1, const Node& n2) const
    {
      return (n1.g() + n1.h() > n2.g() + n2.h()) || ((n1.g() + n1.h() == n2.g() + n2.h()) && (n1.h() > n2.h()));
    };
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
    bool operator()(const Node& n1, const Node& n2) const
    {
      return (n1.x() == n2.x()) && (n1.y() == n2.y());
    };
  };

protected:
  T x_, y_;       // x and y value
  double g_, h_;  // g value, cost to reach this node. h value, heuristic cost to reach the goal
  int id_, pid_;  // Node's index and parent's index
};
}  // namespace structure
}  // namespace common
}  // namespace rmp
#endif  // NODES_H