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
#include "nodes.h"

/**
 * @brief Constructor for Node class
 * @param x   x value
 * @param y   y value
 * @param g   g value, cost to get to this node
 * @param h   h value, heuritic cost of this node
 * @param id  node's id
 * @param pid node's parent's id
 */
Node::Node(int x, int y, double g, double h, int id, int pid) : x_(x), y_(y), g_(g), h_(h), id_(id), pid_(pid)
{
}
Node::Node(const Node& n) : x_(n.x()), y_(n.y()), g_(n.g()), h_(n.h()), id_(n.id()), pid_(n.pid())
{
}

/**
 * @brief get the property of node
 */
int Node::x() const
{
  return x_;
}
int Node::y() const
{
  return y_;
}
double Node::g() const
{
  return g_;
}
double Node::h() const
{
  return h_;
}
int Node::id() const
{
  return id_;
}
int Node::pid() const
{
  return pid_;
}

/**
 * @brief set the property of node
 */
void Node::set_x(int x)
{
  x_ = x;
}
void Node::set_y(int y)
{
  y_ = y;
}
void Node::set_g(double g)
{
  g_ = g;
}
void Node::set_h(double h)
{
  h_ = h;
}
void Node::set_id(int id)
{
  id_ = id;
}
void Node::set_pid(int pid)
{
  pid_ = pid;
}

/**
 * @brief Overloading operator + for Node class
 * @param n another Node
 * @return Node with current node's and input node n's values added
 */
Node Node::operator+(const Node& n) const
{
  Node result;
  result.set_x(x_ + n.x());
  result.set_y(y_ + n.y());
  result.set_g(g_ + n.g());

  return result;
}

/**
 * @brief Overloading operator - for Node class
 * @param n another Node
 * @return Node with current node's and input node n's values subtracted
 */
Node Node::operator-(const Node& n) const
{
  Node result;
  result.set_x(x_ - n.x());
  result.set_y(y_ - n.y());

  return result;
}

/**
 * @brief Overloading operator == for Node class
 * @param n another Node
 * @return true if current node equals node n, else false
 */
bool Node::operator==(const Node& n) const
{
  return x_ == n.x() && y_ == n.y();
}

/**
 * @brief Overloading operator != for Node class
 * @param n another Node
 * @return true if current node equals node n, else false
 */
bool Node::operator!=(const Node& n) const
{
  return !operator==(n);
}

/**
 * @brief Get permissible motion
 * @return Node vector of permissible motions
 */
std::vector<Node> Node::getMotion()
{
  return {
    Node(0, 1, 1),
    Node(1, 0, 1),
    Node(0, -1, 1),
    Node(-1, 0, 1),
    Node(1, 1, std::sqrt(2)),
    Node(1, -1, std::sqrt(2)),
    Node(-1, 1, std::sqrt(2)),
    Node(-1, -1, std::sqrt(2)),
  };
}

/**
 * @brief Compare cost between 2 nodes
 * @param n1 one Node
 * @param n2 another Node
 * @return true if the cost to get to n1 is greater than n2, else false
 */
bool Node::compare_cost::operator()(const Node& n1, const Node& n2) const
{
  // Can modify this to allow tie breaks based on heuristic cost if required
  return (n1.g() + n1.h() > n2.g() + n2.h()) || ((n1.g() + n1.h() == n2.g() + n2.h()) && (n1.h() > n2.h()));
}

/**
 * @brief Compare coordinates between 2 nodes
 * @param n1 one Node
 * @param n2 another Node
 * @return true if n1 equals n2, else false
 */
bool Node::compare_coordinates::operator()(const Node& n1, const Node& n2) const
{
  return (n1.x() == n2.x()) && (n1.y() == n2.y());
}
