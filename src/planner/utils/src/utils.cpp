/***********************************************************
 *
 * @file: utils.cpp
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
#include "utils.h"

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

/**
 * @brief Overloading operator + for Node class
 * @param n another Node
 * @return  Node with current node's and input node n's values added
 */
Node Node::operator+(const Node& n) const
{
  Node result;
  result.x_ = x_ + n.x_;
  result.y_ = y_ + n.y_;
  result.g_ = g_ + n.g_;

  return result;
}

/**
 * @brief Overloading operator - for Node class
 * @param n another Node
 * @return  Node with current node's and input node n's values subtracted
 */
Node Node::operator-(const Node& n) const
{
  Node result;
  result.x_ = x_ - n.x_;
  result.y_ = y_ - n.y_;

  return result;
}

/**
 * @brief Overloading operator == for Node class
 * @param n another Node
 * @return  true if current node equals node n, else false
 */
bool Node::operator==(const Node& n) const
{
  return x_ == n.x_ && y_ == n.y_;
}

/**
 * @brief Overloading operator != for Node class
 * @param n another Node
 * @return  true if current node equals node n, else false
 */
bool Node::operator!=(const Node& n) const
{
  return !operator==(n);
}

/**
 * @brief Construct a new Plane Node object
 * @param x   x value
 * @param y   y value
 * @param g   g value, cost to get to this node
 * @param h   h value, heuritic cost of this node
 * @param id  node's id
 * @param pid node's parent's id
 */
PlaneNode::PlaneNode(int x, int y, double g, double h, int id, int pid) : Node(x, y, g, h, id, pid)
{
  (*this)[0] = x;
  (*this)[1] = y;
}

/**
 * @brief Construct a new Plane Node object
 * @param n another Node
 */
PlaneNode::PlaneNode(const Node& n) : PlaneNode(n.x_, n.y_, n.g_, n.h_, n.id_, n.pid_)
{
}

/**
 * @brief Overlaod () operator to calculate the hash of a Node
 * @param n Node for which the hash is to be calculated
 * @return  hash value, node id
 */
size_t NodeIdAsHash::operator()(const Node& n) const
{
  return n.id_;
}

/**
 * @brief Compare cost between 2 nodes
 * @param n1  one Node
 * @param n2  another Node
 * @return  true if the cost to get to n1 is greater than n2, else false
 */
bool compare_cost::operator()(const Node& n1, const Node& n2) const
{
  // Can modify this to allow tie breaks based on heuristic cost if required
  return (n1.g_ + n1.h_ > n2.g_ + n2.h_) || ((n1.g_ + n1.h_ == n2.g_ + n2.h_) && (n1.h_ > n2.h_));
}

/**
 * @brief Compare coordinates between 2 nodes
 * @param n1  one Node
 * @param n2  another Node
 * @return  true if n1 equals n2, else false
 */
bool compare_coordinates::operator()(const Node& n1, const Node& n2) const
{
  return (n1.x_ == n2.x_) && (n1.y_ == n2.y_);
}

/**
 * @brief Get permissible motion
 * @return  Node vector of permissible motions
 */
std::vector<Node> getMotion()
{
  return { Node(0, 1, 1),
           Node(1, 0, 1),
           Node(0, -1, 1),
           Node(-1, 0, 1),
           Node(1, 1, std::sqrt(2)),
           Node(1, -1, std::sqrt(2)),
           Node(-1, 1, std::sqrt(2)),
           Node(-1, -1, std::sqrt(2)) };
}

/**
 * @brief compare coordinates between 2 nodes
 * @param n1  one Node
 * @param n2  another Node
 * @return  true if n1 equals n2, else false
 */
bool compareCoordinates(const Node& n1, const Node& n2)
{
  return (n1.x_ == n2.x_) && (n1.y_ == n2.y_);
}