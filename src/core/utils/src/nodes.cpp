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

/**
 * @brief Overloading operator + for Node class
 * @param n another Node
 * @return Node with current node's and input node n's values added
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
 * @return Node with current node's and input node n's values subtracted
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
 * @return true if current node equals node n, else false
 */
bool Node::operator==(const Node& n) const
{
  return x_ == n.x_ && y_ == n.y_;
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
  return (n1.g_ + n1.h_ > n2.g_ + n2.h_) || ((n1.g_ + n1.h_ == n2.g_ + n2.h_) && (n1.h_ > n2.h_));
}

/**
 * @brief Compare coordinates between 2 nodes
 * @param n1 one Node
 * @param n2 another Node
 * @return true if n1 equals n2, else false
 */
bool Node::compare_coordinates::operator()(const Node& n1, const Node& n2) const
{
  return (n1.x_ == n2.x_) && (n1.y_ == n2.y_);
}
