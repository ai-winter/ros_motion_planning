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

#include <cmath>
#include <array>
#include <vector>

#define LETHAL_COST 253      // lethal cost
#define NEUTRAL_COST 50      // neutral cost
#define OBSTACLE_FACTOR 0.5  // obstacle factor

constexpr int spacing_for_grid = 10;

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
   * @return  Node with current node's and input node n's values added
   */
  Node operator+(const Node& n) const;

  /**
   * @brief Overloading operator - for Node class
   * @param n another Node
   * @return  Node with current node's and input node n's values subtracted
   */
  Node operator-(const Node& n) const;

  /**
   * @brief Overloading operator == for Node class
   * @param n another Node
   * @return  true if current node equals node n, else false
   */
  bool operator==(const Node& n) const;

  /**
   * @brief Overloading operator != for Node class
   * @param n another Node
   * @return  true if current node equals node n, else false
   */
  bool operator!=(const Node& n) const;

public:
  int x_, y_;     // x and y value
  double g_, h_;  // g value, cost to reach this node. h value, heuristic cost to reach the goal
  int id_, pid_;  // Node's index and parent's index
};

/**
 * @brief 2-d Node in plane, useful in kd-tree search
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
  PlaneNode(int x = 0, int y = 0, double g = 0.0, double h = 0.0, int id = 0, int pid = 0);

  /**
   * @brief Construct a new Plane Node object
   * @param n another Node
   */
  PlaneNode(const Node& n);

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
   * @return  hash value, node id
   */
  size_t operator()(const Node& n) const;
};

/**
 * @brief Struct created to encapsulate function compare cost between 2 Nodes.
 *        Used in with multiple algorithms and classes
 */
struct compare_cost
{
  /**
   * @brief Compare cost between 2 nodes
   * @param n1  one Node
   * @param n2  another Node
   * @return  true if the cost to get to n1 is greater than n2, else false
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
   * @param n1  one Node
   * @param n2  another Node
   * @return  true if n1 equals n2, else false
   */
  bool operator()(const Node& n1, const Node& n2) const;
};

/**
 * @brief Get permissible motion
 * @return  Node vector of permissible motions
 */
std::vector<Node> getMotion();

/**
 * @brief compare coordinates between 2 nodes
 * @param n1  one Node
 * @param n2  another Node
 * @return  true if n1 equals n2, else false
 */
bool compareCoordinates(const Node& n1, const Node& n2);

#endif  // UTILS_H