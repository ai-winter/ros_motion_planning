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
#include <random>

#include "utils.h"

// constants
constexpr int spacing_for_grid = 10;

/******************************** Node Class *************************************/
/**
 * @brief Overloading operator + for Node class
 * @param p node
 * @return Node with current node's and input node p's values added
 */
Node Node::operator+(const Node& p) const {
    Node tmp;
    tmp.x = this->x + p.x;
    tmp.y = this->y + p.y;
    tmp.cost = this->cost + p.cost;
    return tmp;
}

/**
 * @brief Overloading operator - for Node class
 * @param p node
 * @return Node with current node's and input node p's values subtracted
 */
Node Node::operator-(const Node& p) const {
    Node tmp;
    tmp.x = this->x - p.x;
    tmp.y = this->y - p.y;
    return tmp;
}

/**
 * @brief Overloading operator == for Node class
 * @param p node
 * @return bool whether current node equals input node
 */
bool Node::operator==(const Node& p) const {
    return this->x == p.x && this->y == p.y;
}
/*********************************************************************************/


/***************************** Node Hash Class ***********************************/
/**
 * @brief Overlaod () operator to calculate the hash of a Node
 * @param n Node for which the hash is to be calculated
 * @return hash value
 * @details the hash returned is the node id
 */
size_t NodeIdAsHash::operator()(const Node& n) const {
      return n.id;
}
/*********************************************************************************/


bool compare_cost::operator()(const Node& p1, const Node& p2) const {
  // Can modify this to allow tie breaks based on heuristic cost if required
  return p1.cost + p1.h_cost > p2.cost + p2.h_cost ||
         (p1.cost + p1.h_cost == p2.cost + p2.h_cost &&
          p1.h_cost > p2.h_cost);
}
bool compare_coordinates::operator()(const Node& p1, const Node& p2)  const {
  return p1.x == p2.x && p1.y == p2.y;
}

// Possible motions for dijkstra, A*, and similar algorithms.
// Not using this for RRT & RRT* to allow random direction movements.
// TODO(vss): Consider adding option for motion restriction in RRT and RRT* by
//       replacing new node with nearest node that satisfies motion constraints


/*************************** Some useful functions ********************************/
/**
 * @brief Get permissible motion primatives for the bot
 * @return vector of permissible motions
 */
std::vector<Node> getMotion() {
  return {
    Node(0, 1, NEUTRAL_COST, 0, 0, 0),
    Node(1, 0, NEUTRAL_COST, 0, 0, 0),
    Node(0, -1, NEUTRAL_COST, 0, 0, 0),
    Node(-1, 0, NEUTRAL_COST, 0, 0, 0)
    // Node(1, 1, sqrt(2) * NEUTRAL_COST, 0, 0, 0),
    // Node(1, -1, sqrt(2) * NEUTRAL_COST, 0, 0, 0),
    // Node(-1, 1, sqrt(2) * NEUTRAL_COST, 0, 0, 0),
    // Node(-1, -1, sqrt(2) * NEUTRAL_COST, 0, 0, 0)
  };
  // NOTE: Add diagonal movements for A* and D* only after the heuristics in the
  // algorithms have been modified. Refer to README.md. The heuristics currently
  // implemented are based on Manhattan distance and dwill not account for
  // diagonal/ any other motions
}

/**
 * @brief compare coordinates between 2 nodes
 * @param p1 node 1
 * @param p2 node 2
 * @return whether the two nodes are for the same coordinates
 */
bool compareCoordinates(const Node& p1, const Node& p2) {
  return p1.x == p2.x && p1.y == p2.y;
}
/*********************************************************************************/
