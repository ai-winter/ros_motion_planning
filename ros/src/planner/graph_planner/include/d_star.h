/***********************************************************
 *
 * @file: d_star.h
 * @breif: Contains the D* planner class
 * @author: Guo Zhanyu
 * @update: 2023-01-14
 * @version: 1.0
 *
 * Copyright (c) 2023， Guo Zhanyu
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/

#ifndef D_STAR_H
#define D_STAR_H

#include "global_planner.h"
#include "utils.h"

/**
 * @brief Node class for d_star
 * @param x X value
 * @param y Y value
 * @param cost Cost to get to this node
 * @param h_cost Heuritic cost of this node
 * @param id Node's id
 * @param pid Node's parent's id
 * @param tag Node's state
 * @param k Node's minimum h_cost in history
 */
class DNode : public Node
{
public:
    // Node's state
    int tag;
    // Node's minimum h_cost in history
    double k;

public:
    DNode(const int x = 0, const int y = 0, const double cost = 0,
          const double h_cost = 0, const int id = 0, const int pid = 0,
          const int tag = 0, const double k = 0)
        : Node(x, y, cost, h_cost, id, pid), tag(tag), k(k) {}

    /**
     * @brief Overloading operator + for DNode class
     * @param p node
     * @return Node with current node's and input node p's values added
     */
    DNode operator+(const DNode &p) const;
};

namespace d_star_planner
{
    /**
     * @brief Class for objects that plan using the A* algorithm
     */
    class DStar : public global_planner::GlobalPlanner
    {
    public:
        /**
         * @brief  Constructor
         * @param   nx          pixel number in costmap x direction
         * @param   ny          pixel number in costmap y direction
         * @param   resolution  costmap resolution
         */
        DStar(int nx, int ny, double resolution);

        /**
         * @brief D* implementation
         * @param costs     costmap
         * @param start     start node
         * @param goal      goal node
         * @param expand    containing the node been search during the process
         * @return tuple contatining a bool as to whether a path was found, and the path
         */
        std::tuple<bool, std::vector<Node>> plan(const unsigned char *costs, const Node &start,
                                                 const Node &goal, std::vector<Node> &expand);
    };
}

#endif