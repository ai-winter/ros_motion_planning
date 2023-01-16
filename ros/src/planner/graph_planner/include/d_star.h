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