/***********************************************************
 *
 * @file: d_star.cpp
 * @breif: Contains the D* planner class
 * @author: Guo Zhanyu
 * @update: 2023-01-14
 * @version: 1.0
 *
 * Copyright (c) 2023， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/

#include "d_star.h"

namespace d_star_planner
{
    /**
     * @brief  Constructor
     * @param   nx          pixel number in costmap x direction
     * @param   ny          pixel number in costmap y direction
     * @param   resolution  costmap resolution
     */
    DStar::DStar(int nx, int ny, double resolution) : GlobalPlanner(nx, ny, resolution) {}

    /**
     * @brief D* implementation
     * @param costs     costmap
     * @param start     start node
     * @param goal      goal node
     * @param expand    containing the node been search during the process
     * @return tuple contatining a bool as to whether a path was found, and the path
     */
    std::tuple<bool, std::vector<Node>> plan(const unsigned char *costs, const Node &start,
                                             const Node &goal, std::vector<Node> &expand)
    {
        
    }
}