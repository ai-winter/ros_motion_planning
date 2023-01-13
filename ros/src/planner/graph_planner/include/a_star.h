/***********************************************************
 * 
 * @file: a_star.h
 * @breif: Contains the A* (dijkstra and GBFS) planner class
 * @author: Yang Haodong
 * @update: 2022-10-27
 * @version: 1.1
 * 
 * Copyright (c) 2022， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef A_STAR_H
#define A_STAR_H

#include <queue>

#include "global_planner.h"
#include "utils.h"

namespace a_star_planner{
/**
 * @brief Class for objects that plan using the A* algorithm
 */
class AStar : public global_planner::GlobalPlanner {
    public:
        /**
         * @brief  Constructor
         * @param   nx          pixel number in costmap x direction
         * @param   ny          pixel number in costmap y direction
         * @param   resolution  costmap resolution
         * @param   dijkstra    using diksktra implementation
         * @param   gbfs        using gbfs implementation
         */
        AStar(int nx, int ny, double resolution, bool dijkstra=false, bool gbfs=false);

        /**
         * @brief A* implementation
         * @param costs     costmap
         * @param start     start node
         * @param goal      goal node
         * @param expand    containing the node been search during the process
         * @return tuple contatining a bool as to whether a path was found, and the path
         */
        std::tuple<bool, std::vector<Node>> plan(const unsigned char* costs, const Node& start,
                                                 const Node& goal, std::vector<Node> &expand);
    

    private:
        // using diksktra
        bool is_dijkstra_;
        // using greedy best first search(GBFS)
        bool is_gbfs_;
};
}
#endif
