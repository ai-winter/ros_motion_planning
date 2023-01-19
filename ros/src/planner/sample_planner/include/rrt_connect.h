/***********************************************************
 * 
 * @file: rrt_connect.h
 * @breif: Contains the RRT Connect planner class
 * @author: Yang Haodong
 * @update: 2023-1-18
 * @version: 1.0
 * 
 * Copyright (c) 2022， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef RRT_CONNECT_H
#define RRT_CONNECT_H


#include "global_planner.h"
#include "rrt.h"
#include "utils.h"

namespace rrt_planner {
/**
 * @brief Class for objects that plan using the RRT Connect algorithm
 */
class RRTConnect : public RRT {
    public:
        /**
         * @brief  Constructor
         * @param   nx          pixel number in costmap x direction
         * @param   ny          pixel number in costmap y direction
         * @param   resolution  costmap resolution
         * @param   sample_num  andom sample points
         * @param   max_dist    max distance between sample points
         */
        RRTConnect(int nx, int ny, double resolution, int sample_num, double max_dist);
        /**
         * @brief RRT implementation
         * @param costs     costmap
         * @param start     start node
         * @param goal      goal node
         * @param expand    containing the node been search during the process
         * @return tuple contatining a bool as to whether a path was found, and the path
         */
        std::tuple<bool, std::vector<Node>> plan(const unsigned char* costs, const Node& start,
                                                const Node& goal, std::vector<Node> &expand);
    protected:
        // Sampled list forward
        std::unordered_set<Node, NodeIdAsHash, compare_coordinates> sample_list_f_;
        // Sampled list backward
        std::unordered_set<Node, NodeIdAsHash, compare_coordinates> sample_list_b_;

        std::vector<Node> _convertClosedListToPath(const Node& boundaray);
};
}

#endif