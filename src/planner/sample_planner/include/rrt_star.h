/***********************************************************
 * 
 * @file: rrt_star.h
 * @breif: Contains the Rapidly-Exploring Random Tree Star(RRT*) planner class
 * @author: Yang Haodong
 * @update: 2022-10-29
 * @version: 1.0
 * 
 * Copyright (c) 2022， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef RRT_STAR_H
#define RRT_STAR_H

#include <limits>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "global_planner.h"
#include "rrt.h"
#include "utils.h"

namespace rrt_planner {
/**
 * @brief Class for objects that plan using the RRT algorithm
 */
class RRTStar : public RRT {
    public:
        /**
         * @brief  Constructor
         * @param   nx          pixel number in costmap x direction
         * @param   ny          pixel number in costmap y direction
         * @param   resolution  costmap resolution
         * @param   sample_num  andom sample points
         * @param   max_dist    max distance between sample points
         * @param   r           optimization radius
         */
        RRTStar(int nx, int ny, double resolution, int sample_num, double max_dist, double r);
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
        /**
         * @brief Regular the new node by the nearest node in the sample list 
         * @param new_node new sample node
         * @return None
         */
        void _findNearestPoint(Node& new_node);

        double r_;
};
}
#endif  // RRT_H
