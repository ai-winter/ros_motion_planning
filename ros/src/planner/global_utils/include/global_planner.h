/***********************************************************
 * 
 * @file: global_planner.h
 * @breif: Contains the abstract global planner class
 * @author: Yang Haodong
 * @update: 2022-10-24
 * @version: 2.1
 * 
 * Copyright (c) 2022， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/

#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include <iostream>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "utils.h"

/**
 *  Abstract class that is inherited by concerete implementaions of global planner
 *  classes. The Plan function is a pure virtual funciton that is overloaded
 */
namespace global_planner {
class GlobalPlanner {
    public:
        /**
         * @brief  Constructor
         * @param   nx          pixel number in costmap x direction
         * @param   ny          pixel number in costmap y direction
         * @param   resolution  costmap resolution
         */
        GlobalPlanner(int nx, int ny, double resolution) : 
            lethal_cost_(LETHAL_COST), neutral_cost_(NEUTRAL_COST), factor_(OBSTACLE_FACTOR) {
            this->setSize(nx, ny);
            this->setResolution(resolution);
        }

        /**
         * @brief Virtual destructor
         * @return No return value
         * @details default
         */
        virtual ~GlobalPlanner() = default;

        /**
         * @brief Pure virtual function that is overloadde by planner implementations
         * @param costs     costmap
         * @param start     start node
         * @param goal      goal node
         * @param expand    containing the node been search during the process
         * @return tuple contatining a bool as to whether a path was found, and the path
         */
        virtual std::tuple<bool, std::vector<Node>> plan(const unsigned char* costs, const Node& start,
                                                        const Node& goal, std::vector<Node> &expand) = 0;


        public:
            /**
             * @brief  set or reset costmap size
             * @param   nx  pixel number in costmap x direction
             * @param   ny  pixel number in costmap y direction
             */
            void setSize(int nx, int ny);
            /**
             * @brief  set or reset costmap resolution
             * @param resolution costmap resolution
             */
            void setResolution(double resolution);
            /**
             * @brief  set or reset lethal cost
             * @param lethal_cost lethal cost
             */
            void setLethalCost(unsigned char lethal_cost);
            /**
             * @brief  set or reset neutral cost
             * @param neutral_cost neutral cost
             */   
            void setNeutralCost(unsigned char neutral_cost);
            /**
             * @brief  set or reset obstacle factor
             * @param factor obstacle factor
             */   
            void setFactor(double factor);           
            /**
             * @brief  transform from grid index(i) to grid map(x, y)
             * @param x grid map x
             * @param y grid map y
             * @return index grid index
             */
            int grid2Index(int x, int y);
            /**
             * @brief  transform from grid map(x, y) to grid index(i)
             * @param x grid map x
             * @param y grid map y
             * @param y grid map y
             * @return index grid index
             */
            void index2Grid(int index, int& x, int& y);
            /**
             * @brief  transform from grid map(x, y) to costmap(x, y)
             * @param gx grid map x
             * @param gy grid map y
             * @param mx costmap x
             * @param my costmap y
             */
            void map2Grid(double mx, double my, int& gx, int& gy);
            /**
             * @brief  transform from  costmap(x, y) to grid map(x, y)
             * @param gx grid map x
             * @param gy grid map y
             * @param mx costmap x
             * @param my costmap y
             */
            void grid2Map(int gx, int gy, double& mx, double& my); 


    protected:
        // lethal cost
        unsigned char lethal_cost_;
        // neutral cost
        unsigned char neutral_cost_;
        // pixel number in costmap x direction
        int nx_;
        // pixel number in costmap y direction
        int ny_;
        // total pixel number
        int ns_;
        // costmap resolution
        double resolution_;
        // obstacle factor
        double factor_;

        /**
         * @brief convert closed list to path
         * @param closed_list   closed list
         * @param start         start node
         * @param goal          goal node
         * @return vector containing path nodes
         */
        std::vector<Node> _convertClosedListToPath(
            std::unordered_set<Node, NodeIdAsHash, compare_coordinates>& closed_list,
            const Node& start, const Node& goal);
        // std::vector<Node> _convertClosedListToPath(std::vector<Node>& closed_list,
        //     const Node& start, const Node& goal);
    };
}
#endif  // PLANNER_HPP
