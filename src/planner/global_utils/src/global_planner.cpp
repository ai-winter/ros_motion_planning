/***********************************************************
 * 
 * @file: global_planner.cpp
 * @breif: Contains some implement of global planner class
 * @author: Yang Haodong
 * @update: 2022-10-24
 * @version: 2.1
 * 
 * Copyright (c) 2022， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include "global_planner.h"

namespace global_planner {
    /**
     * @brief  set or reset costmap size
     * @param   nx  pixel number in costmap x direction
     * @param   ny  pixel number in costmap y direction
     */
    void GlobalPlanner::setSize(int nx, int ny) {
        this->nx_ = nx;
        this->ny_ = ny;
        this->ns_ = nx * ny;
    }
    /**
     * @brief  set or reset costmap resolution
     * @param resolution costmap resolution
     */
    void GlobalPlanner::setResolution(double resolution) {
        this->resolution_ = resolution;
    }
    /**
     * @brief  set or reset lethal cost
     * @param lethal_cost lethal cost
     */
    void GlobalPlanner::setLethalCost(unsigned char lethal_cost) {
        this->lethal_cost_ = lethal_cost;
    }
    /**
     * @brief  et or reset neutral cost
     * @param neutral_cost neutral cost
     */     
    void GlobalPlanner::setNeutralCost(unsigned char neutral_cost) {
        this->neutral_cost_ = neutral_cost;
    }
    /**
     * @brief  set or reset obstacle factor
     * @param factor obstacle factor
     */   
    void GlobalPlanner::setFactor(double factor){
        this->factor_ = factor;
    }
    /**
     * @brief  transform between grid index(i) and grid map(x, y)
     * @param x grid map x
     * @param y grid map y
     * @return index grid index
     */
    int GlobalPlanner::grid2Index(int x, int y) {
        return x + this->nx_ * y;
    }
    /**
     * @brief  transform from grid map(x, y) to grid index(i)
     * @param x grid map x
     * @param y grid map y
     * @param y grid map y
     * @return index grid index
     */
    void GlobalPlanner::index2Grid(int index, int& x, int& y) {
        x = index % this->nx_;
        y = index / this->nx_;
    }
    /**
     * @brief  transform from costmap(x, y) to grid map(x, y)
     * @param gx grid map x
     * @param gy grid map y
     * @param mx costmap x
     * @param my costmap y
     */
    void GlobalPlanner::map2Grid(double mx, double my, int& gx, int& gy) {
        gx = (int)mx;
        gy = (int)my;
    }
    /**
     * @brief  transform from grid map(x, y) to costmap(x, y)
     * @param gx grid map x
     * @param gy grid map y
     * @param mx costmap x
     * @param my costmap y
     */
    void GlobalPlanner::grid2Map(int gx, int gy, double& mx, double& my) {
        mx = this->resolution_ * (gx + 0.5);
        my = this->resolution_ * (gy + 0.5);
    }

    /**
     * @brief convert closed list to path
     * @param closed_list   closed list
     * @param start         start node
     * @param goal          goal node
     * @return vector containing path nodes
     */
    std::vector<Node> GlobalPlanner::_convertClosedListToPath(
            std::unordered_set<Node, NodeIdAsHash, compare_coordinates>& closed_list,
            const Node& start, const Node& goal) {
        auto current = *closed_list.find(goal);
        std::vector<Node> path;
        while (!compareCoordinates(current, start)) {
            // ROS_INFO("path_id:%d, pid:%d", current.id, current.pid);
            path.push_back(current);
            auto it = closed_list.find(Node(current.pid % this->nx_, current.pid / this->nx_, 0, 0, current.pid));
            if (it != closed_list.end()) {
                current = *it;
            } else return {};
        }
        path.push_back(start);
        return path;
    }
    // std::vector<Node> GlobalPlanner::_convertClosedListToPath(std::vector<Node>& closed_list,
    //         const Node& start, const Node& goal) {
    //     std::vector<Node> path;
    //     int num = closed_list.size();
    //     int index = num - 1;
    //     for(int i = 0; i < num; i++) {
    //         path.push_back(closed_list[index]);
    //         if(closed_list[index] == start)    return path;
    //         for(int j = 0; j < num; j++) {
    //             if(closed_list[j].id == closed_list[index].pid) {
    //                 index = j;
    //                 break;
    //             }
    //         }
    //     }
    // }
}