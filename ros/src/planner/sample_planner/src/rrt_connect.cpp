/***********************************************************
 * 
 * @file: rrt_connect.cpp
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
#include <cmath>

#include "rrt_connect.h"

namespace rrt_planner {
    /**
     * @brief  Constructor
     * @param   nx          pixel number in costmap x direction
     * @param   ny          pixel number in costmap y direction
     * @param   sample_num  andom sample points
     * @param   max_dist    max distance between sample points
     */
    RRTConnect::RRTConnect(int nx, int ny, double resolution, int sample_num, double max_dist)
      : RRT(nx, ny, resolution, sample_num, max_dist) {}

    /**
     * @brief RRT-Connect implementation
     * @param costs     costmap
     * @param start     start node
     * @param goal      goal node
     * @param expand    containing the node been search during the process
     * @return tuple contatining a bool as to whether a path was found, and the path
     */
    std::tuple<bool, std::vector<Node>> RRTConnect::plan(const unsigned char* costs, const Node& start,
                                                  const Node& goal, std::vector<Node> &expand) {
      this->sample_list_f_.clear();
      this->sample_list_b_.clear();
      // copy
      this->start_ = start, this->goal_ = goal;
      this->costs_ = costs;
      this->sample_list_f_.insert(start);
      this->sample_list_b_.insert(goal);
      expand.push_back(start);
      expand.push_back(goal);
      
      // main loop
      int iteration = 0;
      while (iteration < this->sample_num_) {
        // generate a random node in the map
        Node sample_node = this->_generateRandomNode();

        // obstacle
        if (costs[sample_node.id] >= this->lethal_cost_ * this->factor_)
            continue;
        
        // visited
        if (this->sample_list_.find(sample_node) != this->sample_list_.end())
            continue;

        // regular the sample node
        Node new_node = this->_findNearestPoint(this->sample_list_f_, sample_node);
        if (new_node.id == -1)
            continue;
        else {
            this->sample_list_f_.insert(new_node);
            expand.push_back(new_node);
            // backward exploring
            Node new_node_b = this->_findNearestPoint(this->sample_list_b_, new_node);
            if (new_node_b.id != -1) {
                this->sample_list_b_.insert(new_node_b);
                expand.push_back(new_node_b);
                // greedy extending
                while (true) {
                    double dist = std::min(this->max_dist_, this->_dist(new_node, new_node_b));
                    double theta = this->_angle(new_node_b, new_node);
                    Node new_node_b2;
                    new_node_b2.x = new_node_b.x + (int)(dist * cos(theta));
                    new_node_b2.y = new_node_b.y + (int)(dist * sin(theta));
                    new_node_b2.id = this->grid2Index(new_node_b2.x, new_node_b2.y);
                    new_node_b2.pid = new_node_b.id;
                    new_node_b2.cost = dist + new_node_b.cost;

                    if (!this->_isAnyObstacleInPath(new_node_b, new_node_b2)) {
                        this->sample_list_b_.insert(new_node_b2);
                        expand.push_back(new_node_b2);
                        new_node_b = new_node_b2;
                    } else break;

                    // connected -> goal found
                    if (new_node_b == new_node) 
                        return {true, this->_convertClosedListToPath(new_node_b)};
                }
            }

        }

        // swap
        if (this->sample_list_b_.size() < this->sample_list_f_.size())
            std::swap(this->sample_list_f_, this->sample_list_b_);
          
        iteration++;
      }
      return {false, {}};
    }

    /**
     * @brief convert closed list to path
     * @param boundary  connected node that the boudary of forward and backward
     * @return ector containing path nodes
     */
    std::vector<Node> RRTConnect::_convertClosedListToPath(const Node& boundary) {
        if (this->sample_list_f_.find(this->start_) == this->sample_list_.end())
            std::swap(this->sample_list_f_, this->sample_list_b_);
        
        std::vector<Node> path;
        
        // backward
        std::vector<Node> path_b;
        auto current = *this->sample_list_b_.find(boundary);
        while (current != this->goal_) {
            path_b.push_back(current);
            auto it = this->sample_list_b_.find(Node(current.pid % this->nx_, current.pid / this->nx_, 0, 0, current.pid));
            if (it != this->sample_list_b_.end()) {
                current = *it;
            } else return {};
        }
        path_b.push_back(this->goal_);

        // forward
        for (auto rit = path_b.rbegin(); rit != path_b.rend(); rit++)
            path.push_back(*rit);

        current = *this->sample_list_f_.find(boundary);
        while (current != this->start_) {
            auto it = this->sample_list_f_.find(Node(current.pid % this->nx_, current.pid / this->nx_, 0, 0, current.pid));
            if (it != this->sample_list_f_.end()) {
                current = *it;
            } else return {};
            path.push_back(current);
        }

        return path;
    }
}