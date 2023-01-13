/***********************************************************
 * 
 * @file: rrt_star.cpp
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
#include <cmath>
#include <random>

#include "rrt_star.h"

namespace rrt_planner {
    /**
     * @brief  Constructor
     * @param   nx          pixel number in costmap x direction
     * @param   ny          pixel number in costmap y direction
     * @param   resolution  costmap resolution
     * @param   sample_num  andom sample points
     * @param   max_dist    max distance between sample points
     * @param   r           optimization radius
     */
    RRTStar::RRTStar(int nx, int ny, double resolution, int sample_num, double max_dist, double r)
        : RRT(nx, ny, resolution, sample_num, max_dist), r_(r) {}
    /**
     * @brief RRT implementation
     * @param costs     costmap
     * @param start     start node
     * @param goal      goal node
     * @param expand    containing the node been search during the process
     * @return tuple contatining a bool as to whether a path was found, and the path
     */
    std::tuple<bool, std::vector<Node>> RRTStar::plan(const unsigned char* costs, const Node& start,
                                                      const Node& goal, std::vector<Node> &expand) {
      this->sample_list_.clear();
      // copy
      this->start_ = start, this->goal_ = goal;
      this->costs_ = costs;
      this->sample_list_.insert(start);
      expand.push_back(start);
      
      // main loop
      int iteration = 0;
      while (iteration < this->sample_num_) {
        // generate a random node in the map
        Node new_node = this->_generateRandomNode();

        // obstacle
        if (costs[new_node.id] >= this->lethal_cost_ * this->factor_)
            continue;
        
        // visited
        if (this->sample_list_.find(new_node) != this->sample_list_.end())
            continue;

        // regular the sample node
        this->_findNearestPoint(new_node);
        if (new_node.id == -1)
            continue;
        else {
            this->sample_list_.insert(new_node);
            expand.push_back(new_node);
        }
          
        // goal found
        if (_checkGoal(new_node))
            return {true, this->_convertClosedListToPath(this->sample_list_, start, goal)};

            iteration++;
      }
      return {false, {}};
    }


    /**
     * @brief Find the nearest Node that has been seen by the algorithm. This does
     * not consider cost to reach the node.
     * @param new_node Node to which the nearest node must be found
     * @return nearest node
     */
    void RRTStar::_findNearestPoint(Node& new_node) {
        Node nearest_node;
        double min_dist = std::numeric_limits<double>::max();

        for (const auto node : this->sample_list_) {
            // calculate distance
            double new_dist = std::sqrt(std::pow(node.x - new_node.x, 2) + std::pow(node.y - new_node.y, 2));

            // update nearest node
            if (new_dist < min_dist) {
                nearest_node = node;
                new_node.pid = nearest_node.id;
                new_node.cost = new_dist + node.cost;
                min_dist = new_dist;
            }
        }

        // distance longer than the threshold
        if (min_dist > this->max_dist_) {
            // connect sample node and nearest node, then move the nearest node 
            // forward to sample node with `max_distance` as result
            double theta = atan2(new_node.y - nearest_node.y, new_node.x - nearest_node.x);
            new_node.x = nearest_node.x + (int)(this->max_dist_ * cos(theta));
            new_node.y = nearest_node.y + (int)(this->max_dist_ * sin(theta));
            new_node.id = this->grid2Index(new_node.x, new_node.y);
            new_node.cost = this->max_dist_ + nearest_node.cost;
        }

        // obstacle check
        if (!_isAnyObstacleInPath(new_node, nearest_node)) {
            // rewire optimization
            for (auto node : this->sample_list_) {
                // inside the optimization circle
                double new_dist = std::sqrt(std::pow(node.x - new_node.x, 2) + std::pow(node.y - new_node.y, 2));
                if (new_dist < this->r_) {
                    double cost = node.cost + new_dist;
                    // update new sample node's cost and parent 
                    if (new_node.cost > cost) {
                        if (!_isAnyObstacleInPath(new_node, node)) {
                            new_node.pid = node.id;
                            new_node.cost = cost;
                        }
                    } else {
                        // update nodes' cost inside the radius
                        cost = new_node.cost + new_dist;
                        if (cost < node.cost) {
                            if (!_isAnyObstacleInPath(new_node, node)) {
                                node.pid = new_node.id;
                                node.cost = cost;
                            }
                        }
                    }
                } else continue;
            }           
        } else
            new_node.id = -1;
    }
}