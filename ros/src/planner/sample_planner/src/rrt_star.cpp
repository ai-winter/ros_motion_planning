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
        Node sample_node = this->_generateRandomNode();

        // obstacle
        if (costs[sample_node.id] >= this->lethal_cost_ * this->factor_)
            continue;
        
        // visited
        if (this->sample_list_.find(sample_node) != this->sample_list_.end())
            continue;

        // regular the sample node
        Node new_node = this->_findNearestPoint(this->sample_list_, sample_node);
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
     * @brief Regular the new node by the nearest node in the sample list
     * @param list     sample list
     * @param node     sample node
     * @return nearest node
     */
    Node RRTStar::_findNearestPoint(std::unordered_set<Node, NodeIdAsHash, compare_coordinates> list, Node& node) {
        Node nearest_node, new_node(node);
        double min_dist = std::numeric_limits<double>::max();

        for (const auto node_ : list) {
            // calculate distance
            double new_dist = this->_dist(node_, new_node);

            // update nearest node
            if (new_dist < min_dist) {
                nearest_node = node_;
                new_node.pid = nearest_node.id;
                new_node.cost = new_dist + node_.cost;
                min_dist = new_dist;
            }
        }

        // distance longer than the threshold
        if (min_dist > this->max_dist_) {
            // connect sample node and nearest node, then move the nearest node 
            // forward to sample node with `max_distance` as result
            double theta = this->_angle(nearest_node, new_node);
            new_node.x = nearest_node.x + (int)(this->max_dist_ * cos(theta));
            new_node.y = nearest_node.y + (int)(this->max_dist_ * sin(theta));
            new_node.id = this->grid2Index(new_node.x, new_node.y);
            new_node.cost = this->max_dist_ + nearest_node.cost;
        }

        // obstacle check
        if (!_isAnyObstacleInPath(new_node, nearest_node)) {
            // rewire optimization
            for (auto node_ : this->sample_list_) {
                // inside the optimization circle
                double new_dist = this->_dist(node_, new_node);
                if (new_dist < this->r_) {
                    double cost = node_.cost + new_dist;
                    // update new sample node's cost and parent 
                    if (new_node.cost > cost) {
                        if (!_isAnyObstacleInPath(new_node, node_)) {
                            new_node.pid = node_.id;
                            new_node.cost = cost;
                        }
                    } else {
                        // update nodes' cost inside the radius
                        cost = new_node.cost + new_dist;
                        if (cost < node_.cost) {
                            if (!_isAnyObstacleInPath(new_node, node_)) {
                                node_.pid = new_node.id;
                                node_.cost = cost;
                            }
                        }
                    }
                } else continue;
            }           
        } else
            new_node.id = -1;
        return new_node;
    }
}