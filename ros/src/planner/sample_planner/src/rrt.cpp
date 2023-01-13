/***********************************************************
 * 
 * @file: rrt.cpp
 * @breif: Contains the Rapidly-Exploring Random Tree(RRT) planner class
 * @author: Yang Haodong
 * @update: 2022-10-27
 * @version: 1.0
 * 
 * Copyright (c) 2022， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include <cmath>
#include <random>

#include "rrt.h"

namespace rrt_planner {
    /**
     * @brief  Constructor
     * @param   nx          pixel number in costmap x direction
     * @param   ny          pixel number in costmap y direction
     * @param   sample_num  andom sample points
     * @param   max_dist    max distance between sample points
     */
    RRT::RRT(int nx, int ny, double resolution, int sample_num, double max_dist)
      : GlobalPlanner(nx, ny, resolution), sample_num_(sample_num), max_dist_(max_dist) {}

    /**
     * @brief RRT implementation
     * @param costs     costmap
     * @param start     start node
     * @param goal      goal node
     * @param expand    containing the node been search during the process
     * @return tuple contatining a bool as to whether a path was found, and the path
     */
    std::tuple<bool, std::vector<Node>> RRT::plan(const unsigned char* costs, const Node& start,
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
     * @brief Generates a random node
     * @return Generated node
     */
    Node RRT::_generateRandomNode() {
      // obtain a random number from hardware
      std::random_device rd;
      // seed the generator
      std::mt19937 eng(rd());
      // define the range
      std::uniform_int_distribution<int> distr(0, this->ns_ - 1);
      // generate node
      const int id = distr(eng);
      int x, y;
      this->index2Grid(id, x, y);
      return Node(x, y, 0, 0, id, 0);
    }

    /**
     * @brief Find the nearest Node that has been seen by the algorithm. This does
     * not consider cost to reach the node.
     * @param new_node Node to which the nearest node must be found
     * @return nearest node
     */
    void RRT::_findNearestPoint(Node& new_node) {
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
      if (_isAnyObstacleInPath(new_node, nearest_node))
        new_node.id = -1;
    }

    /**
     * @brief Check if there is any obstacle between the 2 nodes.
     * @param n1        Node 1
     * @param n2        Node 2
     * @return bool value of whether obstacle exists between nodes
     */
    bool RRT::_isAnyObstacleInPath(const Node& n1, const Node& n2) {
      double theta = atan2(n2.y - n1.y, n2.x - n1.x);
      double dist = std::sqrt(std::pow(n1.x - n2.x, 2) + std::pow(n1.y - n2.y, 2));

      // distance longer than the threshold
      if (dist > this->max_dist_)
        return true;

      // sample the line between two nodes and check obstacle
      int n_step = (int)(dist / this->resolution_);
      for (int i = 0; i < n_step; i++) {
          float line_x = (float)n1.x + (float)(i * this->resolution_ * cos(theta));
          float line_y = (float)n1.y + (float)(i * this->resolution_ * sin(theta));
          if (this->costs_[this->grid2Index((int)line_x, (int)line_y)] >= this->lethal_cost_ * this->factor_)
              return true;
      }
      return false;
    }

    /**
     * @brief Check if goal is reachable from current node
     * @param new_node Current node
     * @return bool value of whether goal is reachable from current node
     */
    bool RRT::_checkGoal(const Node& new_node) {
      auto dist = std::sqrt(std::pow(this->goal_.x - new_node.x, 2) +
                            std::pow(this->goal_.y - new_node.y, 2));
      if (dist > this->max_dist_) 
        return false;

      if (!_isAnyObstacleInPath(new_node, this->goal_)) {
        Node goal(this->goal_.x, this->goal_.y, dist + new_node.cost, 0,
                  this->grid2Index(this->goal_.x, this->goal_.y), new_node.id);
        this->sample_list_.insert(goal);
        return true;
      }
      return false;
    }
}
