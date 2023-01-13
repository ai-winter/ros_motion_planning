/***********************************************************
 * 
 * @file: a_star.cpp
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
#include <cmath>
#include <queue>
#include <unordered_set>
#include <vector>

#include "a_star.h"

namespace a_star_planner{
  /**
   * @brief  Constructor
   * @param   nx          pixel number in costmap x direction
   * @param   ny          pixel number in costmap y direction
   * @param   resolution  costmap resolution
   */
  AStar::AStar(int nx, int ny, double resolution, bool dijkstra, bool gbfs) : 
      GlobalPlanner(nx, ny, resolution) {
    // can not using both dijkstra and GBFS at the same time
    if(!(dijkstra && gbfs)) {
      this->is_dijkstra_ = dijkstra;
      this->is_gbfs_ = gbfs;
    } else {
      this->is_dijkstra_ = false;
      this->is_gbfs_ = false;   
    }
  };
  /**
   * @brief A* implementation
   * @param costs     costmap
   * @param start     start node
   * @param goal      goal node
   * @param expand    containing the node been search during the process
   * @return tuple contatining a bool as to whether a path was found, and the
   * path
   */
  std::tuple<bool, std::vector<Node>> AStar::plan(const unsigned char* costs, const Node& start,
                                                  const Node& goal, std::vector<Node> &expand) {
    // open list
    std::priority_queue<Node, std::vector<Node>, compare_cost> open_list;
    open_list.push(start);

    // closed list
    std::unordered_set<Node, NodeIdAsHash, compare_coordinates> closed_list;

    // expand list
    expand.clear();
    expand.push_back(start);

    // get all possible motions
    const std::vector<Node> motion = getMotion();

    // main loop
    while (!open_list.empty()) {
      // pop current node from open list
      Node current = open_list.top();
      open_list.pop();
      current.id = this->grid2Index(current.x, current.y);
      
      // current node do not exist in closed list
      if (closed_list.find(current) != closed_list.end()) continue;

      // goal found
      if (current==goal) {
        closed_list.insert(current);
        return {true, this->_convertClosedListToPath(closed_list, start, goal)};
      }

      // explore neighbor of current node
      for (const auto& m : motion) {
        Node new_point = current + m;

        // current node do not exist in closed list
        if (closed_list.find(new_point) != closed_list.end()) continue;

        // explore a new node
        new_point.id = this->grid2Index(new_point.x, new_point.y);
        new_point.pid = current.id;

        // if using dijkstra implementation, do not consider heuristics cost
        if(!this->is_dijkstra_)
          new_point.h_cost = std::abs(new_point.x - goal.x) + std::abs(new_point.y - goal.y);
        // if using GBFS implementation, only consider heuristics cost
        if(this->is_gbfs_)
          new_point.cost = 0;
        
        // goal found
        if (new_point==goal) {
          open_list.push(new_point);
          break;
        }

        // boundary check
        if (new_point.id < 0 || new_point.id >= this->ns_)  continue;

        // obstacle or visited
        if (costs[new_point.id] >= this->lethal_cost_ * this->factor_) continue;

        open_list.push(new_point);
        expand.push_back(new_point);
      }
      closed_list.insert(current);
    }
    return {false, {}};
  }
}
