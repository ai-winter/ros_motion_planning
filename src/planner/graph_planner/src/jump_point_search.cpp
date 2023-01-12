/***********************************************************
 * 
 * @file: jump_point_search.cpp
 * @breif: Contains the Jump Point Search(JPS) planner class
 * @author: Yang Haodong
 * @update: 2022-10-27
 * @version: 1.0
 * 
 * Copyright (c) 2022， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include "jump_point_search.h"

namespace jps_planner {
    /**
     * @brief  Constructor
     * @param   nx          pixel number in costmap x direction
     * @param   ny          pixel number in costmap y direction
     * @param   resolution  costmap resolution
     */
    JumpPointSearch::JumpPointSearch(int nx, int ny, double resolution) : GlobalPlanner(nx, ny, resolution){}

    /**
     * @brief Jump Point Search(JPS) implementation
     * @param costs     costmap
     * @param start     start node
     * @param goal      goal node
     * @param expand    containing the node been search during the process
     * @return tuple contatining a bool as to whether a path was found, and the path
     */
    std::tuple<bool, std::vector<Node>> JumpPointSearch::plan(const unsigned char* costs, const Node& start,
                                                const Node& goal, std::vector<Node> &expand) {
        // copy
        this->costs_ = costs;
        // ROS_INFO("aaa_cost:%d", cos)
        this->start_ = start, this->goal_ = goal;

        // get all possible motions
        std::vector<Node> motion = getMotion();
        open_list_.push(start_);

        // main loop
        while (!open_list_.empty()) {
            // pop current node from open list
            Node current = open_list_.top();
            this->open_list_.pop();
            current.id = this->grid2Index(current.x, current.y);

            // current node do not exist in closed list
            if (this->closed_list_.find(current) != this->closed_list_.end()) continue;

            // goal found
            if (current==this->goal_) {
                this->closed_list_.insert(current);
                ROS_INFO("GOAL 1");
                return {true, this->_convertClosedListToPath(this->closed_list_, this->start_, this->goal_)};
            }

            // explore neighbor of current node
            for (const auto& m : motion) {
                Node new_point = current + m;

                // current node do not exist in closed list
                if (this->closed_list_.find(new_point) != this->closed_list_.end()) continue;

                // explore a new node
                new_point.id = this->grid2Index(new_point.x, new_point.y);
                new_point.pid = current.id;
                new_point.h_cost = std::abs(new_point.x - goal.x) + std::abs(new_point.y - goal.y);

                // goal found
                if (new_point==this->goal_) {
                    this->open_list_.push(new_point);
                    break;
                }
  
                // boundary check
                if (new_point.id < 0 || new_point.id >= this->ns_)  continue;

                // obstacle or visited
                if (costs[new_point.id] >= this->lethal_cost_ * this->factor_) continue;
  
                // ROS_INFO("before jump");
                // start jump point search
                Node jump_point = this->jump(new_point, m, current.id);
                ROS_INFO("%d", jump_point.id);
                // ROS_INFO("after jump");
                // jump point found
                if (jump_point.id != -1) {
                    this->open_list_.push(jump_point);
                    expand.push_back(jump_point);
                    // goal found
                    if (jump_point==this->goal_) {
                        this->closed_list_.insert(current);
                        this->closed_list_.insert(jump_point);
                        ROS_INFO("GOAL 2");
                        return {true, this->_convertClosedListToPath(this->closed_list_, start, goal)};
                    }
                }
                this->open_list_.push(new_point);
            }
            this->closed_list_.insert(current);
        }
        return {false, {}};
    }
    /**
     * @brief detect whether current node has forced neighbor or not 
     * @param cur_point     current node
     * @param next_point    next node
     * @param motion        current motion
     * @return true if current node has forced neighbor else false
     */
    bool JumpPointSearch::hasForcedNeighbours(const Node& cur_point, const Node& next_point,
                                              const Node& motion) {
        // current node's neighbor 
        int cn1x = cur_point.x + motion.y;
        int cn1y = cur_point.y + motion.x;
        int cn1id = this->grid2Index(cn1x, cn1y);
        int cn2x = cur_point.x - motion.y;
        int cn2y = cur_point.y - motion.x;
        int cn2id = this->grid2Index(cn2x, cn2y);

        // next node's neighbor
        int nn1x = next_point.x + motion.y;
        int nn1y = next_point.y + motion.x;
        int nn1id = this->grid2Index(nn1x, nn1y);
        int nn2x = next_point.x - motion.y;
        int nn2y = next_point.y - motion.x;
        int nn2id = this->grid2Index(nn2x, nn2y);
        // ROS_INFO("cn1:%d, cn2: %d, nn1:%d, nn2:%d", cn1id, cn2id, nn1id, nn2id);
        // ROS_INFO("cn1:%d, cn2: %d, nn1:%d, nn2:%d", this->costs_[cn1id], this->costs_[cn2id], this->costs_[nn1id], this->costs_[nn2id]);
        bool a = !(cn1id < 0 || cn1id >= this->ns_ ||
                   this->costs_[cn1id] >= this->lethal_cost_ * this->factor_);
        bool b = !(nn1id < 0 || nn1id >= this->ns_ ||
                   this->costs_[nn1id] >= this->lethal_cost_ * this->factor_);
        bool c = !(cn2id < 0 || cn2id >= this->ns_ ||
                   this->costs_[cn2id] >= this->lethal_cost_ * this->factor_);
        bool d = !(nn2id < 0 || nn2id >= this->ns_ ||
                   this->costs_[nn2id] >= this->lethal_cost_ * this->factor_);
        // ROS_INFO("a:%d, b: %d, c:%d, d:%d", a, b, c, d);
        return (a != b) || (c != d);
    }
    
    /**
     * @brief calculate jump node recursively
     * @param new_point new node
     * @param motion    current motion
     * @param id        parent id of `new_point`
     * @return jump node
     */   
    Node JumpPointSearch::jump(const Node& new_point, const Node& motion, const int id) {
        Node next_point = new_point + motion;
        next_point.id = this->grid2Index(next_point.x, next_point.y);
        next_point.pid = id;
        next_point.h_cost = std::abs(next_point.x - this->goal_.x) + std::abs(next_point.y - this->goal_.y);
// ROS_INFO("next_point id:%d, goal id: %d", next_point.id, goal_.id);

        // next node hit the boundary or obstacle
        if (next_point.id < 0 || next_point.id >= this->ns_ || 
            this->costs_[next_point.id] >= this->lethal_cost_ * this->factor_) {
            return Node(-1, -1, -1, -1, -1, -1);
        }
        

        // goal found
        if (next_point==this->goal_)  return next_point;
// ROS_INFO("before neighbor");
        // detect forced neighbor
        bool fn = this->hasForcedNeighbours(new_point, next_point, motion);
        if (fn)                       return next_point;
// ROS_INFO("after neighbor");
        // calculate the jump node
        Node jump_node = jump(next_point, motion, id);
        // ROS_INFO("!!!!!!!!!!!!!!!!!!");
        // pruned
        if (jump_node.cost != -1 && jump_node.cost + jump_node.h_cost <=
                                    next_point.cost + next_point.h_cost) {
            return jump_node;
        }

        return next_point;
}

}