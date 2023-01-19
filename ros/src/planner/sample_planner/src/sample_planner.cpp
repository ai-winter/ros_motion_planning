/***********************************************************
 * 
 * @file: sample_planner.cpp
 * @breif: Contains the sample planner ROS wrapper class
 * @author: Yang Haodong
 * @update: 2022-10-26
 * @version: 1.0
 * 
 * Copyright (c) 2022， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include <pluginlib/class_list_macros.h>
#include <cmath>

#include "sample_planner.h"
#include "rrt.h"
#include "rrt_star.h"
#include "rrt_connect.h"
#include "informed_rrt.h"

PLUGINLIB_EXPORT_CLASS(sample_planner::SamplePlanner, nav_core::BaseGlobalPlanner)

namespace sample_planner {
    /**
     * @brief  Constructor(default)
     */
    SamplePlanner::SamplePlanner() :
            costmap_(NULL), initialized_(false), g_planner_(NULL){ }
    /**
     * @brief  Constructor
     * @param  name     planner name
     * @param  costmap  costmap pointer
     * @param  frame_id costmap frame ID
     */
    SamplePlanner::SamplePlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) : 
        SamplePlanner() {
        initialize(name, costmap, frame_id);
    }
    /**
     * @brief Destructor
     * @return No return value
     * @details default
     */
    SamplePlanner::~SamplePlanner() {
        if (g_planner_)
            delete g_planner_;
    }


    /**
     * @brief  Planner initialization
     * @param  name         planner name
     * @param  costmapRos   costmap ROS wrapper
     */
    void SamplePlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmapRos) {
        initialize(name, costmapRos->getCostmap(), costmapRos->getGlobalFrameID());
    }
    /**
     * @brief  Planner initialization
     * @param  name     planner name
     * @param  costmap  costmap pointer
     * @param  frame_id costmap frame ID
     */
    void SamplePlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) {
        if (!initialized_) {
            // initialize ROS node
            ros::NodeHandle private_nh("~/" + name);
            // initialize costmap
            this->costmap_ = costmap;
            // costmap frame ID
            this->frame_id_ = frame_id;
            // costmap size
            unsigned int nx = costmap->getSizeInCellsX(), ny = costmap->getSizeInCellsY();
            // costmap resolution
            double resolution = costmap->getResolution();

            /*======================= static parameters loading ==========================*/
            // offset of transform from world(x,y) to grid map(x,y)
            private_nh.param("convert_offset", this->convert_offset_, 0.0);
            // error tolerance
            private_nh.param("default_tolerance", this->tolerance_, 0.0);
            // whether outline the map or not
            private_nh.param("outline_map", this->is_outline_, false);
            // obstacle inflation factor
            private_nh.param("obstacle_factor", this->factor_, 0.5);
            // whether publish expand zone or not
            private_nh.param("expand_zone", this->is_expand_, false);
            // random sample points
            private_nh.param("sample_points", this->sample_points_, 500);
            // max distance between sample points
            private_nh.param("sample_max_d", this->sample_max_d_, 5.0);
            // optimization radius
            private_nh.param("optimization_r", this->opt_r_, 10.0);

            // planner name
            std::string planner_name; 
            private_nh.param("planner_name", planner_name, (std::string)"rrt");
            if (planner_name == "rrt")
                this->g_planner_ = new rrt_planner::RRT(nx, ny, resolution, this->sample_points_, this->sample_max_d_);
            else if (planner_name == "rrt_star")
                this->g_planner_ = new rrt_planner::RRTStar(nx, ny, resolution, this->sample_points_, this->sample_max_d_, this->opt_r_);
            else if (planner_name == "rrt_connect")
                this->g_planner_ = new rrt_planner::RRTConnect(nx, ny, resolution, this->sample_points_, this->sample_max_d_);
            else if (planner_name == "informed_rrt")
                this->g_planner_ = new rrt_planner::InformedRRT(nx, ny, resolution, this->sample_points_, this->sample_max_d_, this->opt_r_);

            ROS_INFO("Using global sample planner: %s", planner_name.c_str());

            /*====================== register topics and services =======================*/
            // register planning publisher
            this->plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
            // register explorer visualization publisher
            this->expand_pub_ = private_nh.advertise<visualization_msgs::Marker>("tree", 1);
            // register planning service
            this->make_plan_srv_ = private_nh.advertiseService("make_plan", &SamplePlanner::makePlanService, this);
  
            // set initialization flag
            this->initialized_ = true;
        } else
            ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
    }
    /**
     * @brief plan a path given start and goal in world map
     * @param start     start in world map
     * @param goal      goal in world map
     * @param plan      plan
     * @param tolerance error tolerance
     * @return true if find a path successfully else false
     */
    bool SamplePlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan){
        return makePlan(start, goal, this->tolerance_, plan);
    }
    bool SamplePlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance,
                std::vector<geometry_msgs::PoseStamped>& plan) {
                    // start thread mutex
        boost::mutex::scoped_lock lock(this->mutex_);
        if (!this->initialized_) {
            ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return false;
        }
        // clear existing plan
        plan.clear();
        // get costmap size
        int nx = this->costmap_->getSizeInCellsX(), ny = this->costmap_->getSizeInCellsY();

        // judege whether goal and start node in costmap frame or not
        if (goal.header.frame_id != this->frame_id_) {
            ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
                    this->frame_id_.c_str(), goal.header.frame_id.c_str());
            return false;
        }
        if (start.header.frame_id != this->frame_id_) {
            ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
                    this->frame_id_.c_str(), start.header.frame_id.c_str());
            return false;
        }
        // get goal and strat node coordinate
        // tranform from world to costmap
        double wx = start.pose.position.x, wy = start.pose.position.y;
        double m_start_x, m_start_y, m_goal_x, m_goal_y;
        if (!this->_worldToMap(wx, wy, m_start_x, m_start_y)) {
            ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
            return false;
        }
        wx = goal.pose.position.x, wy = goal.pose.position.y;
        if (!this->_worldToMap(wx, wy, m_goal_x, m_goal_y)) {
            ROS_WARN_THROTTLE(1.0, "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
            return false;
        }
        // tranform from costmap to grid map
        int g_start_x, g_start_y, g_goal_x, g_goal_y;
        this->g_planner_->map2Grid(m_start_x, m_start_y, g_start_x, g_start_y);
        this->g_planner_->map2Grid(m_goal_x, m_goal_y, g_goal_x, g_goal_y);
        Node n_start(g_start_x, g_start_y, 0, 0, this->g_planner_->grid2Index(g_start_x, g_start_y), 0);
        Node n_goal(g_goal_x, g_goal_y, 0, 0, this->g_planner_->grid2Index(g_goal_x, g_goal_y), 0);
        
        // clear the cost of robot location
        this->costmap_->setCost(g_start_x, g_start_y, costmap_2d::FREE_SPACE);

        // outline the map
        if(this->is_outline_)
            this->_outlineMap(this->costmap_->getCharMap(), nx, ny);

        // calculate path
        std::vector<Node> expand;
        const auto [path_found, path] = this->g_planner_->plan(this->costmap_->getCharMap(), n_start, n_goal, expand);

        if (path_found) {
            if (this->_getPlanFromPath(path, plan)) {
                geometry_msgs::PoseStamped goalCopy = goal;
                goalCopy.header.stamp = ros::Time::now();
                plan.push_back(goalCopy);
            } 
            else  ROS_ERROR("Failed to get a plan from path when a legal path was found. This shouldn't happen.");
        }
        else  ROS_ERROR("Failed to get a path.");
        // publish expand zone
        if(this->is_expand_)
            this->_publishExpand(expand);

        // publish visulization plan
        this->publishPlan(plan);
        return !plan.empty();
    }
    /**
     * @brief  publish planning path
     * @param  path planning path
     */
    void SamplePlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
        if (!this->initialized_) {
            ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return;
        }
        // creat visulized path plan
        nav_msgs::Path gui_plan;
        gui_plan.poses.resize(plan.size());
        gui_plan.header.frame_id = this->frame_id_;
        gui_plan.header.stamp = ros::Time::now();
        for (unsigned int i = 0; i < plan.size(); i++) gui_plan.poses[i] = plan[i];
        // publish plan to rviz
        this->plan_pub_.publish(gui_plan);
    }
    /**
     * @brief  regeister planning service
     * @param  req  request from client
     * @param  resp response from server
     */
    bool SamplePlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
        makePlan(req.start, req.goal, resp.plan.poses);
        resp.plan.header.stamp = ros::Time::now();
        resp.plan.header.frame_id = this->frame_id_;
        return true;
    }


    /**
     * @brief  Inflate the boundary of costmap into obstacles to prevent cross planning
     * @param  costarr  costmap pointer
     * @param  nx       pixel number in costmap x direction
     * @param  ny       pixel number in costmap y direction
     */
    void SamplePlanner::_outlineMap(unsigned char* costarr, int nx, int ny) {
        unsigned char* pc = costarr;
        for (int i = 0; i < nx; i++)            *pc++ = costmap_2d::LETHAL_OBSTACLE;
        pc = costarr + (ny - 1) * nx;
        for (int i = 0; i < nx; i++)            *pc++ = costmap_2d::LETHAL_OBSTACLE;
        pc = costarr;
        for (int i = 0; i < ny; i++, pc += nx)  *pc = costmap_2d::LETHAL_OBSTACLE;
        pc = costarr + nx - 1;
        for (int i = 0; i < ny; i++, pc += nx)  *pc = costmap_2d::LETHAL_OBSTACLE;
    }
    /**
     * @brief  publish expand zone
     * @param  expand  set of expand nodes
     */
    void SamplePlanner::_publishExpand(std::vector<Node> &expand){
        ROS_DEBUG("Expand Zone Size:%ld", expand.size());

        // Initializes a Marker msg for a LINE_LIST
        visualization_msgs::Marker tree_msg;
        tree_msg.header.frame_id = "map";
        tree_msg.id = 0;
        tree_msg.ns = "tree";
        tree_msg.type = visualization_msgs::Marker::LINE_LIST;
        tree_msg.action = visualization_msgs::Marker::ADD;
        tree_msg.pose.orientation.w = 1.0;
        tree_msg.scale.x = 0.05;

        // Publish all edges
        for (auto node : expand)
            if (node.pid != 0)
                this->_pubLine(&tree_msg, &this->expand_pub_, node.id, node.pid);
    }
    /**
     * @brief  tranform from costmap(x, y) to world map(x, y)
     * @param  mx costmap x
     * @param  my costmap y
     * @param  wx world map x
     * @param  wy world map y
     */
    void SamplePlanner::_mapToWorld(double mx, double my, double& wx, double& wy) {
        wx = this->costmap_->getOriginX() + (mx + this->convert_offset_) * this->costmap_->getResolution();
        wy = this->costmap_->getOriginY() + (my + this->convert_offset_) * this->costmap_->getResolution();
    }
    /**
     * @brief  tranform from world map(x, y) to costmap(x, y)
     * @param  mx costmap x
     * @param  my costmap y
     * @param  wx world map x
     * @param  wy world map y
     */
    bool SamplePlanner::_worldToMap(double wx, double wy, double& mx, double& my) {
        double originX = this->costmap_->getOriginX(), originY = this->costmap_->getOriginY();
        double resolution = this->costmap_->getResolution();
        if (wx < originX || wy < originY)
            return false;
        mx = (wx - originX) / resolution - this->convert_offset_;
        my = (wy - originY) / resolution - this->convert_offset_;
        if (mx < this->costmap_->getSizeInCellsX() && my < this->costmap_->getSizeInCellsY())
            return true;

        return false;
    }
    /**
     * @brief  calculate plan from planning path
     * @param  path path generated by global planner
     * @param  plan plan transfromed from path
     * @return bool true if successful else false
     */
    bool SamplePlanner::_getPlanFromPath(std::vector<Node> path, std::vector<geometry_msgs::PoseStamped>& plan) {
        if (!this->initialized_) {
            ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return false;
        }
        std::string globalFrame = this->frame_id_;
        ros::Time planTime = ros::Time::now();
        plan.clear();

        for (int i = path.size() -1; i>=0; i--) {
            double wx, wy;
            this->_mapToWorld((double)path[i].x, (double)path[i].y, wx, wy);
            // coding as message type
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = this->frame_id_;
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            plan.push_back(pose);
        }
        return !plan.empty();
    }

    /**
     *  @brief Publishes a Marker msg with two points in Rviz
     *  @param line_msg Pointer to existing marker object.
     *  @param line_pub Pointer to existing marker Publisher.
     *  @param id first marker id
     *  @param pid second marker id
     */
    void SamplePlanner::_pubLine(visualization_msgs::Marker* line_msg, ros::Publisher* line_pub,
                                 int id, int pid) {
        // Update line_msg header
        line_msg->header.stamp = ros::Time::now();

        // Build msg
        geometry_msgs::Point p1, p2;
        std_msgs::ColorRGBA c1, c2;
        int p1x, p1y, p2x, p2y;

        this->g_planner_->index2Grid(id, p1x, p1y);
        this->g_planner_->grid2Map(p1x, p1y, p1.x, p1.y);
        p1.x = (p1.x + this->convert_offset_) + costmap_->getOriginX();
        p1.y = (p1.y + this->convert_offset_) + costmap_->getOriginY();
        p1.z = 1.0;

        this->g_planner_->index2Grid(pid, p2x, p2y);
        this->g_planner_->grid2Map(p2x, p2y, p2.x, p2.y);
        p2.x = (p2.x + this->convert_offset_) + costmap_->getOriginX();
        p2.y = (p2.y + this->convert_offset_) + costmap_->getOriginY();
        p2.z = 1.0;

        c1.r = 0.43;
        c1.g = 0.54;
        c1.b = 0.24;
        c1.a = 0.5;

        c2.r = 0.43;
        c2.g = 0.54;
        c2.b = 0.24;
        c2.a = 0.5;

        line_msg->points.push_back(p1);
        line_msg->points.push_back(p2);
        line_msg->colors.push_back(c1);
        line_msg->colors.push_back(c2);

        // Publish line_msg
        line_pub->publish(*line_msg);
    }
}