/***********************************************************
 * 
 * @file: graph_planner.cpp
 * @breif: Contains the graph planner ROS wrapper class
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

#include "graph_planner.h"
#include "a_star.h"
#include "jump_point_search.h"

PLUGINLIB_EXPORT_CLASS(graph_planner::GraphPlanner, nav_core::BaseGlobalPlanner)

namespace graph_planner {
    /**
     * @brief  Constructor(default)
     */
    GraphPlanner::GraphPlanner() :
            costmap_(NULL), initialized_(false), g_planner_(NULL){ }
    /**
     * @brief  Constructor
     * @param  name     planner name
     * @param  costmap  costmap pointer
     * @param  frame_id costmap frame ID
     */
    GraphPlanner::GraphPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) : 
        GraphPlanner() {
        initialize(name, costmap, frame_id);
    }
    /**
     * @brief Destructor
     * @return No return value
     * @details default
     */
    GraphPlanner::~GraphPlanner() {
        if (g_planner_)
            delete g_planner_;
    }


    /**
     * @brief  Planner initialization
     * @param  name         planner name
     * @param  costmapRos   costmap ROS wrapper
     */
    void GraphPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmapRos) {
        initialize(name, costmapRos->getCostmap(), costmapRos->getGlobalFrameID());
    }
    /**
     * @brief  Planner initialization
     * @param  name     planner name
     * @param  costmap  costmap pointer
     * @param  frame_id costmap frame ID
     */
    void GraphPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) {
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
            // obstacle factor, 
            private_nh.param("obstacle_factor", this->factor_, 0.5);
            // whether publish expand zone or not
            private_nh.param("expand_zone", this->is_expand_, false);

            // planner name
            std::string planner_name; 
            private_nh.param("planner_name", planner_name, (std::string)"a_star");
            if (planner_name == "a_star")
                this->g_planner_ = new a_star_planner::AStar(nx, ny, resolution);
            else if (planner_name == "dijkstra")
                this->g_planner_ = new a_star_planner::AStar(nx, ny, resolution, true);
            else if (planner_name == "gbfs")
                this->g_planner_ = new a_star_planner::AStar(nx, ny, resolution, false, true); 
            else if (planner_name == "jps")
                this->g_planner_ = new jps_planner::JumpPointSearch(nx, ny, resolution);

            ROS_INFO("Using global graph planner: %s", planner_name.c_str());

            /*====================== register topics and services =======================*/
            // register planning publisher
            this->plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
            // register explorer visualization publisher
            this->expand_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("expand", 1);
            // register planning service
            this->make_plan_srv_ = private_nh.advertiseService("make_plan", &GraphPlanner::makePlanService, this);
  
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
    bool GraphPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan){
        return makePlan(start, goal, this->tolerance_, plan);
    }
    bool GraphPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance,
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
                // 确保终点与发布的规划有相同时间戳
                geometry_msgs::PoseStamped goalCopy = goal;
                goalCopy.header.stamp = ros::Time::now();
                plan.push_back(goalCopy);
            } 
            else  ROS_ERROR("Failed to get a plan from path when a legal path was found. This shouldn't happen.");
        }
        else  ROS_ERROR("Failed to get a path.");
        // publish expand zone
        if(this->is_expand_)   this->_publishExpand(expand);

        // publish visulization plan
        this->publishPlan(plan);
        return !plan.empty();
    }
    /**
     * @brief  publish planning path
     * @param  path planning path
     */
    void GraphPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
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
    bool GraphPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
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
    void GraphPlanner::_outlineMap(unsigned char* costarr, int nx, int ny) {
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
    void GraphPlanner::_publishExpand(std::vector<Node> &expand){
        ROS_DEBUG("Expand Zone Size:%ld", expand.size());
        // 获得代价地图尺寸与分辨率
        int nx = this->costmap_->getSizeInCellsX(), ny = this->costmap_->getSizeInCellsY();
        double resolution = this->costmap_->getResolution();
        nav_msgs::OccupancyGrid grid;
        // 构造探索域
        grid.header.frame_id = this->frame_id_;
        grid.header.stamp = ros::Time::now();
        grid.info.resolution = resolution;
        grid.info.width = nx;
        grid.info.height = ny;
        double wx, wy;
        this->costmap_->mapToWorld(0, 0, wx, wy);
        grid.info.origin.position.x = wx - resolution / 2;
        grid.info.origin.position.y = wy - resolution / 2;
        grid.info.origin.position.z = 0.0;
        grid.info.origin.orientation.w = 1.0;
        grid.data.resize(nx * ny);
        for (unsigned int i = 0; i < grid.data.size(); i++)  grid.data[i] = 0;    
        for (unsigned int i = 0; i < expand.size(); i++)     grid.data[expand[i].id] = 50;
        this->expand_pub_.publish(grid);     
    }
    /**
     * @brief  tranform from costmap(x, y) to world map(x, y)
     * @param  mx costmap x
     * @param  my costmap y
     * @param  wx world map x
     * @param  wy world map y
     */
    void GraphPlanner::_mapToWorld(double mx, double my, double& wx, double& wy) {
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
    bool GraphPlanner::_worldToMap(double wx, double wy, double& mx, double& my) {
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
    bool GraphPlanner::_getPlanFromPath(std::vector<Node> path, std::vector<geometry_msgs::PoseStamped>& plan) {
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
}