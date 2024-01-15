/**
 * *********************************************************
 *
 * @file: graph_planner.cpp
 * @brief: Contains the graph planner ROS wrapper class
 * @author: Yang Haodong
 * @date: 2023-10-26
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong. 
 * All rights reserved.
 * 
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>

#include "graph_planner.h"
#include "a_star.h"
#include "jump_point_search.h"
#include "d_star.h"
#include "lpa_star.h"
#include "d_star_lite.h"
#include "voronoi.h"
#include "theta_star.h"
#include "lazy_theta_star.h"
#include "hybrid_a_star.h"

PLUGINLIB_EXPORT_CLASS(graph_planner::GraphPlanner, nav_core::BaseGlobalPlanner)

namespace graph_planner
{
/**
 * @brief Construct a new Graph Planner object
 */
GraphPlanner::GraphPlanner() : initialized_(false), costmap_(nullptr), g_planner_(nullptr)
{
}

/**
 * @brief Construct a new Graph Planner object
 * @param name        planner name
 * @param costmap_ros the cost map to use for assigning costs to trajectories
 */
GraphPlanner::GraphPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : GraphPlanner()
{
  initialize(name, costmap_ros);
}

/**
 * @brief Destroy the Graph Planner object
 */
GraphPlanner::~GraphPlanner()
{
  if (g_planner_)
  {
    delete g_planner_;
    g_planner_ = nullptr;
  }
}

/**
 * @brief Planner initialization
 * @param name       planner name
 * @param costmapRos costmap ROS wrapper
 */
void GraphPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmapRos)
{
  costmap_ros_ = costmapRos;
  initialize(name);
}

/**
 * @brief Planner initialization
 * @param name     planner name
 * @param costmap  costmap pointer
 * @param frame_id costmap frame ID
 */
void GraphPlanner::initialize(std::string name)
{
  if (!initialized_)
  {
    initialized_ = true;

    // initialize ROS node
    ros::NodeHandle private_nh("~/" + name);

    // initialize costmap
    costmap_ = costmap_ros_->getCostmap();

    // costmap frame ID
    frame_id_ = costmap_ros_->getGlobalFrameID();

    // get costmap properties
    nx_ = costmap_->getSizeInCellsX(), ny_ = costmap_->getSizeInCellsY();
    resolution_ = costmap_->getResolution();

    private_nh.param("convert_offset", convert_offset_, 0.0);  // offset of transform from world(x,y) to grid map(x,y)
    private_nh.param("default_tolerance", tolerance_, 0.0);    // error tolerance
    private_nh.param("outline_map", is_outline_, false);       // whether outline the map or not
    private_nh.param("obstacle_factor", factor_, 0.5);         // obstacle factor, NOTE: no use...
    private_nh.param("expand_zone", is_expand_, false);        // whether publish expand zone or not
    private_nh.param("voronoi_map", is_voronoi_map_, false);   // whether to store Voronoi map or not

    // planner name
    private_nh.param("planner_name", planner_name_, (std::string) "a_star");
    if (planner_name_ == "a_star")
      g_planner_ = new global_planner::AStar(nx_, ny_, resolution_);
    else if (planner_name_ == "dijkstra")
      g_planner_ = new global_planner::AStar(nx_, ny_, resolution_, true);
    else if (planner_name_ == "gbfs")
      g_planner_ = new global_planner::AStar(nx_, ny_, resolution_, false, true);
    else if (planner_name_ == "jps")
      g_planner_ = new global_planner::JumpPointSearch(nx_, ny_, resolution_);
    else if (planner_name_ == "d_star")
      g_planner_ = new global_planner::DStar(nx_, ny_, resolution_);
    else if (planner_name_ == "lpa_star")
      g_planner_ = new global_planner::LPAStar(nx_, ny_, resolution_);
    else if (planner_name_ == "d_star_lite")
      g_planner_ = new global_planner::DStarLite(nx_, ny_, resolution_);
    else if (planner_name_ == "voronoi")
      g_planner_ = new global_planner::VoronoiPlanner(nx_, ny_, resolution_,
                                                      costmap_ros_->getLayeredCostmap()->getCircumscribedRadius());
    else if (planner_name_ == "theta_star")
      g_planner_ = new global_planner::ThetaStar(nx_, ny_, resolution_);
    else if (planner_name_ == "lazy_theta_star")
      g_planner_ = new global_planner::LazyThetaStar(nx_, ny_, resolution_);
    else if (planner_name_ == "hybrid_a_star")
    {
      bool is_reverse;  // whether reverse operation is allowed
      double max_curv;  // maximum curvature of model
      private_nh.param("is_reverse", is_reverse, false);
      private_nh.param("max_curv", max_curv, 1.0);
      g_planner_ = new global_planner::HybridAStar(nx_, ny_, resolution_, is_reverse, max_curv);
    }
    else
      ROS_ERROR("Unknown planner name: %s", planner_name_.c_str());

    // pass costmap information to planner (required)
    g_planner_->setOrigin(costmap_->getOriginX(), costmap_->getOriginY());
    g_planner_->setConvertOffset(convert_offset_);

    ROS_INFO("Using global graph planner: %s", planner_name_.c_str());

    // register planning publisher
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

    // register explorer visualization publisher
    expand_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("expand", 1);

    // register planning service
    make_plan_srv_ = private_nh.advertiseService("make_plan", &GraphPlanner::makePlanService, this);
  }
  else
  {
    ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
  }
}

/**
 * @brief plan a path given start and goal in world map
 * @param start start in world map
 * @param goal  goal in world map
 * @param plan  plan
 * @return true if find a path successfully, else false
 */
bool GraphPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                            std::vector<geometry_msgs::PoseStamped>& plan)
{
  return makePlan(start, goal, tolerance_, plan);
}

/**
 * @brief Plan a path given start and goal in world map
 * @param start     start in world map
 * @param goal      goal in world map
 * @param plan      plan
 * @param tolerance error tolerance
 * @return true if find a path successfully, else false
 */
bool GraphPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                            double tolerance, std::vector<geometry_msgs::PoseStamped>& plan)
{
  // start thread mutex
  boost::mutex::scoped_lock lock(mutex_);
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return false;
  }
  // clear existing plan
  plan.clear();

  // judege whether goal and start node in costmap frame or not
  if (goal.header.frame_id != frame_id_)
  {
    ROS_ERROR("The goal pose passed to this planner must be in the %s frame. It is instead in the %s frame.",
              frame_id_.c_str(), goal.header.frame_id.c_str());
    return false;
  }

  if (start.header.frame_id != frame_id_)
  {
    ROS_ERROR("The start pose passed to this planner must be in the %s frame. It is instead in the %s frame.",
              frame_id_.c_str(), start.header.frame_id.c_str());
    return false;
  }

  // get goal and start node coordinate tranform from world to costmap
  double wx = start.pose.position.x, wy = start.pose.position.y;
  double m_start_x, m_start_y, m_goal_x, m_goal_y;
  if (!g_planner_->world2Map(wx, wy, m_start_x, m_start_y))
  {
    ROS_WARN(
        "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has "
        "been properly localized?");
    return false;
  }
  wx = goal.pose.position.x, wy = goal.pose.position.y;
  if (!g_planner_->world2Map(wx, wy, m_goal_x, m_goal_y))
  {
    ROS_WARN_THROTTLE(1.0,
                      "The goal sent to the global planner is off the global costmap. Planning will always fail to "
                      "this goal.");
    return false;
  }

  // tranform from costmap to grid map
  int g_start_x, g_start_y, g_goal_x, g_goal_y;
  g_planner_->map2Grid(m_start_x, m_start_y, g_start_x, g_start_y);
  g_planner_->map2Grid(m_goal_x, m_goal_y, g_goal_x, g_goal_y);

  // NOTE: how to init start and goal?
  Node start_node(g_start_x, g_start_y, 0, 0, g_planner_->grid2Index(g_start_x, g_start_y), 0);
  Node goal_node(g_goal_x, g_goal_y, 0, 0, g_planner_->grid2Index(g_goal_x, g_goal_y), 0);

  // outline the map
  if (is_outline_)
    g_planner_->outlineMap(costmap_->getCharMap());

  // calculate voronoi map
  bool voronoi_layer_exist = false;
  if (is_voronoi_map_)
  {
    for (auto layer = costmap_ros_->getLayeredCostmap()->getPlugins()->begin();
         layer != costmap_ros_->getLayeredCostmap()->getPlugins()->end(); ++layer)
    {
      boost::shared_ptr<costmap_2d::VoronoiLayer> voronoi_layer =
          boost::dynamic_pointer_cast<costmap_2d::VoronoiLayer>(*layer);
      if (voronoi_layer)
      {
        voronoi_layer_exist = true;
        boost::unique_lock<boost::mutex> lock(voronoi_layer->getMutex());
        voronoi_ = voronoi_layer->getVoronoi();
        break;
      }
    }
    if (!voronoi_layer_exist)
      ROS_WARN("Failed to get a Voronoi layer for potentional application.");
  }

  // calculate path
  std::vector<Node> path;
  std::vector<Node> expand;
  bool path_found = false;

  // planning
  if (planner_name_ == "voronoi")
  {
    if (!voronoi_layer_exist)
      ROS_ERROR("Failed to get a Voronoi layer for Voronoi planner.");
    path_found = dynamic_cast<global_planner::VoronoiPlanner*>(g_planner_)->plan(voronoi_, start_node, goal_node, path);
  }
  else if (planner_name_ == "hybrid_a_star")
  {
    // using world frame
    global_planner::HybridAStar::HybridNode h_start(start.pose.position.x, start.pose.position.y,
                                                    tf2::getYaw(start.pose.orientation));
    global_planner::HybridAStar::HybridNode h_goal(goal.pose.position.x, goal.pose.position.y,
                                                   tf2::getYaw(goal.pose.orientation));
    path_found = dynamic_cast<global_planner::HybridAStar*>(g_planner_)
                     ->plan(costmap_->getCharMap(), h_start, h_goal, path, expand);
  }
  else
    path_found = g_planner_->plan(costmap_->getCharMap(), start_node, goal_node, path, expand);

  // convert path to ros plan
  if (path_found)
  {
    if (_getPlanFromPath(path, plan))
    {
      geometry_msgs::PoseStamped goalCopy = goal;
      goalCopy.header.stamp = ros::Time::now();
      plan.push_back(goalCopy);
    }
    else
      ROS_ERROR("Failed to get a plan from path when a legal path was found. This shouldn't happen.");
  }
  else
    ROS_ERROR("Failed to get a path.");

  // publish expand zone
  if (is_expand_)
    _publishExpand(expand);

  // publish visulization plan
  publishPlan(plan);

  return !plan.empty();
}

/**
 * @brief publish planning path
 * @param path planning path
 */
void GraphPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return;
  }

  // create visulized path plan
  nav_msgs::Path gui_plan;
  gui_plan.poses.resize(plan.size());
  gui_plan.header.frame_id = frame_id_;
  gui_plan.header.stamp = ros::Time::now();
  for (unsigned int i = 0; i < plan.size(); i++)
    gui_plan.poses[i] = plan[i];

  // publish plan to rviz
  plan_pub_.publish(gui_plan);
}

/**
 * @brief Regeister planning service
 * @param req  request from client
 * @param resp response from server
 * @return true
 */
bool GraphPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp)
{
  makePlan(req.start, req.goal, resp.plan.poses);
  resp.plan.header.stamp = ros::Time::now();
  resp.plan.header.frame_id = frame_id_;

  return true;
}

/**
 * @brief publish expand zone
 * @param expand set of expand nodes
 */
void GraphPlanner::_publishExpand(std::vector<Node>& expand)
{
  ROS_DEBUG("Expand Zone Size: %ld", expand.size());

  nav_msgs::OccupancyGrid grid;

  // build expand
  grid.header.frame_id = frame_id_;
  grid.header.stamp = ros::Time::now();
  grid.info.resolution = resolution_;
  grid.info.width = nx_;
  grid.info.height = ny_;

  double wx, wy;
  costmap_->mapToWorld(0, 0, wx, wy);
  grid.info.origin.position.x = wx - resolution_ / 2;
  grid.info.origin.position.y = wy - resolution_ / 2;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;
  grid.data.resize(nx_ * ny_);

  for (unsigned int i = 0; i < grid.data.size(); i++)
    grid.data[i] = 0;
  for (unsigned int i = 0; i < expand.size(); i++)
    grid.data[expand[i].id_] = 50;

  expand_pub_.publish(grid);
}

/**
 * @brief Calculate plan from planning path
 * @param path path generated by global planner
 * @param plan plan transfromed from path, i.e. [start, ..., goal]
 * @return bool true if successful, else false
 */
bool GraphPlanner::_getPlanFromPath(std::vector<Node>& path, std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return false;
  }
  ros::Time planTime = ros::Time::now();
  plan.clear();

  for (int i = path.size() - 1; i >= 0; i--)
  {
    double wx, wy;
    g_planner_->map2World((double)path[i].x_, (double)path[i].y_, wx, wy);

    // coding as message type
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = frame_id_;
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
}  // namespace graph_planner