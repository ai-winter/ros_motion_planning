/**
 * *********************************************************
 *
 * @file: evolutionary_planner.h
 * @brief: Contains the evolutionary planner ROS wrapper class
 * @author: Yang Haodong
 * @date: 2023-7-16
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "evolutionary_planner.h"
#include <pluginlib/class_list_macros.h>

#include "aco.h"
#include "pso.h"
#include "ga.h"

PLUGINLIB_EXPORT_CLASS(evolutionary_planner::EvolutionaryPlanner, nav_core::BaseGlobalPlanner)

namespace evolutionary_planner
{
/**
 * @brief Construct a new Graph Planner object
 */
EvolutionaryPlanner::EvolutionaryPlanner() : initialized_(false), costmap_(nullptr), g_planner_(nullptr)
{
}

/**
 * @brief Construct a new Graph Planner object
 * @param name      planner name
 * @param costmap_ros   the cost map to use for assigning costs to trajectories
 */
EvolutionaryPlanner::EvolutionaryPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : EvolutionaryPlanner()
{
  initialize(name, costmap_ros);
}

/**
 * @brief Destroy the Graph Planner object
 */
EvolutionaryPlanner::~EvolutionaryPlanner()
{
  if (g_planner_)
  {
    delete g_planner_;
    g_planner_ = NULL;
  }
}

/**
 * @brief  Planner initialization
 * @param  name         planner name
 * @param  costmapRos   costmap ROS wrapper
 */
void EvolutionaryPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmapRos)
{
  initialize(name, costmapRos->getCostmap(), costmapRos->getGlobalFrameID());
}

/**
 * @brief Planner initialization
 * @param name      planner name
 * @param costmap   costmap pointer
 * @param frame_id  costmap frame ID
 */
void EvolutionaryPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id)
{
  if (!initialized_)
  {
    initialized_ = true;

    // initialize ROS node
    ros::NodeHandle private_nh("~/" + name);

    // initialize costmap
    costmap_ = costmap;

    // costmap frame ID
    frame_id_ = frame_id;

    // get costmap properties
    nx_ = costmap->getSizeInCellsX(), ny_ = costmap->getSizeInCellsY();
    origin_x_ = costmap_->getOriginX(), origin_y_ = costmap_->getOriginY();
    resolution_ = costmap->getResolution();

    private_nh.param("convert_offset", convert_offset_, 0.0);  // offset of transform from world(x,y) to grid map(x,y)
    private_nh.param("default_tolerance", tolerance_, 0.0);    // error tolerance
    private_nh.param("outline_map", is_outline_, false);       // whether outline the map or not
    private_nh.param("obstacle_factor", factor_, 0.5);         // obstacle factor, NOTE: no use...
    private_nh.param("expand_zone", is_expand_, false);        // whether publish expand zone or not

    // planner name
    std::string planner_name;
    private_nh.param("planner_name", planner_name, (std::string) "aco");
    if (planner_name == "aco")
    {
      int n_ants, n_inherited, point_num, max_iter, init_mode;
      double alpha, beta, rho, Q;
      private_nh.param("n_ants", n_ants, 50);                         // number of ants
      private_nh.param("ant_inherited", n_inherited, 10);             // number of inherited ants
      private_nh.param("point_num_ant", point_num, 5);                // number of position points contained in each ant
      private_nh.param("alpha", alpha, 1.0);                          // pheromone weight coefficient
      private_nh.param("beta", beta, 5.0);                            // heuristic factor weight coefficient
      private_nh.param("rho", rho, 0.1);                              // evaporation coefficient
      private_nh.param("Q", Q, 1.0);                                  // pheromone gain
      private_nh.param("init_mode_ant", init_mode, GEN_MODE_CIRCLE);  // Set the generation mode for the initial
                                                                      // position points of the ants
      private_nh.param("max_iter_ant", max_iter, 100);                // maximum iterations

      g_planner_ = new global_planner::ACO(nx_, ny_, resolution_, n_ants, n_inherited, point_num, alpha, beta, rho, Q,
                                           init_mode, max_iter);
    }
    else if (planner_name == "pso")
    {
      bool pub_particles;
      int n_particles, n_inherited, point_num, max_speed, init_mode, max_iter;
      double w_inertial, w_social, w_cognitive;

      private_nh.param("n_particles", n_particles, 50);    // number of particles
      private_nh.param("pso_inherited", n_inherited, 10);  // number of inherited particles
      private_nh.param("point_num_pso", point_num, 5);     // number of position points contained in each particle
      private_nh.param("max_speed_pso", max_speed, 40);    // The maximum velocity of particle motion
      private_nh.param("w_inertial", w_inertial, 1.0);     // inertia weight
      private_nh.param("w_social", w_social, 2.0);         // social weight
      private_nh.param("w_cognitive", w_cognitive, 1.2);   // cognitive weight
      private_nh.param("init_mode_pso", init_mode, GEN_MODE_CIRCLE);  // Set the generation mode for the initial
                                                                      // position points of the particle swarm
      private_nh.param("max_iter_pso", max_iter, 30);                 // maximum iterations

      g_planner_ = new global_planner::PSO(nx_, ny_, resolution_, n_particles, n_inherited, point_num, w_inertial,
                                           w_social, w_cognitive, max_speed, init_mode, max_iter);
    }
    else if (planner_name == "ga")
    {
      bool pub_genets;
      int n_genets, ga_inherited, point_num, max_speed, init_mode, max_iter;
      double p_select, p_crs, p_mut;
      private_nh.param("n_genets", n_genets, 50);          // number of genets
      private_nh.param("ga_inherited", ga_inherited, 20);  // number of inherited genets
      private_nh.param("point_num_ga", point_num, 5);      // number of position points contained in each genets
      private_nh.param("max_speed_ga", max_speed, 40);     // The maximum velocity of genets motion
      private_nh.param("p_select", p_select, 0.5);         // selection probability
      private_nh.param("p_crs", p_crs, 0.8);               // crossover probability
      private_nh.param("p_mut", p_mut, 0.3);               // mutation probability
      private_nh.param("init_mode_ga", init_mode, GEN_MODE_CIRCLE);  // Set the generation mode for the initial
                                                                     // position points of the particle swarm
      private_nh.param("max_iter_ga", max_iter, 30);                 // maximum iterations

      g_planner_ = new global_planner::GA(nx_, ny_, resolution_, n_genets, ga_inherited, point_num, p_select, p_crs,
                                          p_mut, max_speed, init_mode, max_iter);
    }

    // pass costmap information to planner (required)
    g_planner_->setOrigin(origin_x_, origin_y_);
    g_planner_->setConvertOffset(convert_offset_);
    ROS_INFO("Using global evolutionary planner: %s", planner_name.c_str());

    // register planning publisher
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    // register explorer visualization publisher
    expand_pub_ = private_nh.advertise<visualization_msgs::Marker>(planner_name + "_marker", 1);

    // register planning service
    make_plan_srv_ = private_nh.advertiseService("make_plan", &EvolutionaryPlanner::makePlanService, this);
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
bool EvolutionaryPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
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
bool EvolutionaryPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
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

  // get goal and strat node coordinate tranform from world to costmap
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

  // clear the cost of robot location
  costmap_->setCost(g_start_x, g_start_y, costmap_2d::FREE_SPACE);

  // outline the map
  if (is_outline_)
    g_planner_->outlineMap(costmap_->getCharMap());

  // calculate path
  std::vector<Node> path;
  std::vector<Node> expand;
  bool path_found = g_planner_->plan(costmap_->getCharMap(), start_node, goal_node, path, expand);

  if (path_found)
  {
    if (_getPlanFromPath(path, plan))
    {
      geometry_msgs::PoseStamped goal_copy = goal;
      goal_copy.header.stamp = ros::Time::now();
      plan.push_back(goal_copy);
    }
    else
    {
      ROS_ERROR("Failed to get a plan from path when a legal path was found. This shouldn't happen.");
    }
  }
  else
  {
    ROS_ERROR("Failed to get a path.");
  }

  if (is_expand_ && expand.size())
    _publishExpand(expand);

  // publish visulization plan
  publishPlan(plan);

  return !plan.empty();
}

/**
 * @brief publish planning path
 * @param path  planning path
 */
void EvolutionaryPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
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
 * @param req   request from client
 * @param resp  response from server
 * @return true
 */
bool EvolutionaryPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp)
{
  makePlan(req.start, req.goal, resp.plan.poses);
  resp.plan.header.stamp = ros::Time::now();
  resp.plan.header.frame_id = frame_id_;

  return true;
}

/**
 * @brief Calculate plan from planning path
 * @param path  path generated by global planner
 * @param plan  plan transfromed from path, i.e. [start, ..., goal]
 * @return  bool true if successful, else false
 */
bool EvolutionaryPlanner::_getPlanFromPath(std::vector<Node>& path, std::vector<geometry_msgs::PoseStamped>& plan)
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

/**
 * @brief  publish expand zone
 * @param  expand  set of expand nodes
 */
void EvolutionaryPlanner::_publishExpand(std::vector<Node>& expand)
{
  ROS_DEBUG("Expand Zone Size:%ld", expand.size());

  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.color.r = 0.43;
  marker.color.g = 0.54;
  marker.color.b = 0.24;
  marker.color.a = 0.5;

  // Convert particle positions to geometry_msgs::Point
  for (const auto& node : expand)
  {
    geometry_msgs::Point p;
    p.x = origin_x_ + node.x_ * resolution_;
    p.y = origin_y_ + node.y_ * resolution_;
    p.z = 0.0;
    marker.points.push_back(p);
  }

  // Set the lifetime of the marker (e.g., 1 second)
  marker.lifetime = ros::Duration(1.0);

  expand_pub_.publish(marker);
}

}  // namespace evolutionary_planner