/**
 * *********************************************************
 *
 * @file: path_planner_node.cpp
 * @brief: Contains the path planner ROS wrapper class
 * @author: Yang Haodong
 * @date: 2024-9-24
 * @version: 2.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <tf2/utils.h>
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/MarkerArray.h>

// graph-based planner
#include "path_planner/path_planner_node.h"
#include "path_planner/graph_planner/astar_planner.h"
#include "path_planner/graph_planner/jps_planner.h"
#include "path_planner/graph_planner/dstar_planner.h"
#include "path_planner/graph_planner/lpa_star_planner.h"
#include "path_planner/graph_planner/dstar_lite_planner.h"
#include "path_planner/graph_planner/theta_star_planner.h"
#include "path_planner/graph_planner/s_theta_star_planner.h"
#include "path_planner/graph_planner/lazy_theta_star_planner.h"
#include "path_planner/graph_planner/hybrid_astar_planner.h"
#include "path_planner/graph_planner/voronoi_planner.h"
#include "path_planner/graph_planner/lazy_planner.h"

// sample-based planner
#include "path_planner/sample_planner/rrt_planner.h"
#include "path_planner/sample_planner/rrt_star_planner.h"
#include "path_planner/sample_planner/rrt_connect_planner.h"
#include "path_planner/sample_planner/informed_rrt_star_planner.h"
#include "path_planner/sample_planner/quick_informed_rrt_star_planner.h"

// evolutionary-based planner
#include "path_planner/evolutionary_planner/aco_planner.h"
#include "path_planner/evolutionary_planner/pso_planner.h"
#include "path_planner/evolutionary_planner/ga_planner.h"

#include "common/util/visualizer.h"

PLUGINLIB_EXPORT_CLASS(rmp::path_planner::PathPlannerNode, nav_core::BaseGlobalPlanner)

namespace rmp
{
namespace path_planner
{
using Visualizer = rmp::common::util::Visualizer;

/**
 * @brief Construct a new Graph Planner object
 */
PathPlannerNode::PathPlannerNode() : initialized_(false), g_planner_(nullptr)
{
}

/**
 * @brief Construct a new Graph Planner object
 * @param name        planner name
 * @param costmap_ros the cost map to use for assigning costs to trajectories
 */
PathPlannerNode::PathPlannerNode(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : PathPlannerNode()
{
  initialize(name, costmap_ros);
}

/**
 * @brief Planner initialization
 * @param name       planner name
 * @param costmapRos costmap ROS wrapper
 */
void PathPlannerNode::initialize(std::string name, costmap_2d::Costmap2DROS* costmapRos)
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
void PathPlannerNode::initialize(std::string name)
{
  if (!initialized_)
  {
    initialized_ = true;

    // initialize ROS node
    ros::NodeHandle private_nh("~/" + name);

    // costmap frame ID
    frame_id_ = costmap_ros_->getGlobalFrameID();

    private_nh.param("default_tolerance", tolerance_, 0.0);  // error tolerance
    private_nh.param("outline_map", is_outline_, false);     // whether outline the map or not
    private_nh.param("obstacle_factor", factor_, 0.5);       // obstacle factor, NOTE: no use...
    private_nh.param("expand_zone", is_expand_, false);      // whether publish expand zone or not

    // planner name
    private_nh.param("planner_name", planner_name_, (std::string) "a_star");
    if (planner_name_ == "a_star")
    {
      g_planner_ = std::make_shared<AStarPathPlanner>(costmap_ros_);
      planner_type_ = GRAPH_PLANNER;
    }
    else if (planner_name_ == "dijkstra")
    {
      g_planner_ = std::make_shared<AStarPathPlanner>(costmap_ros_, true);
      planner_type_ = GRAPH_PLANNER;
    }
    else if (planner_name_ == "gbfs")
    {
      g_planner_ = std::make_shared<AStarPathPlanner>(costmap_ros_, false, true);
      planner_type_ = GRAPH_PLANNER;
    }
    else if (planner_name_ == "jps")
    {
      g_planner_ = std::make_shared<JPSPathPlanner>(costmap_ros_);
      planner_type_ = GRAPH_PLANNER;
    }
    else if (planner_name_ == "d_star")
    {
      g_planner_ = std::make_shared<DStarPathPlanner>(costmap_ros_);
      planner_type_ = GRAPH_PLANNER;
    }
    else if (planner_name_ == "lpa_star")
    {
      g_planner_ = std::make_shared<LPAStarPathPlanner>(costmap_ros_);
      planner_type_ = GRAPH_PLANNER;
    }
    else if (planner_name_ == "d_star_lite")
    {
      g_planner_ = std::make_shared<DStarLitePathPlanner>(costmap_ros_);
      planner_type_ = GRAPH_PLANNER;
    }
    else if (planner_name_ == "voronoi")
    {
      g_planner_ = std::make_shared<VoronoiPathPlanner>(costmap_ros_,
                                                        costmap_ros_->getLayeredCostmap()->getCircumscribedRadius());
      planner_type_ = GRAPH_PLANNER;
    }
    else if (planner_name_ == "theta_star")
    {
      g_planner_ = std::make_shared<ThetaStarPathPlanner>(costmap_ros_);
      planner_type_ = GRAPH_PLANNER;
    }
    else if (planner_name_ == "lazy_theta_star")
    {
      g_planner_ = std::make_shared<LazyThetaStarPathPlanner>(costmap_ros_);
      planner_type_ = GRAPH_PLANNER;
    }
    else if (planner_name_ == "s_theta_star")
    {
      g_planner_ = std::make_shared<SThetaStarPathPlanner>(costmap_ros_);
      planner_type_ = GRAPH_PLANNER;
    }
    else if (planner_name_ == "hybrid_a_star")
    {
      bool is_reverse;  // whether reverse operation is allowed
      double max_curv;  // maximum curvature of model
      private_nh.param("is_reverse", is_reverse, false);
      private_nh.param("max_curv", max_curv, 1.0);
      g_planner_ = std::make_shared<HybridAStarPathPlanner>(costmap_ros_, is_reverse, max_curv);
      planner_type_ = GRAPH_PLANNER;
    }
    else if (planner_name_ == "lazy")
    {
      g_planner_ = std::make_shared<LazyPathPlanner>(costmap_ros_);
      planner_type_ = GRAPH_PLANNER;
    }
    else if (planner_name_ == "rrt")
    {
      int sample_points;
      double sample_max_d;
      private_nh.param("sample_points", sample_points, 500);  // random sample points
      private_nh.param("sample_max_d", sample_max_d, 5.0);    // max distance between sample points
      g_planner_ = std::make_shared<RRTPathPlanner>(costmap_ros_, sample_points, sample_max_d);
      planner_type_ = SAMPLE_PLANNER;
    }
    else if (planner_name_ == "rrt_star")
    {
      int sample_points;
      double sample_max_d, optimization_r;
      private_nh.param("sample_points", sample_points, 500);     // random sample points
      private_nh.param("sample_max_d", sample_max_d, 5.0);       // max distance between sample points
      private_nh.param("optimization_r", optimization_r, 10.0);  // optimization radius
      g_planner_ = std::make_shared<RRTStarPathPlanner>(costmap_ros_, sample_points, sample_max_d, optimization_r);
      planner_type_ = SAMPLE_PLANNER;
    }
    else if (planner_name_ == "rrt_connect")
    {
      int sample_points;
      double sample_max_d;
      private_nh.param("sample_points", sample_points, 500);  // random sample points
      private_nh.param("sample_max_d", sample_max_d, 5.0);    // max distance between sample points
      g_planner_ = std::make_shared<RRTConnectPathPlanner>(costmap_ros_, sample_points, sample_max_d);
      planner_type_ = SAMPLE_PLANNER;
    }
    else if (planner_name_ == "informed_rrt")
    {
      int sample_points;
      double sample_max_d, optimization_r;
      private_nh.param("sample_points", sample_points, 500);     // random sample points
      private_nh.param("sample_max_d", sample_max_d, 5.0);       // max distance between sample points
      private_nh.param("optimization_r", optimization_r, 10.0);  // optimization radius
      g_planner_ =
          std::make_shared<InformedRRTStarPathPlanner>(costmap_ros_, sample_points, sample_max_d, optimization_r);
      planner_type_ = SAMPLE_PLANNER;
    }
    else if (planner_name_ == "quick_informed_rrt")
    {
      int sample_points, rewire_threads_n;
      double sample_max_d, optimization_r, prior_set_r, step_ext_d, t_freedom;
      private_nh.param("sample_points", sample_points, 500);        // random sample points
      private_nh.param("sample_max_d", sample_max_d, 5.0);          // max distance between sample points
      private_nh.param("optimization_r", optimization_r, 10.0);     // optimization radius
      private_nh.param("prior_sample_set_r", prior_set_r, 10.0);    // radius of priority circles set
      private_nh.param("rewire_threads_num", rewire_threads_n, 2);  // threads number of rewire process
      private_nh.param("step_extend_d", step_ext_d, 5.0);           // threads number of rewire process
      private_nh.param("t_distr_freedom", t_freedom, 1.0);          // freedom of t distribution
      g_planner_ =
          std::make_shared<QuickInformedRRTStarPathPlanner>(costmap_ros_, sample_points, sample_max_d, optimization_r,
                                                            prior_set_r, rewire_threads_n, step_ext_d, t_freedom);
      planner_type_ = SAMPLE_PLANNER;
    }
    else if (planner_name_ == "aco")
    {
      int n_ants, n_inherited, point_num, max_iter, init_mode;
      double alpha, beta, rho, Q;
      private_nh.param("n_ants", n_ants, 50);              // number of ants
      private_nh.param("ant_inherited", n_inherited, 10);  // number of inherited ants
      private_nh.param("point_num_ant", point_num, 5);     // number of position points contained in each ant
      private_nh.param("alpha", alpha, 1.0);               // pheromone weight coefficient
      private_nh.param("beta", beta, 5.0);                 // heuristic factor weight coefficient
      private_nh.param("rho", rho, 0.1);                   // evaporation coefficient
      private_nh.param("Q", Q, 1.0);                       // pheromone gain
      private_nh.param("init_mode_ant", init_mode,
                       static_cast<int>(ACOPathPlanner::CIRCLE));  // Set the generation mode for the initial
                                                                   // position points of the ants
      private_nh.param("max_iter_ant", max_iter, 100);             // maximum iterations

      g_planner_ = std::make_shared<ACOPathPlanner>(costmap_ros_, n_ants, n_inherited, point_num, alpha, beta, rho, Q,
                                                    init_mode, max_iter);
      planner_type_ = EVOLUTION_PLANNER;
    }
    else if (planner_name_ == "pso")
    {
      bool pub_particles;
      int n_particles, n_inherited, point_num, init_mode, max_iter;
      double w_inertial, w_social, w_cognitive, max_speed;

      private_nh.param("n_particles", n_particles, 50);    // number of particles
      private_nh.param("pso_inherited", n_inherited, 10);  // number of inherited particles
      private_nh.param("point_num_pso", point_num, 5);     // number of position points contained in each particle
      private_nh.param("max_speed_pso", max_speed, 40.0);  // The maximum velocity of particle motion
      private_nh.param("w_inertial", w_inertial, 1.0);     // inertia weight
      private_nh.param("w_social", w_social, 2.0);         // social weight
      private_nh.param("w_cognitive", w_cognitive, 1.2);   // cognitive weight
      private_nh.param("init_mode_pso", init_mode,
                       static_cast<int>(ACOPathPlanner::CIRCLE));  // Set the generation mode for the initial
                                                                   // position points of the particle swarm
      private_nh.param("max_iter_pso", max_iter, 30);              // maximum iterations

      g_planner_ = std::make_shared<PSOPathPlanner>(costmap_ros_, n_particles, n_inherited, point_num, w_inertial,
                                                    w_social, w_cognitive, max_speed, init_mode, max_iter);
      planner_type_ = EVOLUTION_PLANNER;
    }
    else if (planner_name_ == "ga")
    {
      bool pub_genets;
      int n_genets, ga_inherited, point_num, init_mode, max_iter;
      double p_select, p_crs, p_mut, max_speed;
      private_nh.param("n_genets", n_genets, 50);          // number of genets
      private_nh.param("ga_inherited", ga_inherited, 20);  // number of inherited genets
      private_nh.param("point_num_ga", point_num, 5);      // number of position points contained in each genets
      private_nh.param("max_speed_ga", max_speed, 40.0);   // The maximum velocity of genets motion
      private_nh.param("p_select", p_select, 0.5);         // selection probability
      private_nh.param("p_crs", p_crs, 0.8);               // crossover probability
      private_nh.param("p_mut", p_mut, 0.3);               // mutation probability
      private_nh.param("init_mode_ga", init_mode,
                       static_cast<int>(GAPathPlanner::CIRCLE));  // Set the generation mode for the initial
                                                                  // position points of the particle swarm
      private_nh.param("max_iter_ga", max_iter, 30);              // maximum iterations

      g_planner_ = std::make_shared<GAPathPlanner>(costmap_ros_, n_genets, ga_inherited, point_num, p_select, p_crs,
                                                   p_mut, max_speed, init_mode, max_iter);
      planner_type_ = EVOLUTION_PLANNER;
    }
    else
    {
      ROS_ERROR("Unknown planner name: %s", planner_name_.c_str());
    }

    ROS_INFO("Using path planner: %s", planner_name_.c_str());

    // pass costmap information to planner (required)
    g_planner_->setFactor(factor_);

    // register planning publisher
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    tree_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("random_tree", 1);
    particles_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("particles", 1);

    // register explorer visualization publisher
    expand_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("expand", 1);

    // register planning service
    make_plan_srv_ = private_nh.advertiseService("make_plan", &PathPlannerNode::makePlanService, this);
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
bool PathPlannerNode::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
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
bool PathPlannerNode::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                               double tolerance, std::vector<geometry_msgs::PoseStamped>& plan)
{
  // start thread mutex
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*g_planner_->getCostMap()->getMutex());
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
  double g_start_x, g_start_y, g_goal_x, g_goal_y;
  if (!g_planner_->world2Map(wx, wy, g_start_x, g_start_y))
  {
    ROS_WARN(
        "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has "
        "been properly localized?");
    return false;
  }
  wx = goal.pose.position.x, wy = goal.pose.position.y;
  if (!g_planner_->world2Map(wx, wy, g_goal_x, g_goal_y))
  {
    ROS_WARN_THROTTLE(1.0,
                      "The goal sent to the global planner is off the global costmap. Planning will always fail to "
                      "this goal.");
    return false;
  }

  // outline the map
  if (is_outline_)
    g_planner_->outlineMap();

  // calculate path
  PathPlanner::Points3d origin_path;
  PathPlanner::Points3d expand;
  bool path_found = false;

  // planning
  path_found = g_planner_->plan({ g_start_x, g_start_y, tf2::getYaw(start.pose.orientation) },
                                { g_goal_x, g_goal_y, tf2::getYaw(goal.pose.orientation) }, origin_path, expand);

  // convert path to ros plan
  if (path_found)
  {
    if (_getPlanFromPath(origin_path, plan))
    {
      geometry_msgs::PoseStamped goalCopy = goal;
      goalCopy.header.stamp = ros::Time::now();
      plan.push_back(goalCopy);
      // path process
      PathPlanner::Points3d origin_plan, prune_plan;
      for (const auto& pt : plan)
      {
        origin_plan.emplace_back(pt.pose.position.x, pt.pose.position.y);
      }

      // visualization
      const auto& visualizer = rmp::common::util::VisualizerPtr::Instance();
      // publish visulization plan
      if (is_expand_)
      {
        if (planner_type_ == GRAPH_PLANNER)
        {
          // publish expand zone
          visualizer->publishExpandZone(expand, costmap_ros_->getCostmap(), expand_pub_, frame_id_);
        }
        else if (planner_type_ == SAMPLE_PLANNER)
        {
          // publish expand tree
          Visualizer::Lines2d tree_lines;
          for (const auto& node : expand)
          {
            // using theta to record parent id element
            if (node.theta() != 0)
            {
              int px_i, py_i;
              double px_d, py_d, x_d, y_d;
              g_planner_->index2Grid(node.theta(), px_i, py_i);
              g_planner_->map2World(px_i, py_i, px_d, py_d);
              g_planner_->map2World(node.x(), node.y(), x_d, y_d);
              tree_lines.emplace_back(
                  std::make_pair<Visualizer::Point2d, Visualizer::Point2d>({ x_d, y_d }, { px_d, py_d }));
            }
          }
          visualizer->publishLines2d(tree_lines, tree_pub_, frame_id_, "tree", Visualizer::DARK_GREEN, 0.05);
        }
        else if (planner_type_ == EVOLUTION_PLANNER)
        {
          // publish expand particles
          Visualizer::Points2d markers;
          for (const auto& node : expand)
          {
            double wx, wy;
            g_planner_->map2World(node.x(), node.y(), wx, wy);
            markers.emplace_back(wx, wy);
          }
          visualizer->publishPoints(markers, particles_pub_, frame_id_, "particles", Visualizer::DARK_GREEN, 0.1,
                                    Visualizer::CUBE);
        }
        else
        {
          ROS_WARN("Unknown planner type.");
        }
      }

      visualizer->publishPlan(origin_plan, plan_pub_, frame_id_);
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
  return !plan.empty();
}

/**
 * @brief Regeister planning service
 * @param req  request from client
 * @param resp response from server
 * @return true
 */
bool PathPlannerNode::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp)
{
  makePlan(req.start, req.goal, resp.plan.poses);
  resp.plan.header.stamp = ros::Time::now();
  resp.plan.header.frame_id = frame_id_;

  return true;
}

/**
 * @brief Calculate plan from planning path
 * @param path path generated by global planner
 * @param plan plan transfromed from path, i.e. [start, ..., goal]
 * @return bool true if successful, else false
 */
bool PathPlannerNode::_getPlanFromPath(PathPlanner::Points3d& path, std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return false;
  }
  plan.clear();

  for (const auto& pt : path)
  {
    double wx, wy;
    g_planner_->map2World(pt.x(), pt.y(), wx, wy);

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
}  // namespace path_planner
}  // namespace rmp