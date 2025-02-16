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

// path planner
#include "path_planner/path_planner_node.h"

#include "common/util/log.h"
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

    private_nh.param("default_tolerance", tolerance_, 0.0);                  // error tolerance
    private_nh.param("outline_map", is_outline_, false);                     // whether outline the map or not
    private_nh.param("expand_zone", is_expand_, false);                      // whether publish expand zone or not
    private_nh.param("show_safety_corridor", show_safety_corridor_, false);  // whether visualize safety corridor

    PathPlannerFactory::PlannerProps path_planner_props;
    if (!PathPlannerFactory::createPlanner(private_nh, costmap_ros_, path_planner_props))
    {
      R_ERROR << "Create path planner failed.";
    }

    g_planner_ = path_planner_props.planner_ptr;
    planner_type_ = path_planner_props.planner_type;

    // register planning publisher
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    points_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("key_points", 1);
    lines_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("safety_corridor", 1);
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
    R_ERROR << "This planner has not been initialized yet, but it is being used, please call initialize() before use";
    return false;
  }
  // clear existing plan
  plan.clear();

  // judege whether goal and start node in costmap frame or not
  if (goal.header.frame_id != frame_id_)
  {
    R_ERROR << "The goal pose passed to this planner must be in the " << frame_id_ << " frame. It is instead in the "
            << goal.header.frame_id << " frame.";
    return false;
  }

  if (start.header.frame_id != frame_id_)
  {
    R_ERROR << "The start pose passed to this planner must be in the " << frame_id_ << " frame. It is instead in the "
            << start.header.frame_id << " frame.";
    return false;
  }

  // visualization
  const auto& visualizer = rmp::common::util::VisualizerPtr::Instance();

  // outline the map
  if (is_outline_)
    g_planner_->outlineMap();

  // calculate path
  PathPlanner::Points3d origin_plan;
  PathPlanner::Points3d expand;
  bool path_found = false;

  // planning
  // auto start_time = std::chrono::high_resolution_clock::now();
  path_found = g_planner_->plan({ start.pose.position.x, start.pose.position.y, tf2::getYaw(start.pose.orientation) },
                                { goal.pose.position.x, goal.pose.position.y, tf2::getYaw(goal.pose.orientation) },
                                origin_plan, expand);
  // auto finish_time = std::chrono::high_resolution_clock::now();
  // std::chrono::duration<double> cal_time = finish_time - start_time;
  // R_INFO << "Calculation Time: " << cal_time.count() << " s";

  // convert path to ros plan
  if (path_found)
  {
    if (_getPlanFromPath(origin_plan, plan))
    {
      geometry_msgs::PoseStamped goalCopy = goal;
      goalCopy.header.stamp = ros::Time::now();
      plan.pop_back();
      plan.push_back(goalCopy);
      plan[0].pose.orientation = start.pose.orientation;

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
          R_WARN << "Unknown planner type.";
        }
      }

      visualizer->publishPlan(origin_plan, plan_pub_, frame_id_);
    }
    else
    {
      R_ERROR << "Failed to get a plan from path when a legal path was found. This shouldn't happen.";
    }
  }
  else
  {
    R_ERROR << "Failed to get a path.";
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
    R_ERROR << "This planner has not been initialized yet, but it is being used, please call initialize() before use";
    return false;
  }
  plan.clear();

  for (const auto& pt : path)
  {
    // double wx, wy;
    // g_planner_->map2World(pt.x(), pt.y(), wx, wy);

    // coding as message type
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = pt.x();
    pose.pose.position.y = pt.y();
    pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, pt.theta());
    pose.pose.orientation.x = q.getX();
    pose.pose.orientation.y = q.getY();
    pose.pose.orientation.z = q.getZ();
    pose.pose.orientation.w = q.getW();
    plan.push_back(pose);
  }

  return !plan.empty();
}
}  // namespace path_planner
}  // namespace rmp