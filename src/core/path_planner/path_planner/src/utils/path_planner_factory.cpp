/**
 * *********************************************************
 *
 * @file: path_planner_factory.cpp
 * @brief: Create the planner with specifical parameters
 * @author: Yang Haodong
 * @date: 2025-02-16
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "path_planner/utils/path_planner_factory.h"

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
#include "path_planner/graph_planner/hybrid_astar_planner/hybrid_astar_planner.h"
#include "path_planner/graph_planner/voronoi_planner.h"
#include "path_planner/graph_planner/lazy_planner.h"

// sample-based planner
#include "path_planner/sample_planner/rrt_planner.h"
#include "path_planner/sample_planner/rrt_star_planner.h"
#include "path_planner/sample_planner/rrt_connect_planner.h"
#include "path_planner/sample_planner/informed_rrt_star_planner.h"

// evolutionary-based planner
#include "path_planner/evolutionary_planner/aco_planner.h"
#include "path_planner/evolutionary_planner/pso_planner.h"
#include "path_planner/evolutionary_planner/ga_planner.h"

namespace rmp {
namespace path_planner {
/**
 * @brief Create and configure planner
 * @param nh ROS node handler
 * @param costmap_ros costmap ROS wrapper
 * @param planner_props planner property
 * @return bool true if create successful, else false
 */
bool PathPlannerFactory::createPlanner(ros::NodeHandle& nh,
                                       costmap_2d::Costmap2DROS* costmap_ros,
                                       PlannerProps& planner_props) {
  std::string planner_name;
  nh.param("planner_name", planner_name, (std::string) "astar");  // planner name

  if (planner_name == "astar") {
    planner_props.planner_ptr = std::make_shared<AStarPathPlanner>(costmap_ros);
    planner_props.planner_type = GRAPH_PLANNER;
  } else if (planner_name == "dijkstra") {
    planner_props.planner_ptr = std::make_shared<AStarPathPlanner>(costmap_ros, true);
    planner_props.planner_type = GRAPH_PLANNER;
  } else if (planner_name == "gbfs") {
    planner_props.planner_ptr =
        std::make_shared<AStarPathPlanner>(costmap_ros, false, true);
    planner_props.planner_type = GRAPH_PLANNER;
  } else if (planner_name == "jps") {
    planner_props.planner_ptr = std::make_shared<JPSPathPlanner>(costmap_ros);
    planner_props.planner_type = GRAPH_PLANNER;
  } else if (planner_name == "dstar") {
    planner_props.planner_ptr = std::make_shared<DStarPathPlanner>(costmap_ros);
    planner_props.planner_type = GRAPH_PLANNER;
  } else if (planner_name == "lpa_star") {
    planner_props.planner_ptr = std::make_shared<LPAStarPathPlanner>(costmap_ros);
    planner_props.planner_type = GRAPH_PLANNER;
  } else if (planner_name == "dstar_lite") {
    planner_props.planner_ptr = std::make_shared<DStarLitePathPlanner>(costmap_ros);
    planner_props.planner_type = GRAPH_PLANNER;
  } else if (planner_name == "voronoi") {
    planner_props.planner_ptr = std::make_shared<VoronoiPathPlanner>(
        costmap_ros, costmap_ros->getLayeredCostmap()->getCircumscribedRadius());
    planner_props.planner_type = GRAPH_PLANNER;
  } else if (planner_name == "lazy") {
    planner_props.planner_ptr = std::make_shared<LazyPathPlanner>(costmap_ros);
    planner_props.planner_type = GRAPH_PLANNER;
  } else if (planner_name == "theta_star") {
    planner_props.planner_ptr = std::make_shared<ThetaStarPathPlanner>(costmap_ros);
    planner_props.planner_type = GRAPH_PLANNER;
  } else if (planner_name == "lazy_theta_star") {
    planner_props.planner_ptr = std::make_shared<LazyThetaStarPathPlanner>(costmap_ros);
    planner_props.planner_type = GRAPH_PLANNER;
  } else if (planner_name == "s_theta_star") {
    planner_props.planner_ptr = std::make_shared<SThetaStarPathPlanner>(costmap_ros);
    planner_props.planner_type = GRAPH_PLANNER;
  } else if (planner_name == "hybrid_astar") {
    planner_props.planner_ptr = std::make_shared<HybridAStarPathPlanner>(costmap_ros);
    planner_props.planner_type = GRAPH_PLANNER;
  } else if (planner_name == "rrt") {
    planner_props.planner_ptr = std::make_shared<RRTPathPlanner>(costmap_ros);
    planner_props.planner_type = SAMPLE_PLANNER;
  } else if (planner_name == "rrt_star") {
    planner_props.planner_ptr = std::make_shared<RRTStarPathPlanner>(costmap_ros);
    planner_props.planner_type = SAMPLE_PLANNER;
  } else if (planner_name == "rrt_connect") {
    planner_props.planner_ptr = std::make_shared<RRTConnectPathPlanner>(costmap_ros);
    planner_props.planner_type = SAMPLE_PLANNER;
  } else if (planner_name == "informed_rrt") {
    planner_props.planner_ptr = std::make_shared<InformedRRTStarPathPlanner>(costmap_ros);
    planner_props.planner_type = SAMPLE_PLANNER;
  } else if (planner_name == "aco") {
    planner_props.planner_ptr = std::make_shared<ACOPathPlanner>(costmap_ros);
    planner_props.planner_type = EVOLUTION_PLANNER;
  } else if (planner_name == "pso") {
    planner_props.planner_ptr = std::make_shared<PSOPathPlanner>(costmap_ros);
    planner_props.planner_type = EVOLUTION_PLANNER;
  } else if (planner_name == "ga") {
    planner_props.planner_ptr = std::make_shared<GAPathPlanner>(costmap_ros);
    planner_props.planner_type = EVOLUTION_PLANNER;
  } else {
    R_ERROR << "Unknown planner name: " << planner_name;
    return false;
  }

  R_INFO << "Using path planner: " << planner_name;
  return true;
}
}  // namespace path_planner
}  // namespace rmp