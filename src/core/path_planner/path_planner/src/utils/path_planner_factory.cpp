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
#include "path_planner/graph_planner/bi_jps_planner.h"
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
#include "path_planner/sample_planner/quick_informed_rrt_star_planner.h"

// evolutionary-based planner
#include "path_planner/evolutionary_planner/aco_planner.h"
#include "path_planner/evolutionary_planner/pso_planner.h"
#include "path_planner/evolutionary_planner/ga_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Create and configure planner
 * @param nh ROS node handler
 * @param costmap_ros costmap ROS wrapper
 * @param planner_props planner property
 * @return bool true if create successful, else false
 */
bool PathPlannerFactory::createPlanner(ros::NodeHandle& nh, costmap_2d::Costmap2DROS* costmap_ros,
                                       PlannerProps& planner_props)
{
  double obstacle_factor;
  std::string planner_name;
  nh.param("planner_name", planner_name, (std::string) "astar");  // planner name
  nh.param("obstacle_factor", obstacle_factor, 0.5);              // obstacle factor

  if (planner_name == "astar")
  {
    planner_props.planner_ptr = std::make_shared<AStarPathPlanner>(costmap_ros, obstacle_factor);
    planner_props.planner_type = GRAPH_PLANNER;
  }
  else if (planner_name == "dijkstra")
  {
    planner_props.planner_ptr = std::make_shared<AStarPathPlanner>(costmap_ros, obstacle_factor, true);
    planner_props.planner_type = GRAPH_PLANNER;
  }
  else if (planner_name == "gbfs")
  {
    planner_props.planner_ptr = std::make_shared<AStarPathPlanner>(costmap_ros, obstacle_factor, false, true);
    planner_props.planner_type = GRAPH_PLANNER;
  }
  else if (planner_name == "jps")
  {
    planner_props.planner_ptr = std::make_shared<JPSPathPlanner>(costmap_ros, obstacle_factor);
    planner_props.planner_type = GRAPH_PLANNER;
  }
  else if (planner_name == "bi_jps")
  {
    planner_props.planner_ptr = std::make_shared<BiJPSPathPlanner>(costmap_ros, obstacle_factor);
    planner_props.planner_type = GRAPH_PLANNER;
  }
  else if (planner_name == "dstar")
  {
    planner_props.planner_ptr = std::make_shared<DStarPathPlanner>(costmap_ros, obstacle_factor);
    planner_props.planner_type = GRAPH_PLANNER;
  }
  else if (planner_name == "lpa_star")
  {
    planner_props.planner_ptr = std::make_shared<LPAStarPathPlanner>(costmap_ros, obstacle_factor);
    planner_props.planner_type = GRAPH_PLANNER;
  }
  else if (planner_name == "dstar_lite")
  {
    planner_props.planner_ptr = std::make_shared<DStarLitePathPlanner>(costmap_ros, obstacle_factor);
    planner_props.planner_type = GRAPH_PLANNER;
  }
  else if (planner_name == "voronoi")
  {
    planner_props.planner_ptr = std::make_shared<VoronoiPathPlanner>(
        costmap_ros, costmap_ros->getLayeredCostmap()->getCircumscribedRadius(), obstacle_factor);
    planner_props.planner_type = GRAPH_PLANNER;
  }
  else if (planner_name == "lazy")
  {
    planner_props.planner_ptr = std::make_shared<LazyPathPlanner>(costmap_ros);
    planner_props.planner_type = GRAPH_PLANNER;
  }
  else if (planner_name == "theta_star")
  {
    planner_props.planner_ptr = std::make_shared<ThetaStarPathPlanner>(costmap_ros, obstacle_factor);
    planner_props.planner_type = GRAPH_PLANNER;
  }
  else if (planner_name == "lazy_theta_star")
  {
    planner_props.planner_ptr = std::make_shared<LazyThetaStarPathPlanner>(costmap_ros, obstacle_factor);
    planner_props.planner_type = GRAPH_PLANNER;
  }
  else if (planner_name == "s_theta_star")
  {
    planner_props.planner_ptr = std::make_shared<SThetaStarPathPlanner>(costmap_ros, obstacle_factor);
    planner_props.planner_type = GRAPH_PLANNER;
  }
  else if (planner_name == "hybrid_astar")
  {
    double goal_tolerance, cost_penalty, curve_sample_ratio, minimum_turning_radius, non_straight_penalty,
        change_penalty, reverse_penalty, retrospective_penalty, lookup_table_dim, analytic_expansion_ratio,
        analytic_expansion_max_length;
    int dim_3_size, max_iterations, max_approach_iterations, motion_model;
    bool traverse_unknown, cache_obstacle_heuristic, downsample_obstacle_heuristic;
    nh.param(planner_name + "/motion_model", motion_model, 2);
    nh.param(planner_name + "/goal_tolerance", goal_tolerance, 0.125);
    nh.param(planner_name + "/dim_3_size", dim_3_size, 1);
    nh.param(planner_name + "/max_iterations", max_iterations, 10000);
    nh.param(planner_name + "/max_approach_iterations", max_approach_iterations, 1000);
    nh.param(planner_name + "/traverse_unknown", traverse_unknown, true);
    nh.param(planner_name + "/cost_penalty", cost_penalty, 1.0);
    nh.param(planner_name + "/cache_obstacle_heuristic", cache_obstacle_heuristic, false);
    nh.param(planner_name + "/curve_sample_ratio", curve_sample_ratio, 0.1);
    nh.param(planner_name + "/minimum_turning_radius", minimum_turning_radius, 0.4);
    nh.param(planner_name + "/non_straight_penalty", non_straight_penalty, 1.0);
    nh.param(planner_name + "/change_penalty", change_penalty, 1.0);
    nh.param(planner_name + "/reverse_penalty", reverse_penalty, 1.0);
    nh.param(planner_name + "/retrospective_penalty", retrospective_penalty, 0.0);
    nh.param(planner_name + "/lookup_table_dim", lookup_table_dim, 10.0);
    nh.param(planner_name + "/analytic_expansion_ratio", analytic_expansion_ratio, 3.5);
    nh.param(planner_name + "/analytic_expansion_max_length", analytic_expansion_max_length, 3.0);
    nh.param(planner_name + "/downsample_obstacle_heuristic", downsample_obstacle_heuristic, true);

    HybridSearchInfo info;
    info.motion_model = motion_model;
    info.goal_tolerance = goal_tolerance;
    info.dim_3_size = dim_3_size;
    info.max_iterations = max_iterations;
    info.max_approach_iterations = max_approach_iterations;
    info.traverse_unknown = traverse_unknown;
    info.cost_penalty = cost_penalty;
    info.cache_obstacle_heuristic = cache_obstacle_heuristic;
    info.curve_sample_ratio = curve_sample_ratio;
    info.minimum_turning_radius = minimum_turning_radius / costmap_ros->getCostmap()->getResolution();
    info.non_straight_penalty = non_straight_penalty;
    info.change_penalty = change_penalty;
    info.reverse_penalty = reverse_penalty;
    info.retrospective_penalty = retrospective_penalty;
    info.lookup_table_dim = lookup_table_dim;
    info.analytic_expansion_ratio = analytic_expansion_ratio;
    info.analytic_expansion_max_length = analytic_expansion_max_length;
    info.downsample_obstacle_heuristic = downsample_obstacle_heuristic;
    planner_props.planner_ptr = std::make_shared<HybridAStarPathPlanner>(costmap_ros, obstacle_factor, info);
    planner_props.planner_type = GRAPH_PLANNER;
  }
  else if (planner_name == "rrt")
  {
    int sample_points;
    double sample_max_d;
    nh.param(planner_name + "/sample_points", sample_points, 500);  // random sample points
    nh.param(planner_name + "/sample_max_d", sample_max_d, 5.0);    // max distance between sample points
    planner_props.planner_ptr =
        std::make_shared<RRTPathPlanner>(costmap_ros, obstacle_factor, sample_points, sample_max_d);
    planner_props.planner_type = SAMPLE_PLANNER;
  }
  else if (planner_name == "rrt_star")
  {
    int sample_points;
    double sample_max_d, optimization_r;
    nh.param(planner_name + "/sample_points", sample_points, 500);     // random sample points
    nh.param(planner_name + "/sample_max_d", sample_max_d, 5.0);       // max distance between sample points
    nh.param(planner_name + "/optimization_r", optimization_r, 10.0);  // optimization radius
    planner_props.planner_ptr =
        std::make_shared<RRTStarPathPlanner>(costmap_ros, obstacle_factor, sample_points, sample_max_d, optimization_r);
    planner_props.planner_type = SAMPLE_PLANNER;
  }
  else if (planner_name == "rrt_connect")
  {
    int sample_points;
    double sample_max_d;
    nh.param(planner_name + "/sample_points", sample_points, 500);  // random sample points
    nh.param(planner_name + "/sample_max_d", sample_max_d, 5.0);    // max distance between sample points
    planner_props.planner_ptr =
        std::make_shared<RRTConnectPathPlanner>(costmap_ros, obstacle_factor, sample_points, sample_max_d);
    planner_props.planner_type = SAMPLE_PLANNER;
  }
  else if (planner_name == "informed_rrt")
  {
    int sample_points;
    double sample_max_d, optimization_r;
    nh.param(planner_name + "/sample_points", sample_points, 500);     // random sample points
    nh.param(planner_name + "/sample_max_d", sample_max_d, 5.0);       // max distance between sample points
    nh.param(planner_name + "/optimization_r", optimization_r, 10.0);  // optimization radius
    planner_props.planner_ptr = std::make_shared<InformedRRTStarPathPlanner>(
        costmap_ros, obstacle_factor, sample_points, sample_max_d, optimization_r);
    planner_props.planner_type = SAMPLE_PLANNER;
  }
  else if (planner_name == "quick_informed_rrt")
  {
    int sample_points, rewire_threads_n;
    double sample_max_d, optimization_r, prior_set_r, step_ext_d, t_freedom;
    nh.param(planner_name + "/sample_points", sample_points, 500);        // random sample points
    nh.param(planner_name + "/sample_max_d", sample_max_d, 5.0);          // max distance between sample points
    nh.param(planner_name + "/optimization_r", optimization_r, 10.0);     // optimization radius
    nh.param(planner_name + "/prior_sample_set_r", prior_set_r, 10.0);    // radius of priority circles set
    nh.param(planner_name + "/rewire_threads_num", rewire_threads_n, 2);  // threads number of rewire process
    nh.param(planner_name + "/step_extend_d", step_ext_d, 5.0);           // threads number of rewire process
    nh.param(planner_name + "/t_distr_freedom", t_freedom, 1.0);          // freedom of t distribution
    planner_props.planner_ptr = std::make_shared<QuickInformedRRTStarPathPlanner>(
        costmap_ros, obstacle_factor, sample_points, sample_max_d, optimization_r, prior_set_r, rewire_threads_n,
        step_ext_d, t_freedom);
    planner_props.planner_type = SAMPLE_PLANNER;
  }
  else if (planner_name == "aco")
  {
    int n_ants, n_inherited, point_num, max_iter, init_mode;
    double alpha, beta, rho, Q;
    nh.param(planner_name + "/population_num", n_ants, 50);      // number of ants
    nh.param(planner_name + "/inherited_num", n_inherited, 10);  // number of inherited ants
    nh.param(planner_name + "/point_num", point_num, 5);         // number of position points contained in each ant
    nh.param(planner_name + "/alpha", alpha, 1.0);               // pheromone weight coefficient
    nh.param(planner_name + "/beta", beta, 5.0);                 // heuristic factor weight coefficient
    nh.param(planner_name + "/rho", rho, 0.1);                   // evaporation coefficient
    nh.param(planner_name + "/Q", Q, 1.0);                       // pheromone gain
    nh.param(planner_name + "/init_mode", init_mode,
             static_cast<int>(ACOPathPlanner::CIRCLE));   // Set the generation mode for the initial
                                                          // position points of the ants
    nh.param(planner_name + "/max_iter", max_iter, 100);  // maximum iterations

    planner_props.planner_ptr = std::make_shared<ACOPathPlanner>(costmap_ros, obstacle_factor, n_ants, n_inherited,
                                                                 point_num, alpha, beta, rho, Q, init_mode, max_iter);
    planner_props.planner_type = EVOLUTION_PLANNER;
  }
  else if (planner_name == "pso")
  {
    int n_particles, n_inherited, point_num, init_mode, max_iter;
    double w_inertial, w_social, w_cognitive, max_speed;

    nh.param(planner_name + "/population_num", n_particles, 50);  // number of particles
    nh.param(planner_name + "/inherited_num", n_inherited, 10);   // number of inherited particles
    nh.param(planner_name + "/point_num", point_num,
             5);                                                // number of position points contained in each particle
    nh.param(planner_name + "/max_speed", max_speed, 40.0);     // The maximum velocity of particle motion
    nh.param(planner_name + "/w_inertial", w_inertial, 1.0);    // inertia weight
    nh.param(planner_name + "/w_social", w_social, 2.0);        // social weight
    nh.param(planner_name + "/w_cognitive", w_cognitive, 1.2);  // cognitive weight
    nh.param(planner_name + "/init_mode", init_mode,
             static_cast<int>(ACOPathPlanner::CIRCLE));  // Set the generation mode for the initial
                                                         // position points of the particle swarm
    nh.param(planner_name + "/max_iter", max_iter, 30);  // maximum iterations

    planner_props.planner_ptr =
        std::make_shared<PSOPathPlanner>(costmap_ros, obstacle_factor, n_particles, n_inherited, point_num, w_inertial,
                                         w_social, w_cognitive, max_speed, init_mode, max_iter);
    planner_props.planner_type = EVOLUTION_PLANNER;
  }
  else if (planner_name == "ga")
  {
    int n_genets, ga_inherited, point_num, init_mode, max_iter;
    double p_select, p_crs, p_mut, max_speed;
    nh.param(planner_name + "/population_num", n_genets, 50);     // number of genets
    nh.param(planner_name + "/inherited_num", ga_inherited, 20);  // number of inherited genets
    nh.param(planner_name + "/point_num", point_num,
             5);                                             // number of position points contained in each genets
    nh.param(planner_name + "/max_speed", max_speed, 40.0);  // The maximum velocity of genets motion
    nh.param(planner_name + "/p_select", p_select, 0.5);     // selection probability
    nh.param(planner_name + "/p_crs", p_crs, 0.8);           // crossover probability
    nh.param(planner_name + "/p_mut", p_mut, 0.3);           // mutation probability
    nh.param(planner_name + "/init_mode", init_mode,
             static_cast<int>(GAPathPlanner::CIRCLE));   // Set the generation mode for the initial
                                                         // position points of the particle swarm
    nh.param(planner_name + "/max_iter", max_iter, 30);  // maximum iterations

    planner_props.planner_ptr =
        std::make_shared<GAPathPlanner>(costmap_ros, obstacle_factor, n_genets, ga_inherited, point_num, p_select,
                                        p_crs, p_mut, max_speed, init_mode, max_iter);
    planner_props.planner_type = EVOLUTION_PLANNER;
  }
  else
  {
    R_ERROR << "Unknown planner name: " << planner_name;
    return false;
  }

  R_INFO << "Using path planner: " << planner_name;
  return true;
}
}  // namespace path_planner
}  // namespace rmp