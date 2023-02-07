
#ifndef DWA_H
#define DWA_H

#include <vector>
#include <Eigen/Core>

#include <dwa_planner/DWAPlannerConfig.h>

// for creating a local cost grid
#include <base_local_planner/map_grid_visualizer.h>

// for obstacle data access
#include <costmap_2d/costmap_2d.h>

#include <base_local_planner/trajectory.h>
#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/simple_trajectory_generator.h>

#include <base_local_planner/oscillation_cost_function.h>
#include <base_local_planner/map_grid_cost_function.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/twirling_cost_function.h>
#include <base_local_planner/simple_scored_sampling_planner.h>

#include <nav_msgs/Path.h>

namespace dwa_planner
{
/**
 * @class DWAPlanner
 * @brief A class implementing a local planner using the Dynamic Window Approach
 */
class DWA
{
public:
  /**
   * @brief  Constructor for the planner
   * @param name The name of the planner
   * @param costmap_ros A pointer to the costmap instance the planner should use
   * @param global_frame the frame id of the tf frame to use
   */
  DWA(std::string name, base_local_planner::LocalPlannerUtil* planner_util);

  /**
   * @brief Reconfigures the trajectory planner
   */
  void reconfigure(DWAPlannerConfig& cfg);

  /**
   * @brief  Check if a trajectory is legal for a position/velocity pair
   * @param pos The robot's position
   * @param vel The robot's velocity
   * @param vel_samples The desired velocity
   * @return True if the trajectory is valid, false otherwise
   */
  bool checkTrajectory(const Eigen::Vector3f pos, const Eigen::Vector3f vel, const Eigen::Vector3f vel_samples);

  /**
   * @brief Given the current position and velocity of the robot, find the best trajectory to exectue
   * @param global_pose The current position of the robot
   * @param global_vel The current velocity of the robot
   * @param drive_velocities The velocities to send to the robot base
   * @return The highest scoring trajectory. A cost >= 0 means the trajectory is legal to execute.
   */
  base_local_planner::Trajectory findBestPath(const geometry_msgs::PoseStamped& global_pose,
                                              const geometry_msgs::PoseStamped& global_vel,
                                              geometry_msgs::PoseStamped& drive_velocities);

  /**
   * @brief  Update the cost functions before planning
   * @param  global_pose The robot's current pose
   * @param  new_plan The new global plan
   * @param  footprint_spec The robot's footprint
   *
   * The obstacle cost function gets the footprint.
   * The path and goal cost functions get the global_plan
   * The alignment cost functions get a version of the global plan
   *   that is modified based on the global_pose
   */
  void updatePlanAndLocalCosts(const geometry_msgs::PoseStamped& global_pose,
                               const std::vector<geometry_msgs::PoseStamped>& new_plan,
                               const std::vector<geometry_msgs::Point>& footprint_spec);

  /**
   * @brief Get the period at which the local planner is expected to run
   * @return The simulation period
   */
  double getSimPeriod()
  {
    return sim_period_;
  }

  /**
   * @brief Compute the components and total cost for a map grid cell
   * @param cx The x coordinate of the cell in the map grid
   * @param cy The y coordinate of the cell in the map grid
   * @param path_cost Will be set to the path distance component of the cost function
   * @param goal_cost Will be set to the goal distance component of the cost function
   * @param occ_cost Will be set to the costmap value of the cell
   * @param total_cost Will be set to the value of the overall cost function, taking into account the scaling parameters
   * @return True if the cell is traversible and therefore a legal location for the robot to move to
   */
  bool getCellCosts(int cx, int cy, float& path_cost, float& goal_cost, float& occ_cost, float& total_cost);

  /**
   * sets new plan and resets state
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

private:
  base_local_planner::LocalPlannerUtil* planner_util_;

  double stop_time_buffer_;  ///< @brief How long before hitting something we're going to enforce that the robot stop
  double path_distance_bias_, goal_distance_bias_, occdist_scale_;
  Eigen::Vector3f vsamples_;

  double sim_period_;  ///< @brief The number of seconds to use to compute max/min vels for dwa
  base_local_planner::Trajectory result_traj_;

  double forward_point_distance_;

  std::vector<geometry_msgs::PoseStamped> global_plan_;

  boost::mutex configuration_mutex_;
  std::string frame_id_;
  ros::Publisher traj_cloud_pub_;
  bool publish_cost_grid_pc_;  ///< @brief Whether or not to build and publish a PointCloud
  bool publish_traj_pc_;

  double cheat_factor_;

  base_local_planner::MapGridVisualizer map_viz_;  ///< @brief The map grid visualizer for outputting the potential
                                                   ///< field generated by the cost function

  // see constructor body for explanations
  base_local_planner::SimpleTrajectoryGenerator generator_;
  base_local_planner::OscillationCostFunction oscillation_costs_;
  base_local_planner::ObstacleCostFunction obstacle_costs_;
  base_local_planner::MapGridCostFunction path_costs_;
  base_local_planner::MapGridCostFunction goal_costs_;
  base_local_planner::MapGridCostFunction goal_front_costs_;
  base_local_planner::MapGridCostFunction alignment_costs_;
  base_local_planner::TwirlingCostFunction twirling_costs_;

  base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;
};
};  // namespace dwa_planner
#endif
