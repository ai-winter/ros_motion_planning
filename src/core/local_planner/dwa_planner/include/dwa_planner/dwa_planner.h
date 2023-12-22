
#ifndef DWA_PLANNER_H
#define DWA_PLANNER_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf2_ros/buffer.h>

#include <dynamic_reconfigure/server.h>
#include <dwa_planner/DWAPlannerConfig.h>

#include <angles/angles.h>

#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/latched_stop_rotate_controller.h>

#include <base_local_planner/odometry_helper_ros.h>

#include <dwa_planner/dwa.h>

namespace dwa_planner
{
/**
 * @class DWAPlannerROS
 * @brief ROS Wrapper for the DWAPlanner that adheres to the
 * BaseLocalPlanner interface and can be used as a plugin for move_base.
 */
class DWAPlanner : public nav_core::BaseLocalPlanner
{
public:
  /**
   * @brief  Constructor for DWAPlannerROS wrapper
   */
  DWAPlanner();

  /**
   * @brief  Constructs the ros wrapper
   * @param name The name to give this instance of the trajectory planner
   * @param tf A pointer to a transform listener
   * @param costmap The cost map to use for assigning costs to trajectories
   */
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief  Destructor for the wrapper
   */
  ~DWAPlanner();

  /**
   * @brief  Given the current position, orientation, and velocity of the robot,
   * compute velocity commands to send to the base
   * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
   * @return True if a valid trajectory was found, false otherwise
   */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  /**
   * @brief  Given the current position, orientation, and velocity of the robot,
   * compute velocity commands to send to the base, using dynamic window approach
   * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
   * @return True if a valid trajectory was found, false otherwise
   */
  bool dwaComputeVelocityCommands(geometry_msgs::PoseStamped& global_pose, geometry_msgs::Twist& cmd_vel);

  /**
   * @brief  Set the plan that the controller is following
   * @param orig_global_plan The plan to pass to the controller
   * @return True if the plan was updated successfully, false otherwise
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  /**
   * @brief  Check if the goal pose has been achieved
   * @return True if achieved, false otherwise
   */
  bool isGoalReached();

  bool isInitialized()
  {
    return initialized_;
  }

private:
  /**
   * @brief Callback to update the local planner's parameters based on dynamic reconfigure
   */
  void reconfigureCB(DWAPlannerConfig& config, uint32_t level);

  void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

  void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

  tf2_ros::Buffer* tf_;  ///< @brief Used for transforming point clouds

  // for visualisation, publishers of global and local plan
  ros::Publisher g_plan_pub_, l_plan_pub_;

  base_local_planner::LocalPlannerUtil planner_util_;

  boost::shared_ptr<DWA> dp_;  ///< @brief The trajectory controller

  costmap_2d::Costmap2DROS* costmap_ros_;

  dynamic_reconfigure::Server<DWAPlannerConfig>* dsrv_;
  dwa_planner::DWAPlannerConfig default_config_;
  bool setup_;
  geometry_msgs::PoseStamped current_pose_;

  base_local_planner::LatchedStopRotateController latchedStopRotateController_;

  bool initialized_;

  base_local_planner::OdometryHelperRos odom_helper_;
  std::string odom_topic_;
};
};  // namespace dwa_planner
#endif
