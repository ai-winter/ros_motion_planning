#include <pluginlib/class_list_macros.h>
#include "orca_planner.h"

PLUGINLIB_EXPORT_CLASS(orca_planner::OrcaPlanner, nav_core::BaseLocalPlanner)

namespace orca_planner
{

OrcaPlanner::OrcaPlanner() : initialized_(false), costmap_ros_(nullptr), tf_(nullptr), odom_flag_(false)
{
}

OrcaPlanner::OrcaPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) : OrcaPlanner()
{
  initialize(name, tf, costmap_ros);
}

OrcaPlanner::~OrcaPlanner()
{
}

// Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
void OrcaPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_)
  {
    RVO::RVOSimulator* sim = new RVO::RVOSimulator();
    sim->setTimeStep(0.25f);

    initialized_ = true;
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    ros::NodeHandle nh = ros::NodeHandle("~/" + name);

    nh.param("agent_number", agent_number_, -1);
    nh.param("agent_id", agent_id_, -1);

    other_odoms_.resize(agent_number_);
    for (int i = 0; i < agent_number_; ++i)
    {
      ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
          "/robot" + std::to_string(i + 1) + "/odom", 1, boost::bind(&OrcaPlanner::odometryCallback, this, _1, i + 1));
      odom_subs_.push_back(odom_sub);
      ROS_WARN("agent %d, subscribe to agent %d.", agent_id_, i + 1);
    }
  }
}

void OrcaPlanner::odometryCallback(const nav_msgs::OdometryConstPtr& msg, int agent_id)
{
  if (!odom_flag_)
    odom_flag_ = true;

  other_odoms_[agent_id - 1] = *msg;
}

bool OrcaPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized");
    return false;
  }
  return true;
}

bool OrcaPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized");
    return false;
  }
  return true;
}

bool OrcaPlanner::isGoalReached()
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized");
    return false;
  }
  return false;
}
}  // namespace orca_planner