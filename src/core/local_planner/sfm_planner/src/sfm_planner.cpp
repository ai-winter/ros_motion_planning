#include <pluginlib/class_list_macros.h>
#include "sfm_planner.h"

PLUGINLIB_EXPORT_CLASS(sfm_planner::SfmPlanner, nav_core::BaseLocalPlanner)

namespace sfm_planner
{

SfmPlanner::SfmPlanner()
  : initialized_(false), costmap_ros_(nullptr), tf_(nullptr), odom_flag_(false), goal_reached_(false), first_plan_(true)
{
}

SfmPlanner::SfmPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) : SfmPlanner()
{
  initialize(name, tf, costmap_ros);
}

SfmPlanner::~SfmPlanner()
{
}

// Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
void SfmPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_)
  {
    initialized_ = true;
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    ros::NodeHandle nh = ros::NodeHandle("~/" + name);

    // base
    nh.param("goal_dist_tolerance", goal_dist_tol_, 0.2);
    nh.param("rotate_tolerance", rotate_tol_, 0.5);
    nh.param("base_frame", base_frame_, base_frame_);
    nh.param("map_frame", map_frame_, map_frame_);

    // multi-robot info
    nh.param("agent_number", agent_number_, -1);
    nh.param("agent_id", agent_id_, -1);

    // linear velocity
    nh.param("max_v", max_v_, 0.5);
    nh.param("min_v", min_v_, 0.0);
    nh.param("max_v_inc", max_v_inc_, 0.5);

    // angular velocity
    nh.param("max_w", max_w_, 1.57);
    nh.param("min_w", min_w_, 0.0);
    nh.param("max_w_inc", max_w_inc_, 1.57);

    // sfm parameters
    nh.param("radius", agent_.radius, 0.2);
    nh.param("agent_max_velocity", agent_.desiredVelocity, 0.8);
    nh.param("agent_goal_weight", agent_.params.forceFactorDesired, 2.0);
    nh.param("agent_obstacle_weight", agent_.params.forceFactorObstacle, 10.0);
    nh.param("agent_sigma_obstacle", agent_.params.forceSigmaObstacle, 0.2);
    nh.param("agent_social_weight", agent_.params.forceFactorSocial, 2.1);

    others_.resize(agent_number_);
    other_odoms_.resize(agent_number_);
    for (int i = 0; i < agent_number_; ++i)
    {
      ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
          "/robot" + std::to_string(i + 1) + "/odom", 1, boost::bind(&SfmPlanner::odometryCallback, this, _1, i + 1));
      odom_subs_.push_back(odom_sub);
      ROS_INFO("agent %d, subscribe to agent %d.", agent_id_, i + 1);
    }

    int spin_cnt = 5 * agent_number_;
    ros::Rate rate(10);
    ROS_WARN("[SFM] Waiting for odoms...");
    while (spin_cnt-- > 0)
    {
      ros::spinOnce();
      rate.sleep();
    }

    double controller_freqency;
    nh.param("/move_base/controller_frequency", controller_freqency, 10.0);
    d_t_ = 1 / controller_freqency;

    initState();
    ROS_INFO("SFM planner initialized!");
  }
  else
  {
    ROS_WARN("SFM planner has already been initialized.");
  }
}

void SfmPlanner::initState()
{
  if (!odom_flag_)
  {
    ROS_ERROR("Odom not received!");
    return;
  }

  nav_msgs::Odometry agent_odom = other_odoms_[agent_id_ - 1];
  agent_.id = agent_id_ - 1;
  agent_.position.set(agent_odom.pose.pose.position.x, agent_odom.pose.pose.position.y);
  agent_.yaw = utils::Angle::fromRadian(tf2::getYaw(agent_odom.pose.pose.orientation));
  agent_.velocity.set(agent_odom.twist.twist.linear.x, agent_odom.twist.twist.linear.y);
  agent_.linearVelocity = agent_.velocity.norm();
  agent_.angularVelocity = agent_odom.twist.twist.angular.z;
}

void SfmPlanner::odometryCallback(const nav_msgs::OdometryConstPtr& msg, int agent_id)
{
  if (!odom_flag_)
    odom_flag_ = true;

  other_odoms_[agent_id - 1] = *msg;
}

bool SfmPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized");
    return false;
  }

  ROS_INFO("Got new plan");

  if (first_plan_ || goal_.center.getX() != orig_global_plan.back().pose.position.x ||
      goal_.center.getY() != orig_global_plan.back().pose.position.y)
  {
    first_plan_ = false;
    goal_.center.set(orig_global_plan.back().pose.position.x, orig_global_plan.back().pose.position.y);
    goal_.radius = goal_dist_tol_;
    agent_.goals.clear();
    agent_.goals.push_back(goal_);
    goal_reached_ = false;
  }

  return true;
}

bool SfmPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized");
    return false;
  }

  nav_msgs::Odometry agent_odom = other_odoms_[agent_id_ - 1];
  utils::Vector2d curr_pose(agent_odom.pose.pose.position.x, agent_odom.pose.pose.position.y);
  if ((curr_pose - goal_.center).norm() < goal_dist_tol_)
  {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    goal_reached_ = true;
  }
  else
  {
    // TODO: update closest obstacle
    // handleObstacles();

    // update pedestrian around
    handleAgents();

    // compute Social Forces
    sfm::SFM.computeForces(agent_, others_);

    // update model
    sfm::SFM.updatePosition(agent_, d_t_);

    cmd_vel.linear.x = linearRegularization(agent_odom, agent_.linearVelocity);
    cmd_vel.angular.z = angularRegularization(agent_odom, agent_.angularVelocity);
  }

  return true;
}

void SfmPlanner::handleAgents()
{
  if (!odom_flag_)
  {
    ROS_ERROR("Odom not received!");
    return;
  }

  for (int i = 0; i < agent_number_; ++i)
  {
    sfm::Agent& other = others_[i];
    nav_msgs::Odometry other_odom = other_odoms_[i];
    other.id = i;
    if (i == agent_.id)
      continue;

    other.position.set(other_odom.pose.pose.position.x, other_odom.pose.pose.position.y);
    other.yaw = utils::Angle::fromRadian(tf2::getYaw(other_odom.pose.pose.orientation));
    other.radius = agent_.radius;
    other.velocity.set(other_odom.twist.twist.linear.x, other_odom.twist.twist.linear.y);
    other.linearVelocity = other.velocity.norm();
    other.angularVelocity = other_odom.twist.twist.angular.z;
  }
}

bool SfmPlanner::isGoalReached()
{
  if (!initialized_)
  {
    ROS_ERROR("ORCA planner has not been initialized");
    return false;
  }

  if (goal_reached_)
  {
    ROS_INFO("GOAL Reached!");
    return true;
  }

  // todo

  return false;
}

}  // namespace sfm_planner