#ifndef LOCAL_PLANNER_H_
#define LOCAL_PLANNER_H_

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

using namespace std;

namespace local_planner
{
    class LocalPlanner : public nav_core::BaseLocalPlanner
    {
    public:
        LocalPlanner();
        LocalPlanner(std::string name, tf2_ros::Buffer *tf,
                     costmap_2d::Costmap2DROS *costmap_ros);

        ~LocalPlanner();

        void initialize(std::string name, tf2_ros::Buffer *tf,
                        costmap_2d::Costmap2DROS *costmap_ros);

        bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);

        bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

        bool isGoalReached();

    private:
        costmap_2d::Costmap2DROS *costmap_ros_;
        tf2_ros::Buffer *tf_;
        bool initialized_;
    };
};

#endif