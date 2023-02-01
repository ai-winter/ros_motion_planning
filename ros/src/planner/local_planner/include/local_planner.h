#ifndef LOCAL_PLANNER_H_
#define LOCAL_PLANNER_H_

#include <nav_core/base_local_planner.h>
#include <base_local_planner/odometry_helper_ros.h>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

using namespace std;

namespace local_planner
{
    class LocalPlanner : public nav_core::BaseLocalPlanner
    {
    public:
        LocalPlanner();
        LocalPlanner(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros);

        ~LocalPlanner();

        void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros);

        bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);

        bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

        bool isGoalReached();

        double getGoalPositionDistance(const geometry_msgs::PoseStamped &global_pose, double goal_x, double goal_y);

        std::vector<double> getEulerAngles(geometry_msgs::PoseStamped &Pose);

        double LinearPIDController(nav_msgs::Odometry &base_odometry, double next_t_x, double next_t_y);

        double AngularPIDController(nav_msgs::Odometry &base_odometry, double target_th_w, double robot_orien);

        // void controller(double x, double y, double theta, double x_d, double y_d, geometry_msgs::Twist &cmd_vel);

        void rangeAngle(double &angle);

    private:
        void robotStops()
        {
            goal_reached_ = true;
            ROS_INFO("Robot will stop.");
        }

        void getTransformedPosition(geometry_msgs::PoseStamped &pose, double *x, double *y, double *theta)
        {
            geometry_msgs::PoseStamped ps;
            pose.header.stamp = ros::Time(0);

            // tf_->transformPose(base_frame_, pose, ps);
            tf_->transform(pose, ps, base_frame_);

            *x = ps.pose.position.x;
            *y = ps.pose.position.y;

            // theta = tf::getYaw(ps.pose.orientation);
            tf2::Quaternion q(ps.pose.orientation.x, ps.pose.orientation.y,
                              ps.pose.orientation.z, ps.pose.orientation.w);
            *theta = q.getAngle();
        }

        costmap_2d::Costmap2DROS *costmap_ros_;
        tf2_ros::Buffer *tf_;
        bool initialized_, goal_reached_, rotating_to_goal_;
        std::vector<geometry_msgs::PoseStamped> global_plan_;
        int plan_index_, last_plan_index_;
        double error_lin_, error_ang_;
        double integral_lin_, integral_ang_;
        std::vector<double> final_orientation;
        double robot_curr_orien;
        // tf::Vector3 robot_curr_pose;
        double robot_curr_pose[3];
        double p_window_, o_window_;
        double p_precision_, o_precision_;
        double d_t_;
        double max_vel_lin_, min_vel_lin_, max_incr_lin_;
        double max_vel_ang_, min_vel_ang_, max_incr_ang_;
        double k_p_lin_, k_i_lin_, k_d_lin_;
        double k_p_ang_, k_i_ang_, k_d_ang_;
        // double k_, l_;
        std::string base_frame_;
        base_local_planner::OdometryHelperRos *odom_helper_;
        ros::Publisher target_pose_pub_, curr_pose_pub;
        ros::Subscriber emergency_stop_sub_;
    };
};

#endif