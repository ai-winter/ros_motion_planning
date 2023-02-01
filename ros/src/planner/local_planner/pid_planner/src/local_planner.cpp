#include "local_planner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(local_planner::LocalPlanner, nav_core::BaseLocalPlanner)

namespace local_planner
{
    LocalPlanner::LocalPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

    LocalPlanner::LocalPlanner(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
        : costmap_ros_(NULL), tf_(NULL), initialized_(false)
    {
        initialize(name, tf, costmap_ros);
    }

    LocalPlanner::~LocalPlanner() {}

    void LocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized_)
        {
            tf_ = tf;
            costmap_ros_ = costmap_ros;
            initialized_ = true;

            // ******* TODO: improve to parameters **********
            // target
            p_window_ = 0.7;
            o_window_ = 0.7;
            // goal tolerance
            p_precision_ = 0.5;
            o_precision_ = 0.2;
            // linear
            max_vel_lin_ = 0.4;
            min_vel_lin_ = 0.0;
            max_incr_lin_ = 0.3;
            // angular
            max_vel_ang_ = 0.4;
            min_vel_ang_ = -0.4;
            max_incr_ang_ = 0.25;
            // pid controller params
            k_p_lin_ = 2.00;
            k_i_lin_ = 0.00;
            k_d_lin_ = 0.00;

            k_p_ang_ = 2.00;
            k_i_ang_ = 0.00;
            k_d_ang_ = 0.00;

            // k_ = 1.0;
            // l_ = 0.5;
            // **********************************************

            ros::NodeHandle nh = ros::NodeHandle("~/" + name);
            odom_helper_ = new base_local_planner::OdometryHelperRos("/odom");
            target_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/target_pose", 10);
            curr_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);

            base_frame_ = "base_link";
            plan_index_ = 0;
            last_plan_index_ = 0;
            robot_curr_pose[0] = 0;
            robot_curr_pose[1] = 0;
            robot_curr_pose[2] = 0;
            robot_curr_orien = 0;

            double controller_freqency;
            nh.param("/move_base/controller_frequency", controller_freqency, 10.0);
            d_t_ = 1 / controller_freqency;
            ROS_INFO("Local planner initialized!");
        }
        else
        {
            ROS_WARN("Local planner has already been initialized.");
        }
    }

    bool LocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }

        // set new plan
        global_plan_.clear();
        global_plan_ = orig_global_plan;

        // reset plan parameters
        plan_index_ = 0;
        goal_reached_ = false;

        // reset pid
        integral_lin_ = integral_ang_ = 0.0;
        error_lin_ = error_ang_ = 0.0;

        return true;
    }

    double LocalPlanner::LinearPIDController(nav_msgs::Odometry &base_odometry, double next_t_x, double next_t_y)
    {
        double vel_curr = hypot(base_odometry.twist.twist.linear.y, base_odometry.twist.twist.linear.x);

        // linear velocity of depends on the p_windows (direct relation)
        double vel_target = hypot(next_t_x, next_t_y) / d_t_;

        if (fabs(vel_target) > max_vel_lin_)
            vel_target = copysign(max_vel_lin_, vel_target);

        double err_vel = vel_target - vel_curr;

        integral_lin_ += err_vel * d_t_;
        double derivative_lin = (err_vel - error_lin_) / d_t_;
        double incr_lin = k_p_lin_ * err_vel + k_i_lin_ * integral_lin_ + k_d_lin_ * derivative_lin;
        error_lin_ = err_vel;

        if (fabs(incr_lin) > max_incr_lin_)
            incr_lin = copysign(max_incr_lin_, incr_lin);

        double x_velocity = vel_curr + incr_lin;
        if (fabs(x_velocity) > max_vel_lin_)
            x_velocity = copysign(max_vel_lin_, x_velocity);
        if (fabs(x_velocity) < min_vel_lin_)
            x_velocity = copysign(min_vel_lin_, x_velocity);

        return x_velocity;
    }

    double LocalPlanner::AngularPIDController(nav_msgs::Odometry &base_odometry, double target_th_w, double robot_orien)
    {
        double orien_err = target_th_w - robot_orien;
        rangeAngle(orien_err);

        double target_vel_ang = orien_err / d_t_;
        if (fabs(target_vel_ang) > max_vel_ang_)
            target_vel_ang = copysign(max_vel_ang_, target_vel_ang);

        double vel_ang = base_odometry.twist.twist.angular.z;
        double error_ang = target_vel_ang - vel_ang;
        integral_ang_ += error_ang * d_t_;
        double derivative_ang = (error_ang - error_ang_) / d_t_;
        double incr_ang = k_p_ang_ * error_ang + k_i_ang_ * integral_ang_ + k_d_ang_ * derivative_ang;
        error_ang_ = error_ang;

        if (fabs(incr_ang) > max_incr_ang_)
            incr_ang = copysign(max_incr_ang_, incr_ang);

        // double th_velocity = copysign(vel_ang + incr_ang, target_vel_ang);
        double th_velocity = vel_ang + incr_ang;
        if (fabs(th_velocity) > max_vel_ang_)
            th_velocity = copysign(max_vel_ang_, th_velocity);
        if (fabs(th_velocity) < min_vel_ang_)
            th_velocity = copysign(min_vel_ang_, th_velocity);

        ROS_INFO("orien_err: %lf, target_vel_ang: %lf, vel_ang: %lf, th_velocity: %lf", orien_err, target_vel_ang, vel_ang, th_velocity);

        return th_velocity;
    }

    // void LocalPlanner::controller(double x, double y, double theta, double x_d, double y_d, geometry_msgs::Twist &cmd_vel)
    // {
    //     Eigen::Matrix2d A;
    //     Eigen::Vector2d u;
    //     Eigen::Vector2d v_w;
    //     A << cos(theta), -l_ * sin(theta), sin(theta), l_ * cos(theta);
    //     u << k_ * (x_d - x), k_ * (y_d - y);
    //     v_w = A.ldlt().solve(u);

    //     cmd_vel.linear.x = v_w(0);
    //     cmd_vel.angular.z = v_w(1);
    // }

    bool LocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }

        if (goal_reached_)
        {
            ROS_ERROR("This planner goal reached without motion.");
            return true;
        }

        // next target
        geometry_msgs::PoseStamped target;
        geometry_msgs::PoseStamped curr_pose;

        double t_x, t_y, t_th;
        double x_vel = 0, th_vel = 0;
        double t_th_w = 0.0;

        while (plan_index_ < global_plan_.size())
        {
            target = global_plan_[plan_index_];
            int next_plan_index = min(((int)global_plan_.size()) - 1, plan_index_ + 1);
            t_th_w = atan2((global_plan_[next_plan_index].pose.position.y -
                            global_plan_[plan_index_].pose.position.y),
                           (global_plan_[next_plan_index].pose.position.x -
                            global_plan_[plan_index_].pose.position.x));
            rangeAngle(t_th_w);

            // tf::Quaternion th_target_quat = tf::createQuaternionFromYaw(t_th_w);
            tf2::Quaternion th_target_quat;
            th_target_quat.setRPY(0, 0, t_th_w);

            target.pose.orientation.x = th_target_quat[0];
            target.pose.orientation.y = th_target_quat[1];
            target.pose.orientation.z = th_target_quat[2];
            target.pose.orientation.w = th_target_quat[3];
            getTransformedPosition(target, &t_x, &t_y, &t_th);
            rangeAngle(t_th);

            if (hypot(t_x, t_y) > p_window_ || fabs(t_th) > o_window_)
                break;
            plan_index_++;
        }

        if (plan_index_ >= global_plan_.size() - 1)
        {
            getTransformedPosition(global_plan_.back(), &t_x, &t_y, &t_th);
        }

        // invoking robot pose and orientaiton
        geometry_msgs::PoseStamped global_pose;
        costmap_ros_->getRobotPose(global_pose);

        // robot_curr_pose = global_pose.getOrigin();
        robot_curr_pose[0] = global_pose.pose.position.x;
        robot_curr_pose[1] = global_pose.pose.position.y;
        robot_curr_pose[2] = global_pose.pose.position.z;

        // robot_curr_orien = tf::getYaw(global_pose.getRotation());
        tf2::Quaternion q(global_pose.pose.orientation.x, global_pose.pose.orientation.y,
                          global_pose.pose.orientation.z, global_pose.pose.orientation.w);
        robot_curr_orien = q.getAngle(); // [0, 2pi]
        rangeAngle(robot_curr_orien);

        // odometry observation - getting robot velocities in robot frame
        nav_msgs::Odometry base_odom;
        odom_helper_->getOdom(base_odom);

        // get final goal orientation - Quaternion to Euler
        final_orientation = getEulerAngles(global_plan_.back());
        rangeAngle(final_orientation[2]);

        // controller(robot_curr_pose[0], robot_curr_pose[1], robot_curr_orien, target.pose.position.x, target.pose.position.y, cmd_vel);

        if (getGoalPositionDistance(global_plan_.back(), robot_curr_pose[0], robot_curr_pose[1]) <= p_precision_)
        {
            // check to see if the goal orientation has been reached
            if (fabs(final_orientation[2] - robot_curr_orien) <= o_precision_)
            {
                // set the velocity command to zero
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z = 0.0;
                rotating_to_goal_ = false;
                goal_reached_ = true;
            }
            else
            {
                th_vel = AngularPIDController(base_odom, final_orientation[2], robot_curr_orien);
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z = th_vel;
            }
        }
        else
        {
            //********Linear velocity controller-PID*************
            x_vel = LinearPIDController(base_odom, t_x, t_y);

            //********Angular velocity controller-PID************
            if (plan_index_ >= global_plan_.size() - 5)
            {
                t_th_w = final_orientation[2];
            }
            th_vel = AngularPIDController(base_odom, t_th_w, robot_curr_orien);

            cmd_vel.linear.x = x_vel;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = th_vel;
        }

        // publish next target pose
        target.header.frame_id = "/map";
        target.header.stamp = ros::Time::now();
        target_pose_pub_.publish(target);

        // publish robot pose
        curr_pose.header.frame_id = "/map";
        curr_pose.header.stamp = ros::Time::now();

        // tf::Quaternion curr_orien_quat = tf::createQuaternionFromYaw(robot_curr_orien);
        tf2::Quaternion curr_orien_quat;
        curr_orien_quat.setRPY(0, 0, robot_curr_orien);
        curr_pose.pose.position.x = robot_curr_pose[0];
        curr_pose.pose.position.y = robot_curr_pose[1];
        curr_pose.pose.position.z = robot_curr_pose[2];
        curr_pose.pose.orientation.x = curr_orien_quat[0];
        curr_pose.pose.orientation.y = curr_orien_quat[1];
        curr_pose.pose.orientation.z = curr_orien_quat[2];
        curr_pose.pose.orientation.w = curr_orien_quat[3];
        curr_pose_pub.publish(curr_pose);

        return true;
    }

    bool LocalPlanner::isGoalReached()
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }

        if (goal_reached_)
        {
            ROS_INFO("This planner goal reached...");
            return true;
        }

        if (plan_index_ >= global_plan_.size() - 1)
        {
            if (fabs(final_orientation[2] - robot_curr_orien) <= o_precision_ &&
                getGoalPositionDistance(global_plan_.back(), robot_curr_pose[0], robot_curr_pose[1]) <= p_precision_)
            {
                goal_reached_ = true;
                robotStops();
                ROS_INFO("Goal has been reached!");
            }
        }
        return goal_reached_;
    }

    double LocalPlanner::getGoalPositionDistance(const geometry_msgs::PoseStamped &global_pose, double goal_x, double goal_y)
    {
        return hypot(goal_x - global_pose.pose.position.x,
                     goal_y - global_pose.pose.position.y);
    }

    std::vector<double> LocalPlanner::getEulerAngles(geometry_msgs::PoseStamped &Pose)
    {
        std::vector<double> EulerAngles;
        EulerAngles.resize(3, 0);

        // tf::Quaternion q(Pose.pose.orientation.x, Pose.pose.orientation.y,
        //                  Pose.pose.orientation.z, Pose.pose.orientation.w);
        tf2::Quaternion q(Pose.pose.orientation.x, Pose.pose.orientation.y,
                          Pose.pose.orientation.z, Pose.pose.orientation.w);

        // tf::Matrix3x3 m(q);
        tf2::Matrix3x3 m(q);

        m.getRPY(EulerAngles[0], EulerAngles[1], EulerAngles[2]);
        return EulerAngles;
    }

    void LocalPlanner::rangeAngle(double &angle)
    {
        if (angle > M_PI)
            angle -= (2 * M_PI);
        if (angle < -M_PI)
            angle += (2 * M_PI);
    }
}