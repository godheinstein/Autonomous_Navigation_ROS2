#include "ee4308_turtle/controller.hpp"
#include "angles/angles.h"

namespace ee4308::turtle
{
    void Controller::cleanup() { RCLCPP_INFO_STREAM(node_->get_logger(), "Cleaning up plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::activate() { RCLCPP_INFO_STREAM(node_->get_logger(), "Activating plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::deactivate() { RCLCPP_INFO_STREAM(node_->get_logger(), "Deactivating plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::setSpeedLimit(const double &speed_limit, const bool &percentage)
    {
        (void)speed_limit;
        (void)percentage;
    }

    void Controller::setPlan(const nav_msgs::msg::Path &path) { global_plan_ = path; }

    // ====================================== LAB 1, PROJ 1 ====================================================

    void Controller::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        (void)costmap_ros;

        // initialize states / variables
        node_ = parent.lock(); // this class is not a node_. It is instantiated as part of a node_ `parent`.
        tf_ = tf;
        plugin_name_ = name;

        // initialize parameters
        initParam(node_, plugin_name_ + ".desired_linear_vel", desired_linear_vel_, 0.18);
        initParam(node_, plugin_name_ + ".desired_lookahead_dist", desired_lookahead_dist_, 0.6);
        initParam(node_, plugin_name_ + ".max_angular_vel", max_angular_vel_, 1.0);
        initParam(node_, plugin_name_ + ".max_linear_vel", max_linear_vel_, 0.22);
        initParam(node_, plugin_name_ + ".xy_goal_thres", xy_goal_thres_, 0.05);
        initParam(node_, plugin_name_ + ".yaw_goal_thres", yaw_goal_thres_, 0.25);
        initParam(node_, plugin_name_ + ".curvature_threshold", curvature_threshold_, 2.5);
        initParam(node_, plugin_name_ + ".lookahead_gain", lookahead_gain_, 1.5);
        initParam(node_, plugin_name_ + ".proximity_threshold", proximity_threshold_, 0.15);

        // initialize topics
        sub_scan_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(),
            std::bind(&Controller::laser_callback, this, std::placeholders::_1)); 
    }

    void Controller::laser_callback(sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        scan_ranges_ = msg->ranges;
    }

    double getClosestObstacleDistance(const std::vector<float> &ranges) {
        double min_range = std::numeric_limits<double>::max();
        for (const auto &range : ranges) {
            if (range < min_range) {
                min_range = range;
            }
        }
        return min_range;
    }

    geometry_msgs::msg::TwistStamped Controller::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::Twist &velocity,
        nav2_core::GoalChecker *goal_checker)
    {
        (void)velocity;     // not used
        (void)goal_checker; // not used

        // check if path exists
        if (global_plan_.poses.empty())
        {
            RCLCPP_WARN_STREAM(node_->get_logger(), "Global plan is empty!");
            return writeCmdVel(0, 0);
        }

        // fetch the goal pose
        geometry_msgs::msg::PoseStamped goal_pose = global_plan_.poses.back();
        double dist_to_waypoint = sqrt(pow(goal_pose.pose.position.x - pose.pose.position.x, 2) + pow(goal_pose.pose.position.y - pose.pose.position.y, 2));
        // check if the current pose is close to the goal
        if (dist_to_waypoint < xy_goal_thres_)
        {
            RCLCPP_WARN_STREAM(node_->get_logger(), "within threshold");
            double ang_diff = limitAngle(getYawFromQuaternion(goal_pose.pose.orientation) - getYawFromQuaternion(pose.pose.orientation));
            if (fabs(ang_diff) < yaw_goal_thres_)
            {
                // stop the robot
                return writeCmdVel(0.0, 0.0);
            }
            else
            {
                // rotate in place
                return writeCmdVel(0.0, sgn(ang_diff) * max_angular_vel_);
            }
        }

        // getting distances from robot for all path points
        std::vector<double> distances;
        for (auto pose_path : global_plan_.poses)
        {
            double del_x_path_to_robot = pose_path.pose.position.x - pose.pose.position.x; // path - robot
            double del_y_path_to_robot = pose_path.pose.position.y - pose.pose.position.y; // path - robot
            
            distances.push_back(sqrt(pow(del_x_path_to_robot, 2) + pow(del_y_path_to_robot, 2)));
        }

        // finding the index for minimum distance
        auto min_it = std::min_element(distances.begin(), distances.end());
        size_t min_index = std::distance(distances.begin(), min_it);
        bool goalislookahead = true;
        // getting lookahead point
        auto i = min_index; // start from the closest point and go forward
        while (i < global_plan_.poses.size()) 
        {
            auto pose_i = global_plan_.poses[i];
            double del_x_path = pose_i.pose.position.x - pose.pose.position.x; // path - robot
            double del_y_path = pose_i.pose.position.y - pose.pose.position.y; // path - robot

            if (sqrt(pow(del_x_path,2) + pow(del_y_path,2)) > desired_lookahead_dist_)
            {
                goalislookahead = false;
                break;
            }
        i++;
        }

        // defining lookahead point for further calculations
        auto lookahead_point = global_plan_.poses[i];
        // transform the lookahead point to the robot frame
        double phi = getYawFromQuaternion(pose.pose.orientation);
        double dx = lookahead_point.pose.position.x - pose.pose.position.x;
        double dy = lookahead_point.pose.position.y - pose.pose.position.y;
        double x_prime = dx * std::cos(phi) + dy * std::sin(phi);
        double y_prime = -dx * std::sin(phi) + dy * std::cos(phi);

        // calculate the curvature c
        double L = std::hypot(x_prime, y_prime);
        double curvature = (2.0 * y_prime) / (L * L);

        // calculate the angular velocity w
        double linear_velocity = desired_linear_vel_;

        // scan_ranges_ contains the laser scan data
        double d_o = getClosestObstacleDistance(scan_ranges_);

        if (curvature_threshold_ < fabs(curvature))
        {
            linear_velocity = (linear_velocity * curvature_threshold_) / fabs(curvature);
            std::cout << "curvature_threshold_: " << (curvature_threshold_) / fabs(curvature) << std::endl;
        }

        if (fabs(d_o) < proximity_threshold_)
        {
            linear_velocity = (linear_velocity * fabs(d_o)) / proximity_threshold_;
            std::cout << "curvature_threshold_: " << (fabs(d_o)) / proximity_threshold_ << std::endl;
        }

        double angular_velocity = linear_velocity * curvature;

        if (std::fabs(linear_velocity) > max_linear_vel_)
        {
            linear_velocity = std::copysign(max_linear_vel_, linear_velocity);
            angular_velocity = linear_velocity * curvature;
        }

        if (std::fabs(angular_velocity) > max_angular_vel_)
        {
            angular_velocity = std::copysign(max_angular_vel_, angular_velocity);
            linear_velocity = angular_velocity / curvature;
        }

        desired_lookahead_dist_ = (lookahead_gain_ * linear_velocity + 0.1);
        double lookahead_ang = std::atan2(y_prime, x_prime);

        if (fabs(lookahead_ang) > 3.14/6 && !goalislookahead)
        {
            RCLCPP_WARN_STREAM(node_->get_logger(), "lookahead_ang: " << lookahead_ang);
            return writeCmdVel(0, sgn(lookahead_ang) * max_angular_vel_);
        }

        return writeCmdVel(linear_velocity, angular_velocity);
    }

    geometry_msgs::msg::TwistStamped Controller::writeCmdVel(double linear_vel, double angular_vel)
    {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.twist.linear.x = linear_vel;
        cmd_vel.twist.angular.z = angular_vel;
        return cmd_vel;
    }
}

PLUGINLIB_EXPORT_CLASS(ee4308::turtle::Controller, nav2_core::Controller)