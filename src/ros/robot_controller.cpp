/**
 * @file robot_controller.cpp
 * @brief ROS通信管理类实现
 */

#include "ros/robot_controller.h"

RobotController::RobotController(ros::NodeHandle& nh)
    : nh_(nh)
    , battery_level_(0.0)
    , current_linear_speed_(0.0)
    , current_angular_speed_(0.0)
{
    // 创建发布器和订阅器
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    odom_sub_ = nh_.subscribe("odom", 10, &RobotController::odomCallback, this);
    battery_sub_ = nh_.subscribe("battery_state", 10, &RobotController::batteryCallback, this);
    map_sub_ = nh_.subscribe("map", 1, &RobotController::mapCallback, this);
    scan_sub_ = nh_.subscribe("scan", 10, &RobotController::scanCallback, this);
}

void RobotController::publishVelocity(double linear_x, double angular_z)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = linear_x;
    cmd_vel.angular.z = angular_z;
    cmd_vel_pub_.publish(cmd_vel);
}

void RobotController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    latest_odom_ = msg;
    current_linear_speed_ = msg->twist.twist.linear.x;
    current_angular_speed_ = msg->twist.twist.angular.z;
}

void RobotController::batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
    battery_level_ = msg->percentage;
}

void RobotController::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    latest_map_ = msg;
}

void RobotController::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    latest_scan_ = msg;
} 