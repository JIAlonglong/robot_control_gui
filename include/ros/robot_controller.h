/**
 * @file robot_controller.h
 * @brief ROS通信管理类
 */

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/BatteryState.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

class RobotController {
public:
    explicit RobotController(ros::NodeHandle& nh);

    // 发布速度命令
    void publishVelocity(double linear_x, double angular_z);

    // 获取机器人状态
    double getBatteryLevel() const { return battery_level_; }
    double getCurrentLinearSpeed() const { return current_linear_speed_; }
    double getCurrentAngularSpeed() const { return current_angular_speed_; }

    // 获取最新的传感器数据
    nav_msgs::Odometry::ConstPtr getLatestOdom() const { return latest_odom_; }
    sensor_msgs::LaserScan::ConstPtr getLatestScan() const { return latest_scan_; }
    nav_msgs::OccupancyGrid::ConstPtr getLatestMap() const { return latest_map_; }

private:
    // ROS节点句柄
    ros::NodeHandle& nh_;

    // 发布器和订阅器
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber battery_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber scan_sub_;

    // 回调函数
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    // 状态变量
    double battery_level_;
    double current_linear_speed_;
    double current_angular_speed_;

    // 最新的传感器数据
    nav_msgs::Odometry::ConstPtr latest_odom_;
    sensor_msgs::LaserScan::ConstPtr latest_scan_;
    nav_msgs::OccupancyGrid::ConstPtr latest_map_;
};

#endif // ROBOT_CONTROLLER_H 