/**
 * @file robot_controller.cpp
 * @brief ROS通信管理类实现
 */

#include "ros/robot_controller.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetMap.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <fstream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

RobotController::RobotController(QObject* parent)
    : QObject(parent)
    , is_navigating_(false)
    , is_mapping_(false)
    , navigation_mode_(0)
{
    // 从参数服务器读取配置
    ros::NodeHandle private_nh("~");
    std::string robot_name, cmd_vel_topic, odom_topic, scan_topic, map_topic, initial_pose_topic;
    
    private_nh.param<std::string>("robot_name", robot_name, "turtlebot3");
    private_nh.param<std::string>("cmd_vel_topic", cmd_vel_topic, "/cmd_vel");
    private_nh.param<std::string>("odom_topic", odom_topic, "/odom");
    private_nh.param<std::string>("scan_topic", scan_topic, "/scan");
    private_nh.param<std::string>("map_topic", map_topic, "/map");
    private_nh.param<std::string>("initial_pose_topic", initial_pose_topic, "/initialpose");
    
    private_nh.param<double>("max_linear_velocity", max_linear_velocity_, 0.22);
    private_nh.param<double>("max_angular_velocity", max_angular_velocity_, 2.84);

    // 创建发布器
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
    initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(initial_pose_topic, 1);

    // 创建订阅器
    map_sub_ = nh_.subscribe(map_topic, 1, &RobotController::mapCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic, 1, &RobotController::odomCallback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 1, &RobotController::scanCallback, this);

    // 创建action客户端
    move_base_client_ = std::make_shared<MoveBaseClient>("move_base", true);
    
    // 等待action服务器
    ROS_INFO("Waiting for move_base action server...");
    move_base_client_->waitForServer();
    ROS_INFO("Connected to move_base action server");
}

RobotController::~RobotController()
{
    // 关闭所有发布器和订阅器
    cmd_vel_pub_.shutdown();
    initial_pose_pub_.shutdown();
    map_sub_.shutdown();
    odom_sub_.shutdown();
    scan_sub_.shutdown();
}

bool RobotController::setNavigationGoal(double x, double y, double theta)
{
    if (!move_base_client_->isServerConnected()) {
        ROS_ERROR("Navigation server is not connected");
        return false;
    }

    // 如果正在建图，不允许导航
    if (is_mapping_) {
        ROS_WARN("Cannot navigate while mapping is active");
        return false;
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    
    // 将偏航角转换为四元数
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    goal.target_pose.pose.orientation.w = q.w();
    goal.target_pose.pose.orientation.x = q.x();
    goal.target_pose.pose.orientation.y = q.y();
    goal.target_pose.pose.orientation.z = q.z();

    // 根据导航模式设置参数
    switch (navigation_mode_) {
        case 1: // 快速模式
            nh_.setParam("/move_base/DWAPlannerROS/max_vel_x", max_linear_velocity_);
            nh_.setParam("/move_base/DWAPlannerROS/max_vel_theta", max_angular_velocity_);
            break;
        case 2: // 精确模式
            nh_.setParam("/move_base/DWAPlannerROS/max_vel_x", max_linear_velocity_ * 0.5);
            nh_.setParam("/move_base/DWAPlannerROS/max_vel_theta", max_angular_velocity_ * 0.5);
            break;
        default: // 默认模式
            nh_.setParam("/move_base/DWAPlannerROS/max_vel_x", max_linear_velocity_ * 0.8);
            nh_.setParam("/move_base/DWAPlannerROS/max_vel_theta", max_angular_velocity_ * 0.8);
            break;
    }

    move_base_client_->sendGoal(goal,
        boost::bind(&RobotController::doneCallback, this, _1, _2),
        boost::bind(&RobotController::activeCallback, this),
        boost::bind(&RobotController::feedbackCallback, this, _1));

    is_navigating_ = true;
    ROS_INFO("Navigation goal set to: x=%.2f, y=%.2f, theta=%.2f", x, y, theta);
    return true;
}

void RobotController::cancelNavigation()
{
    if (is_navigating_ && move_base_client_->isServerConnected()) {
        move_base_client_->cancelAllGoals();
        is_navigating_ = false;
        ROS_INFO("Navigation cancelled");
    }
}

void RobotController::doneCallback(const actionlib::SimpleClientGoalState& state,
                                 const move_base_msgs::MoveBaseResultConstPtr& result)
{
    is_navigating_ = false;
    ROS_INFO("Navigation finished with state: %s", state.toString().c_str());
}

void RobotController::activeCallback()
{
    ROS_INFO("Goal just went active");
}

void RobotController::feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
    // 可以在这里处理导航反馈
}

bool RobotController::isNavigating() const
{
    return is_navigating_;
}

bool RobotController::setNavigationMode(int mode)
{
    if (mode >= 0 && mode <= 2) {  // 支持3种模式：0-默认，1-快速，2-精确
        navigation_mode_ = mode;
        return true;
    }
    return false;
}

int RobotController::getNavigationMode() const
{
    return navigation_mode_;
}

bool RobotController::startMapping()
{
    if (!is_mapping_) {
        // 停止导航服务（如果正在运行）
        if (is_navigating_) {
            cancelNavigation();
        }

        // 关闭AMCL（如果正在运行）
        ros::ServiceClient amcl_client = nh_.serviceClient<std_srvs::Empty>("/global_localization/kill");
        std_srvs::Empty srv;
        if (amcl_client.exists()) {
            amcl_client.call(srv);
        }

        // 启动SLAM
        ros::NodeHandle slam_nh;
        slam_nh.setParam("/slam_gmapping/base_frame", "base_footprint");
        slam_nh.setParam("/slam_gmapping/odom_frame", "odom");
        slam_nh.setParam("/slam_gmapping/map_frame", "map");

        // 等待SLAM服务启动
        ros::service::waitForService("/slam_gmapping/dynamic_map", ros::Duration(5.0));
        is_mapping_ = true;
        ROS_INFO("SLAM started successfully");
        return true;
    }
    return false;
}

bool RobotController::stopMapping()
{
    if (is_mapping_) {
        // 停止SLAM
        ros::ServiceClient slam_client = nh_.serviceClient<std_srvs::Empty>("/slam_gmapping/stop");
        std_srvs::Empty srv;
        if (slam_client.call(srv)) {
            is_mapping_ = false;
            ROS_INFO("SLAM stopped successfully");
            return true;
        } else {
            ROS_ERROR("Failed to stop SLAM");
        }
    }
    return false;
}

bool RobotController::saveMap(const std::string& filename)
{
    if (!is_mapping_ && !filename.empty()) {
        // 使用 map_saver 命令行工具保存地图
        std::string cmd = "rosrun map_server map_saver -f " + filename;
        if (system(cmd.c_str()) == 0) {
            ROS_INFO("Map saved successfully to: %s", filename.c_str());
            return true;
        } else {
            ROS_ERROR("Failed to save map to: %s", filename.c_str());
        }
    }
    return false;
}

bool RobotController::loadMap(const std::string& filename)
{
    if (!filename.empty()) {
        // 停止SLAM（如果正在运行）
        if (is_mapping_) {
            stopMapping();
        }

        // 关闭当前的map_server（如果有）
        system("rosnode kill /map_server");
        ros::Duration(1.0).sleep();  // 等待节点关闭

        // 启动新的map_server
        std::string cmd = "rosrun map_server map_server " + filename + " __name:=map_server &";
        if (system(cmd.c_str()) == 0) {
            // 等待地图加载
            ros::Duration(2.0).sleep();
            ROS_INFO("Map loaded successfully from: %s", filename.c_str());
            return true;
        }
        ROS_ERROR("Failed to load map from: %s", filename.c_str());
    }
    return false;
}

bool RobotController::setInitialPose(double x, double y, double theta)
{
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    
    pose.pose.pose.position.x = x;
    pose.pose.pose.position.y = y;
    pose.pose.pose.position.z = 0.0;
    
    // 将偏航角转换为四元数
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    pose.pose.pose.orientation.w = q.w();
    pose.pose.pose.orientation.x = q.x();
    pose.pose.pose.orientation.y = q.y();
    pose.pose.pose.orientation.z = q.z();

    // 设置协方差矩阵（表示初始位置的不确定性）
    for (int i = 0; i < 36; i++) {
        pose.pose.covariance[i] = 0.0;
    }
    pose.pose.covariance[0] = 0.25;  // x方差
    pose.pose.covariance[7] = 0.25;  // y方差
    pose.pose.covariance[35] = 0.06853; // theta方差
    
    initial_pose_pub_.publish(pose);
    ROS_INFO("Initial pose set to: x=%.2f, y=%.2f, theta=%.2f", x, y, theta);
    return true;
}

bool RobotController::updateCostmap()
{
    // 清除代价地图
    ros::ServiceClient clear_costmaps_client = 
        nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    std_srvs::Empty srv;
    
    if (clear_costmaps_client.call(srv)) {
        ROS_INFO("Costmaps cleared successfully");
        return true;
    } else {
        ROS_ERROR("Failed to clear costmaps");
        return false;
    }
}

void RobotController::publishVelocity(double linear, double angular)
{
    // 限制速度在最大值范围内
    linear = std::max(-max_linear_velocity_, std::min(linear, max_linear_velocity_));
    angular = std::max(-max_angular_velocity_, std::min(angular, max_angular_velocity_));

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = linear;
    cmd_vel.angular.z = angular;
    cmd_vel_pub_.publish(cmd_vel);
}

void RobotController::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    auto map = std::make_shared<nav_msgs::OccupancyGrid>(*msg);
    emit mapUpdated(map);
}

void RobotController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    auto odom = std::make_shared<nav_msgs::Odometry>(*msg);
    emit odomUpdated(odom);
}

void RobotController::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    auto scan = std::make_shared<sensor_msgs::LaserScan>(*msg);
    emit scanUpdated(scan);
} 