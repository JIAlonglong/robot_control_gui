/*
 * Copyright (c) 2025 JIAlonglong
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file robot_controller.cpp
 * @brief ROS通信管理类实现
 */

#include "ros/robot_controller.h"  // 这里的路径是相对于include目录的
#include <ros/master.h>
#include <QDebug>
#include <tf2/utils.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/BatteryState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <interactive_markers/interactive_marker_server.h>
#include <amcl/AMCLConfig.h>
#include <rviz/visualization_manager.h>
#include <rviz/tool_manager.h>
#include <rviz/tool.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <angles/angles.h>

RobotController::RobotController(QObject* parent)
    : QObject(parent)
    , nh_()
    , is_localizing_(false)
    , is_navigating_(false)
    , is_mapping_(false)
    , localization_progress_(0.0)
    , mapping_progress_(0.0)
    , yaw_tolerance_(0.1)
    , auto_localization_radius_(0.15)  // 设置自动定位的运动半径为15厘米
    , is_connected_(false)
    , master_uri_()
    , hostname_()
{
    // 设置发布者和订阅者
    setupPublishers();
    setupSubscribers();
    
    // 初始化服务客户端
    global_localization_client_ = nh_.serviceClient<std_srvs::Empty>("/global_localization");
    clear_costmaps_client_ = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    
    // 初始化 action 客户端
    move_base_client_ = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
    
    // 等待 action 服务器启动,最多等待30秒,每5秒重试一次
    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(30.0);  // 总超时时间30秒
    bool server_connected = false;
    
    while (ros::ok() && !server_connected && (ros::Time::now() - start_time) < timeout) {
        ROS_INFO("Waiting for move_base action server...");
        server_connected = move_base_client_->waitForServer(ros::Duration(5.0));
        if (!server_connected) {
            ROS_WARN("Move base action server not available, retrying...");
            emit navigationStatusChanged("等待导航服务器启动...");
        }
    }
    
    if (!server_connected) {
        ROS_ERROR("Failed to connect to move_base action server after 30 seconds");
        emit navigationStatusChanged("无法连接到导航服务器！");
        is_initialized_ = false;
        return;
    }
    
    ROS_INFO("Successfully connected to move_base action server");
    emit navigationStatusChanged("导航服务器连接成功");
    
    // 初始化可视化标记发布器
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/localization_markers", 1);
    
    // 标记为已初始化
    is_initialized_ = true;
}

RobotController::~RobotController()
{
    // 停止机器人
    stop();
    
    // 清理资源
    cleanup();
}

void RobotController::cleanup()
{
    // 停止所有活动的目标
    if (move_base_client_ && move_base_client_->isServerConnected()) {
        move_base_client_->cancelAllGoals();
    }
    
    // 停止自动定位
    if (is_localizing_) {
        is_localizing_ = false;
        localization_monitor_timer_.stop();
    }
    
    // 关闭所有发布者和订阅者
    if (cmd_vel_pub_) cmd_vel_pub_.shutdown();
    if (initial_pose_pub_) initial_pose_pub_.shutdown();
    if (global_localization_pub_) global_localization_pub_.shutdown();
    if (marker_pub_) marker_pub_.shutdown();
    if (tool_manager_pub_) tool_manager_pub_.shutdown();
    
    if (odom_sub_) odom_sub_.shutdown();
    if (battery_sub_) battery_sub_.shutdown();
    if (diagnostics_sub_) diagnostics_sub_.shutdown();
    if (scan_sub_) scan_sub_.shutdown();
    if (amcl_pose_sub_) amcl_pose_sub_.shutdown();
    if (map_sub_) map_sub_.shutdown();
    if (initial_pose_sub_) initial_pose_sub_.shutdown();
    if (goal_sub_) goal_sub_.shutdown();
    
    // 关闭服务客户端
    if (global_localization_client_) global_localization_client_.shutdown();
    if (clear_costmaps_client_) clear_costmaps_client_.shutdown();
}

void RobotController::setupPublishers()
{
    try {
        // 速度控制发布者
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        
        // 初始位姿发布者
        initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
        
        // 全局定位发布者
        global_localization_pub_ = nh_.advertise<std_msgs::Bool>("/global_localization", 1);
        
        // 可视化标记发布者
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/localization_markers", 1);
        
        // 工具管理器发布者
        tool_manager_pub_ = nh_.advertise<std_msgs::String>("/rviz/current_tool", 1);
        
        // 目标点发布者
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
        
        ROS_INFO("All publishers initialized successfully");
    } catch (const ros::Exception& e) {
        ROS_ERROR("Failed to initialize publishers: %s", e.what());
        throw;
    }
}

void RobotController::setupSubscribers()
{
    try {
        // 订阅导航目标点
        goal_sub_ = nh_.subscribe("/move_base_simple/goal", 10,
            &RobotController::goalCallback, this);
        ROS_INFO("Subscribed to /move_base_simple/goal");

        // 订阅初始位姿
        initial_pose_sub_ = nh_.subscribe("/initialpose", 10,
            &RobotController::initialPoseCallback, this);
        ROS_INFO("Subscribed to /initialpose");

        // 订阅里程计数据
        odom_sub_ = nh_.subscribe("odom", 10,
            &RobotController::odomCallback, this);
        ROS_INFO("Subscribed to odom");

        // 订阅激光扫描数据
        scan_sub_ = nh_.subscribe("scan", 10,
            &RobotController::scanCallback, this);
        ROS_INFO("Subscribed to scan");

        // 订阅AMCL位姿
        amcl_pose_sub_ = nh_.subscribe("amcl_pose", 10,
            &RobotController::amclPoseCallback, this);
        ROS_INFO("Subscribed to amcl_pose");

        // 订阅电池状态
        battery_sub_ = nh_.subscribe("battery_state", 10,
            &RobotController::batteryCallback, this);
        ROS_INFO("Subscribed to battery_state");

        // 订阅诊断信息
        diagnostics_sub_ = nh_.subscribe("/diagnostics", 10,
            &RobotController::diagnosticsCallback, this);
        ROS_INFO("Subscribed to /diagnostics");

        // 订阅地图数据
        map_sub_ = nh_.subscribe("map", 1,
            &RobotController::mapCallback, this);
        ROS_INFO("Subscribed to map");

        ROS_INFO("All subscribers set up successfully");
    } catch (const ros::Exception& e) {
        ROS_ERROR("Failed to set up subscribers: %s", e.what());
        throw;
    }
}

void RobotController::publishVelocity(double linear, double angular)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = linear;
    cmd_vel.angular.z = angular;
    cmd_vel_pub_.publish(cmd_vel);
    
    // 更新当前速度
    linear_velocity_ = linear;
    angular_velocity_ = angular;
    
    // 发送速度更新信号
    emit velocityChanged(linear, angular);
}

void RobotController::setLinearVelocity(double linear)
{
    publishVelocity(linear, angular_velocity_);
}

void RobotController::setAngularVelocity(double angular)
{
    publishVelocity(linear_velocity_, angular);
}

void RobotController::stop()
{
    publishVelocity(0.0, 0.0);
}

void RobotController::setInitialPose(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
    if (!initial_pose_pub_) {
        ROS_ERROR("Initial pose publisher not initialized");
        return;
    }

    try {
        // 发布初始位姿
        initial_pose_pub_.publish(pose);
        ROS_INFO("Published initial pose");
        
        // 更新内部状态
        current_amcl_pose_ = pose;
        current_pose_ = pose.pose.pose;
        
        // 计算yaw角并发送状态更新
        double yaw = tf2::getYaw(pose.pose.pose.orientation) * 180.0 / M_PI;
        QString status_msg = QString("当前位置: (%1, %2) 朝向%3°")
            .arg(current_pose_.position.x, 0, 'f', 3)
            .arg(current_pose_.position.y, 0, 'f', 3)
            .arg(yaw, 0, 'f', 1);
        
        emit poseUpdated(pose.pose.pose);
        emit localizationStatusChanged(status_msg);
        
        // 清除代价地图
        std_srvs::Empty srv;
        if (clear_costmaps_client_.call(srv)) {
            ROS_INFO("Costmaps cleared after setting initial pose");
        }
        
        // 重置交互模式
        interaction_mode_ = InteractionMode::NONE;
        
        ROS_INFO("Initial pose set successfully");
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in setInitialPose: %s", e.what());
    }
}

void RobotController::startGlobalLocalization()
{
    if (!is_initialized_) {
        ROS_ERROR("RobotController not initialized");
        emit localizationStatusChanged("错误：RobotController未初始化");
        emit localizationStateChanged("未初始化");
        return;
    }

    std_srvs::Empty srv;
    if (global_localization_client_.call(srv)) {
        ROS_INFO("Global localization triggered");
        emit localizationStatusChanged("已触发全局定位");
    } else {
        ROS_WARN("Failed to trigger global localization");
        emit localizationStatusChanged("触发全局定位失败");
        emit localizationStateChanged("失败");
    }
}

void RobotController::cancelGlobalLocalization()
{
    if (is_initialized_) {
        ROS_INFO("Cancelling global localization");
        emit localizationStatusChanged("已取消全局定位");
        emit localizationStateChanged("已取消");
    }
}

void RobotController::startNavigation()
{
    if (!is_initialized_ || !move_base_client_) {
        ROS_ERROR("RobotController not initialized or move_base client not available");
        emit navigationStatusChanged("导航服务未初始化");
        return;
    }

    if (current_goal_.header.frame_id.empty()) {
        ROS_ERROR("No navigation goal set");
        emit navigationStatusChanged("请先设置导航目标点");
        return;
    }

    try {
        ROS_INFO("Starting navigation to goal: x=%.2f, y=%.2f", 
                 current_goal_.pose.position.x, 
                 current_goal_.pose.position.y);

        // 清除代价地图
        std_srvs::Empty srv;
        if (clear_costmaps_client_.call(srv)) {
            ROS_INFO("Costmaps cleared before navigation");
        }

        // 发布目标点到move_base_simple/goal话题
        if (goal_pub_) {
            // 多次发布以确保目标点被正确接收
            for (int i = 0; i < 3; i++) {
                goal_pub_.publish(current_goal_);
                ros::Duration(0.1).sleep();  // 等待0.1秒
            }
            ROS_INFO("Published goal to move_base_simple/goal multiple times");
        } else {
            ROS_ERROR("Goal publisher not initialized");
            emit navigationStatusChanged("错误：目标点发布器未初始化！");
            return;
        }

        // 创建并发送导航目标
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = current_goal_;
        
        // 设置回调函数
        move_base_client_->sendGoal(
            goal,
            boost::bind(&RobotController::navigationDoneCallback, this, _1, _2),
            boost::bind(&RobotController::navigationActiveCallback, this),
            boost::bind(&RobotController::navigationFeedbackCallback, this, _1)
        );

        is_navigating_ = true;
        emit navigationStateChanged("进行中");
        emit navigationStatusChanged("开始导航...");
        ROS_INFO("Navigation started");

    } catch (const std::exception& e) {
        ROS_ERROR("Exception in startNavigation: %s", e.what());
        emit navigationStatusChanged("启动导航失败！");
        is_navigating_ = false;
    }
}

void RobotController::pauseNavigation()
{
    if (!is_initialized_ || !move_base_client_) {
        ROS_ERROR("RobotController not initialized or move_base client not available");
        return;
    }

    if (is_navigating_) {
        move_base_client_->cancelGoal();
        is_navigating_ = false;
        emit navigationStateChanged("已暂停");
        emit navigationStatusChanged("导航已暂停");
    }
}

void RobotController::stopNavigation()
{
    if (!is_initialized_ || !move_base_client_) {
        ROS_ERROR("RobotController not initialized or move_base client not available");
        return;
    }

    if (is_navigating_) {
        move_base_client_->cancelGoal();
        is_navigating_ = false;
        emit navigationStateChanged("已停止");
        emit navigationStatusChanged("导航已停止");
    }
}

void RobotController::startMapping(const QString& method)
{
    if (!is_initialized_) {
        ROS_ERROR("RobotController not initialized");
        return;
    }

    try {
        current_mapping_method_ = method;  // 设置当前建图方法
        
        if (is_mapping_) {
            throw std::runtime_error("建图已在进行中");
        }

        is_mapping_ = true;
        mapping_progress_ = 0.0;

        // 根据不同的建图方法启动相应的节点
        QString launch_cmd;
        if (method == "Gmapping SLAM") {
            launch_cmd = "roslaunch robot_control_gui gmapping.launch";
        } else if (method == "Cartographer SLAM") {
            launch_cmd = "roslaunch robot_control_gui cartographer.launch";
        } else if (method == "Hector SLAM") {
            launch_cmd = "roslaunch robot_control_gui hector_slam.launch";
        } else {
            throw std::runtime_error("未知的建图方法");
        }

        // 启动建图节点
        if (system((launch_cmd + " &").toStdString().c_str()) != 0) {
            throw std::runtime_error("启动" + method.toStdString() + "失败");
        }

        // 等待地图话题出现
        ros::Duration(1.0).sleep();

        // 订阅建图进度和地图话题
        map_sub_ = nh_.subscribe("/map", 1, &RobotController::mapCallback, this);
        map_progress_sub_ = nh_.subscribe("/mapping_progress", 1, 
            &RobotController::handleMappingProgress, this);

        emit mappingStatusChanged(tr("建图已启动: %1").arg(method));
        ROS_INFO("Mapping started with method: %s", method.toStdString().c_str());
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to start mapping: %s", e.what());
        throw;
    }
}

void RobotController::stopMapping()
{
    if (!is_mapping_) return;

    // 停止建图节点
    if (current_mapping_method_ == "Gmapping SLAM") {
        system("rosnode kill /slam_gmapping");
    } else if (current_mapping_method_ == "Cartographer SLAM") {
        system("rosnode kill /cartographer_node");
    } else if (current_mapping_method_ == "Hector SLAM") {
        system("rosnode kill /hector_slam");
    }

    is_mapping_ = false;
    map_progress_sub_.shutdown();
    emit mappingStatusChanged(tr("建图已停止"));
}

void RobotController::saveMap(const QString& filename)
{
    if (!is_mapping_) return;

    std_srvs::Empty srv;
    if (map_saver_client_.call(srv)) {
        system(QString("rosrun map_server map_saver -f %1").arg(filename).toStdString().c_str());
        emit mappingStatusChanged(tr("地图已保存: %1").arg(filename));
    } else {
        emit mappingStatusChanged(tr("地图保存失败"));
    }
}

void RobotController::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    if (!is_mapping_) return;
    
    try {
        // 发送地图更新信号
        emit mapUpdated(*map);
        ROS_INFO_THROTTLE(1.0, "Map updated: size=%dx%d", map->info.width, map->info.height);
    } catch (const std::exception& e) {
        ROS_ERROR("Error in map callback: %s", e.what());
    }
}

void RobotController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // 更新机器人位姿和速度
    current_pose_ = msg->pose.pose;
    linear_velocity_ = msg->twist.twist.linear.x;
    angular_velocity_ = msg->twist.twist.angular.z;
    
    // 发送速度更新信号
    emit velocityChanged(linear_velocity_, angular_velocity_);
}

void RobotController::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // 更新激光扫描数据
    current_scan_ = *msg;
    emit laserScanUpdated(*msg);
}

void RobotController::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    current_amcl_pose_ = *msg;
    // 保存协方差数据用于监控定位质量
    for(int i = 0; i < 36; i++) {
        current_pose_cov_[i] = msg->pose.covariance[i];
    }
    
    // 计算yaw角（弧度转角度）
    double yaw = tf2::getYaw(msg->pose.pose.orientation) * 180.0 / M_PI;
    
    // 发送位姿更新信号
    emit poseUpdated(msg->pose.pose);
    emit localizationStatusChanged(QString("当前位置: (%1, %2) 朝向%3°")
        .arg(current_pose_.position.x, 0, 'f', 3)
        .arg(current_pose_.position.y, 0, 'f', 3)
        .arg(yaw, 0, 'f', 1));
}

void RobotController::batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
    // 更新电池状态
    battery_percentage_ = msg->percentage;
    battery_voltage_ = msg->voltage;
    battery_current_ = msg->current;
    battery_temperature_ = msg->temperature;
    
    // 发送电池状态更新信号
    emit batteryStateChanged(*msg);
}

void RobotController::diagnosticsCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg)
{
    // 更新诊断信息
    for (const auto& status : msg->status) {
        if (status.name == "motors") {
            motor_status_ = QString::fromStdString(status.message);
            emit diagnosticsUpdated(*msg);
            break;
        }
    }
}

void RobotController::setNavigationGoal(const geometry_msgs::PoseStamped& goal)
{
    try {
        ROS_INFO("Setting navigation goal");
        
        // 完整复制目标点信息
        current_goal_.header = goal.header;
        current_goal_.header.frame_id = "map";  // 确保frame_id正确
        current_goal_.header.stamp = ros::Time::now();
        current_goal_.pose = goal.pose;
        
        double yaw = tf2::getYaw(goal.pose.orientation) * 180.0 / M_PI;
        
        ROS_INFO("Goal set: x=%.3f, y=%.3f, yaw=%.3f",
                 current_goal_.pose.position.x,
                 current_goal_.pose.position.y,
                 yaw);
                 
        QString status_msg = QString("已设置导航目标: (%1, %2) 朝向%3°")
            .arg(current_goal_.pose.position.x, 0, 'f', 3)
            .arg(current_goal_.pose.position.y, 0, 'f', 3)
            .arg(yaw, 0, 'f', 1);
        
        // 发送目标点设置信号
        emit goalSet(current_goal_);
        emit navigationStatusChanged(status_msg);
        emit navigationStateChanged("就绪");
        
        ROS_INFO("Navigation goal is ready");
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in setNavigationGoal: %s", e.what());
        emit navigationStatusChanged("设置导航目标点失败！");
    }
}

void RobotController::loadMap(const QString& filename)
{
    // TODO: 实现地图加载功能
    ROS_INFO("Loading map from file: %s", filename.toStdString().c_str());
}

void RobotController::setYawTolerance(double value)
{
    yaw_tolerance_ = value;
    // TODO: 更新 move_base 参数
}

void RobotController::setInflationRadius(double value)
{
    inflation_radius_ = value;
    // TODO: 更新 move_base 参数
}

void RobotController::setTransformTolerance(double value)
{
    transform_tolerance_ = value;
    // TODO: 更新 move_base 参数
}

void RobotController::setPlannerFrequency(double value)
{
    planner_frequency_ = value;
    // TODO: 更新 move_base 参数
}

void RobotController::setControllerFrequency(double value)
{
    controller_frequency_ = value;
    // TODO: 更新 move_base 参数
}

void RobotController::setGlobalCostmapUpdateFrequency(double value)
{
    global_costmap_update_frequency_ = value;
    // TODO: 更新 move_base 参数
}

void RobotController::setLocalCostmapUpdateFrequency(double value)
{
    local_costmap_update_frequency_ = value;
    // TODO: 更新 move_base 参数
}

void RobotController::setPlannedPathBias(double value)
{
    planned_path_bias_ = value;
    // TODO: 更新 move_base 参数
}

void RobotController::setRecoveryBehaviorEnabled(bool enabled)
{
    recovery_behavior_enabled_ = enabled;
    // TODO: 更新 move_base 参数
}

void RobotController::setClearingRotationAllowed(bool allowed)
{
    clearing_rotation_allowed_ = allowed;
    // TODO: 更新 move_base 参数
}

void RobotController::setNavigationMode(int mode)
{
    navigation_mode_ = mode;
    // TODO: 根据模式切换导航策略
    emit navigationModeChanged(mode);
}

bool RobotController::testConnection(const std::string& master_uri)
{
    try {
        // 临时保存当前的环境变量
        QString old_master = qgetenv("ROS_MASTER_URI");
        
        // 设置新的 Master URI
        qputenv("ROS_MASTER_URI", master_uri.c_str());
        
        // 设置连接超时时间
        ros::Duration timeout(2.0);  // 2秒超时
        ros::Time start_time = ros::Time::now();
        
        // 尝试获取 ROS Master 上的话题列表
        ros::master::V_TopicInfo topic_info;
        bool connected = false;
        
        while (ros::Time::now() - start_time < timeout) {
            if (ros::master::getTopics(topic_info)) {
                // 验证是否能找到关键的ROS话题
                bool found_rosout = false;
                bool found_clock = false;
                
                for (const auto& topic : topic_info) {
                    if (topic.name == "/rosout") {
                        found_rosout = true;
                    }
                    if (topic.name == "/clock") {
                        found_clock = true;
                    }
                }
                
                if (found_rosout) {  // 至少要有 rosout 话题
                    connected = true;
                    break;
                }
            }
            ros::Duration(0.1).sleep();  // 等待100ms后重试
        }
        
        // 恢复原来的环境变量
        qputenv("ROS_MASTER_URI", old_master.toUtf8());
        
        if (connected) {
            ROS_INFO("Successfully connected to ROS Master at %s", master_uri.c_str());
            ROS_INFO("Found %lu topics", topic_info.size());
        } else {
            ROS_WARN("Failed to connect to ROS Master at %s", master_uri.c_str());
        }
        
        return connected;
        
    } catch (const std::exception& e) {
        ROS_ERROR("Error testing connection: %s", e.what());
        return false;
    }
}

void RobotController::setMasterURI(const QString& uri)
{
    master_uri_ = uri;
}

void RobotController::setHostname(const QString& hostname)
{
    hostname_ = hostname;
}

void RobotController::setRobotModel(const std::string& model)
{
    // TODO: 根据机器人型号设置相应的参数
    qDebug() << "Robot model set to:" << QString::fromStdString(model);
}

void RobotController::setSerialPort(const std::string& port)
{
    // TODO: 配置串口设备
    qDebug() << "Serial port set to:" << QString::fromStdString(port);
}

void RobotController::setBaudrate(int baudrate)
{
    // TODO: 配置串口波特率
    qDebug() << "Baudrate set to:" << baudrate;
}

void RobotController::setMaxLinearVelocity(double max_linear)
{
    max_linear_velocity_ = max_linear;
    qDebug() << "Max linear velocity set to:" << max_linear;
}

void RobotController::setMaxAngularVelocity(double max_angular)
{
    max_angular_velocity_ = max_angular;
    qDebug() << "Max angular velocity set to:" << max_angular;
}

void RobotController::enableInitialPoseSetting(bool enable)
{
    if (enable) {
        interaction_mode_ = InteractionMode::SET_INITIAL_POSE;
        ROS_INFO("Switched to initial pose mode");
    } else {
        interaction_mode_ = InteractionMode::NONE;
        ROS_INFO("Switched back to default mode");
    }
}

void RobotController::enableGoalSetting(bool enable)
{
    if (enable) {
        // 切换到目标点设置工具
        interaction_mode_ = InteractionMode::SET_GOAL;
        std_msgs::String tool_msg;
        tool_msg.data = "rviz/SetGoal";
        tool_manager_pub_.publish(tool_msg);
        ROS_INFO("Switched to goal tool");
    } else {
        // 切换回默认工具
        interaction_mode_ = InteractionMode::NONE;
        std_msgs::String tool_msg;
        tool_msg.data = "rviz/MoveCamera";
        tool_manager_pub_.publish(tool_msg);
        ROS_INFO("Switched back to default tool");
    }
}

void RobotController::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    if (!initial_pose_pub_) {
        ROS_ERROR("Initial pose publisher not initialized");
        return;
    }

    try {
        static ros::Time last_publish_time = ros::Time(0);
        ros::Time current_time = ros::Time::now();
        
        // 如果距离上次发布时间小于1秒，则跳过
        if ((current_time - last_publish_time).toSec() < 1.0) {
            return;
        }

        geometry_msgs::PoseWithCovarianceStamped current_pose = *msg;
        current_pose.header.stamp = current_time;
        
        initial_pose_pub_.publish(current_pose);
        ROS_INFO("Published initial pose");
        
        current_amcl_pose_ = current_pose;
        current_pose_ = current_pose.pose.pose;
        
        double yaw = tf2::getYaw(current_pose.pose.pose.orientation) * 180.0 / M_PI;
        
        emit poseUpdated(current_pose.pose.pose);
        
        QString status_msg = QString("当前位置: (%1, %2) 朝向%3°")
            .arg(current_pose_.position.x, 0, 'f', 3)
            .arg(current_pose_.position.y, 0, 'f', 3)
            .arg(yaw, 0, 'f', 1);
        emit localizationStatusChanged(status_msg);
        
        // 只在第一次设置初始位姿时清除代价地图
        static bool first_pose_set = false;
        if (!first_pose_set) {
            std_srvs::Empty srv;
            if (clear_costmaps_client_.call(srv)) {
                ROS_INFO("Costmaps cleared after setting initial pose");
            }
            first_pose_set = true;
        }
        
        // 切换回移动相机工具
        interaction_mode_ = InteractionMode::NONE;
        std_msgs::String tool_msg;
        tool_msg.data = "rviz/MoveCamera";
        tool_manager_pub_.publish(tool_msg);
        
        last_publish_time = current_time;
        ROS_INFO("Initial pose set");
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in initialPoseCallback: %s", e.what());
    } catch (...) {
        ROS_ERROR("Unknown exception in initialPoseCallback");
    }
}

void RobotController::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    try {
        ROS_INFO("Received new navigation goal");
        
        // 完整复制目标点信息
        current_goal_.header = msg->header;
        current_goal_.header.frame_id = "map";  // 确保frame_id正确
        current_goal_.header.stamp = ros::Time::now();
        current_goal_.pose = msg->pose;
        
        double yaw = tf2::getYaw(msg->pose.orientation) * 180.0 / M_PI;
        
        ROS_INFO("Goal set: x=%.3f, y=%.3f, yaw=%.3f",
                 current_goal_.pose.position.x,
                 current_goal_.pose.position.y,
                 yaw);
                 
        QString status_msg = QString("已设置导航目标: (%1, %2) 朝向%3°")
            .arg(current_goal_.pose.position.x, 0, 'f', 3)
            .arg(current_goal_.pose.position.y, 0, 'f', 3)
            .arg(yaw, 0, 'f', 1);
        emit navigationStatusChanged(status_msg);

        // 等待一小段时间确保目标点被正确保存
        ros::Duration(0.1).sleep();
        
        // 验证目标点是否正确保存
        if (current_goal_.header.frame_id.empty()) {
            ROS_ERROR("Failed to save navigation goal");
            emit navigationStatusChanged("设置导航目标点失败！");
            return;
        }
        
        // 清除代价地图
        std_srvs::Empty srv;
        if (clear_costmaps_client_.call(srv)) {
            ROS_INFO("Costmaps cleared");
        }
        
        // 发送目标点设置信号
        emit goalSet(current_goal_);
        
        // 不再自动开始导航,等待用户点击开始导航按钮
        ROS_INFO("Navigation goal is ready. Waiting for start command.");
        emit navigationStatusChanged("导航目标已设置,请点击开始导航");
        emit navigationStateChanged("就绪");
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in goalCallback: %s", e.what());
        emit navigationStatusChanged("设置导航目标点失败！");
    }
}

void RobotController::startAutoLocalization()
{
    ROS_INFO("Starting auto localization...");
    emit localizationStatusChanged("正在启动自动定位...");
    
    if (!is_initialized_) {
        ROS_ERROR("RobotController not initialized!");
        emit localizationStatusChanged("错误：RobotController未初始化！");
        return;
    }

    if (is_navigating_) {
        ROS_INFO("Canceling navigation before starting auto localization");
        emit localizationStatusChanged("正在取消导航...");
        stopNavigation();
        ros::Duration(0.5).sleep();
    }
    
    is_localizing_ = true;
    localization_progress_ = 0.0;
    
    // 清除代价地图
    std_srvs::Empty srv;
    if (!clear_costmaps_client_.call(srv)) {
        ROS_WARN("Failed to clear costmaps");
        emit localizationStatusChanged("警告：清除代价地图失败");
    } else {
        emit localizationStatusChanged("已清除代价地图");
    }
    
    // 触发全局定位
    std_srvs::Empty global_loc_srv;
    if (global_localization_client_.call(global_loc_srv)) {
        ROS_INFO("Global localization triggered");
        emit localizationStatusChanged("已触发全局定位，正在分散粒子...");
    } else {
        ROS_WARN("Failed to trigger global localization");
        emit localizationStatusChanged("警告：触发全局定位失败");
    }
    
    // 获取机器人当前位置作为自动定位的中心点
    try {
        tf::StampedTransform transform;
        ROS_INFO("Looking up transform from map to base_footprint");
        emit localizationStatusChanged("正在获取机器人当前位置...");
        
        tf_listener_.lookupTransform("map", "base_footprint", ros::Time(0), transform);
        localization_center_.x = transform.getOrigin().x();
        localization_center_.y = transform.getOrigin().y();
        localization_center_.z = 0.0;
        
        QString center_msg = QString("已设置定位中心点: (%1, %2)")
            .arg(localization_center_.x, 0, 'f', 3)
            .arg(localization_center_.y, 0, 'f', 3);
        emit localizationStatusChanged(center_msg);
        
        ROS_INFO("Got robot position: x=%.2f, y=%.2f", localization_center_.x, localization_center_.y);
    } catch (tf::TransformException& ex) {
        ROS_WARN("Failed to get robot position: %s", ex.what());
        // 如果无法获取位置，使用(0,0)作为中心点
        localization_center_.x = 0.0;
        localization_center_.y = 0.0;
        localization_center_.z = 0.0;
        emit localizationStatusChanged("警告：无法获取机器人位置，使用(0,0)作为中心点");
    }
    
    // 发布可视化标记
    ROS_INFO("Publishing visualization markers");
    emit localizationStatusChanged("正在显示定位范围标记...");
    publishLocalizationMarkers();
    
    // 设置AMCL参数
    ROS_INFO("Setting AMCL parameters");
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::Config conf;
    
    // 设置整数参数
    dynamic_reconfigure::IntParameter min_particles;
    min_particles.name = "min_particles";
    min_particles.value = 20000;
    conf.ints.push_back(min_particles);
    
    dynamic_reconfigure::IntParameter max_particles;
    max_particles.name = "max_particles";
    max_particles.value = 50000;
    conf.ints.push_back(max_particles);
    
    // 设置浮点数参数
    dynamic_reconfigure::DoubleParameter update_min_d;
    update_min_d.name = "update_min_d";
    update_min_d.value = 0.02;
    conf.doubles.push_back(update_min_d);
    
    dynamic_reconfigure::DoubleParameter update_min_a;
    update_min_a.name = "update_min_a";
    update_min_a.value = 0.05;
    conf.doubles.push_back(update_min_a);
    
    srv_req.config = conf;
    
    if (!ros::service::call("/amcl/set_parameters", srv_req, srv_resp)) {
        ROS_WARN("Failed to set AMCL parameters");
    }
    
    // 初始化运动参数
    current_rotation_speed_ = 0.3;  // 初始旋转速度
    current_linear_speed_ = 0.0;    // 初始线速度
    last_direction_change_ = ros::Time::now();
    
    // 启动定位监控定时器
    ROS_INFO("Starting localization monitor timer");
    localization_monitor_timer_ = nh_.createTimer(
        ros::Duration(0.1),  // 提高更新频率到10Hz 
        &RobotController::monitorLocalization,
        this
    );
    
    emit localizationProgressChanged(0.0);
    emit localizationStatusChanged("开始自动定位...");
    
    ROS_INFO("Auto localization started successfully");
}

void RobotController::monitorLocalization(const ros::TimerEvent&)
{
    if (!is_localizing_) {
        return;
    }

    try {
        // 计算粒子云的协方差来评估定位质量
        double position_variance = 
            current_pose_cov_[0] + current_pose_cov_[7];  // x和y方向的方差之和
        double orientation_variance = current_pose_cov_[35]; // yaw方向的方差
        
        // 获取当前机器人位置
        tf::StampedTransform transform;
        try {
            tf_listener_.lookupTransform("map", "base_footprint", ros::Time(0), transform);
            
            // 计算到中心点的距离
            double dx = transform.getOrigin().x() - localization_center_.x;
            double dy = transform.getOrigin().y() - localization_center_.y;
            double distance = sqrt(dx*dx + dy*dy);
            
            // 如果检测到障碍物，执行避让
            if (is_obstacle_detected_) {
                // 根据激光扫描数据选择避让方向
                if (left_space_larger_) {
                    current_rotation_speed_ = 0.2;  // 降低避障转向速度
                    current_linear_speed_ = 0.0;
                    emit localizationStatusChanged("正在向左避让障碍物...");
                } else {
                    current_rotation_speed_ = -0.2;  // 降低避障转向速度
                    current_linear_speed_ = 0.0;
                    emit localizationStatusChanged("正在向右避让障碍物...");
                }
            }
            // 如果没有障碍物，执行正常的定位运动
            else {
                // 根据距离调整速度
                if (distance > auto_localization_radius_ * 0.9) {
                    // 接近边界时减速并转向中心
                    double angle_to_center = atan2(-dy, -dx);
                    double current_yaw = tf::getYaw(transform.getRotation());
                    double angle_diff = angles::shortest_angular_distance(current_yaw, angle_to_center);
                    
                    // 如果面向中心，向前移动；否则原地旋转
                    if (fabs(angle_diff) < 0.2) {  // 降低角度阈值
                        current_rotation_speed_ = 0.0;
                        current_linear_speed_ = 0.03;  // 降低线速度为3cm/s
                        emit localizationStatusChanged("正在向中心移动...");
                    } else {
                        current_rotation_speed_ = (angle_diff > 0) ? 0.15 : -0.15;  // 降低旋转速度
                        current_linear_speed_ = 0.0;
                        emit localizationStatusChanged("正在转向中心点...");
                    }
                } else {
                    // 在安全范围内，执行定位运动
                    static ros::Time last_switch = ros::Time::now();
                    double time_since_switch = (ros::Time::now() - last_switch).toSec();
                    
                    // 每5秒切换一次运动方向
                    if (time_since_switch > 5.0) {
                        current_rotation_speed_ = -current_rotation_speed_;
                        last_switch = ros::Time::now();
                    }
                    
                    // 保持较低的速度进行精确定位
                    if (current_rotation_speed_ == 0.0) {
                        current_rotation_speed_ = 0.15;  // 降低旋转速度
                    }
                    current_linear_speed_ = 0.02;  // 保持较低的线速度
                }
            }
            
            // 发布速度命令
            publishVelocity(current_linear_speed_, current_rotation_speed_);
            
        } catch (tf::TransformException& ex) {
            ROS_WARN_THROTTLE(1.0, "Failed to get robot position: %s", ex.what());
            publishVelocity(0.0, 0.0);
            emit localizationStatusChanged("无法获取机器人位置，请检查TF转换...");
        }
        
        // 使用更严格的方差阈值判断定位成功
        if (position_variance < 0.05 && orientation_variance < 0.05) {  // 降低方差阈值
            localization_monitor_timer_.stop();
            
            // 停止机器人运动
            publishVelocity(0.0, 0.0);
            
            // 将当前AMCL位姿设置为初始位姿
            geometry_msgs::PoseWithCovarianceStamped initial_pose;
            initial_pose.header.frame_id = "map";
            initial_pose.header.stamp = ros::Time::now();
            initial_pose.pose = current_amcl_pose_.pose;
            
            // 设置协方差
            for (int i = 0; i < 36; ++i) {
                initial_pose.pose.covariance[i] = 0.0;
            }
            initial_pose.pose.covariance[0] = 0.25;   // x
            initial_pose.pose.covariance[7] = 0.25;   // y
            initial_pose.pose.covariance[35] = 0.068; // yaw
            
            if (!initial_pose_pub_) {
                ROS_ERROR("Initial pose publisher is invalid!");
                emit localizationStatusChanged("错误：无法发布初始位姿！");
                return;
            }

            // 多次发布初始位姿以确保设置成功
            for(int i = 0; i < 3; i++) {
                try {
                    initial_pose_pub_.publish(initial_pose);
                    ros::Duration(0.1).sleep();  // 等待0.1秒
                } catch (const std::exception& e) {
                    ROS_ERROR("Failed to publish initial pose: %s", e.what());
                    emit localizationStatusChanged("错误：发布初始位姿失败！");
                    return;
                }
            }
            ROS_INFO("Published initial pose multiple times after successful localization");
            
            // 多次清除代价地图以确保完全重置
            std_srvs::Empty srv;
            for(int i = 0; i < 2; i++) {
                if (clear_costmaps_client_.call(srv)) {
                    ROS_INFO("Costmaps cleared after successful localization (attempt %d)", i+1);
                }
                ros::Duration(0.1).sleep();
            }
            
            is_localizing_ = false;
            is_localized_ = true;
            
            // 计算最终位置和朝向
            double final_yaw = tf2::getYaw(initial_pose.pose.pose.orientation) * 180.0 / M_PI;
            QString result_msg = QString("定位成功！最终位置: (%1, %2) 朝向: %3°")
                .arg(initial_pose.pose.pose.position.x, 0, 'f', 3)
                .arg(initial_pose.pose.pose.position.y, 0, 'f', 3)
                .arg(final_yaw, 0, 'f', 1);
            
            emit localizationStatusChanged(result_msg);
            emit localizationProgressChanged(100.0);
            emit localizationStateChanged("已完成");
            
            // 重新初始化导航相关组件
            ros::Duration(0.5).sleep();  // 等待系统稳定
            
            // 确保move_base客户端仍然有效
            if (!move_base_client_ || !move_base_client_->isServerConnected()) {
                ROS_WARN("Reinitializing move_base client after localization");
                move_base_client_.reset(new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true));
                ros::Duration(1.0).sleep();  // 等待连接建立
            }
        } else {
            // 根据方差计算定位进度
            double progress = std::max(0.0, std::min(100.0, 
                (1.0 - position_variance/1.0) * 100.0));  // 调整进度计算
            emit localizationProgressChanged(progress);
            emit localizationStatusChanged(
                QString("正在定位 (位置误差: %1m, 角度误差: %2°)")
                .arg(sqrt(position_variance), 0, 'f', 3)  // 显示更精确的误差值
                .arg(sqrt(orientation_variance) * 180.0 / M_PI, 0, 'f', 2)
            );
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in monitorLocalization: %s", e.what());
        emit localizationStatusChanged(QString("定位监控出错：%1").arg(e.what()));
    }
}

void RobotController::setupSafety()
{
    // 订阅激光雷达数据
    laser_scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(
        "scan", 1, &RobotController::laserScanCallback, this);
        
    // 设置安全参数
    safety_distance_ = 0.2;  // 安全距离阈值(20厘米)
    is_obstacle_detected_ = false;
}

void RobotController::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    if (!is_localizing_) return;  // 只在自动定位时进行障碍物检测

    // 检查前方180度范围内是否有障碍物
    int start_index = scan->ranges.size() / 4;  // 左侧45度
    int end_index = scan->ranges.size() * 3 / 4;  // 右侧45度
    
    // 计算左右两侧的平均距离
    float left_avg_dist = 0.0f;
    float right_avg_dist = 0.0f;
    int left_count = 0;
    int right_count = 0;
    
    // 计算中间区域的最小距离
    float min_front_dist = scan->range_max;
    int mid_start = scan->ranges.size() * 3 / 8;  // 前方左侧22.5度
    int mid_end = scan->ranges.size() * 5 / 8;    // 前方右侧22.5度
    
    // 统计左右两侧和前方的距离
    for (int i = start_index; i < end_index; i++) {
        if (scan->ranges[i] > scan->range_min && scan->ranges[i] < scan->range_max) {
            if (i < mid_start) {  // 左侧区域
                left_avg_dist += scan->ranges[i];
                left_count++;
            } else if (i > mid_end) {  // 右侧区域
                right_avg_dist += scan->ranges[i];
                right_count++;
            } else {  // 前方区域
                min_front_dist = std::min(min_front_dist, scan->ranges[i]);
            }
        }
    }
    
    // 计算平均值
    if (left_count > 0) left_avg_dist /= left_count;
    if (right_count > 0) right_avg_dist /= right_count;
    
    // 更新避让方向标志
    left_space_larger_ = (left_avg_dist > right_avg_dist);
    
    // 如果前方距离小于安全距离，标记为检测到障碍物
    if (min_front_dist < static_cast<float>(safety_distance_)) {
        bool was_obstacle_detected = is_obstacle_detected_;
        is_obstacle_detected_ = true;
        
        // 只在状态改变或每2秒输出一次日志
        if (!was_obstacle_detected) {
            ROS_INFO("Obstacle detected at distance %.2f m, will turn %s", 
                     min_front_dist, 
                     left_space_larger_ ? "left" : "right");
        } else {
            ROS_INFO_THROTTLE(2.0, "Obstacle detected at distance %.2f m, will turn %s", 
                     min_front_dist, 
                     left_space_larger_ ? "left" : "right");
        }
    } else {
        if (is_obstacle_detected_) {
            ROS_INFO("Obstacle cleared");
        }
        is_obstacle_detected_ = false;
    }
}

void RobotController::navigationDoneCallback(
    const actionlib::SimpleClientGoalState& state,
    const move_base_msgs::MoveBaseResultConstPtr& result)
{
    is_navigating_ = false;

    switch (state.state_) {
        case actionlib::SimpleClientGoalState::SUCCEEDED:
            ROS_INFO("Navigation succeeded");
            emit navigationStateChanged("已完成");
            emit navigationStatusChanged("导航成功完成");
            emit navigationProgressChanged(100.0);
            break;

        case actionlib::SimpleClientGoalState::ABORTED:
            ROS_WARN("Navigation aborted");
            emit navigationStateChanged("已中止");
            emit navigationStatusChanged("导航被中止");
            break;

        case actionlib::SimpleClientGoalState::REJECTED:
            ROS_WARN("Navigation goal rejected");
            emit navigationStateChanged("已拒绝");
            emit navigationStatusChanged("导航目标被拒绝");
            break;

        case actionlib::SimpleClientGoalState::PREEMPTED:
            ROS_INFO("Navigation preempted");
            emit navigationStateChanged("已取消");
            emit navigationStatusChanged("导航已取消");
            break;

        case actionlib::SimpleClientGoalState::LOST:
            ROS_ERROR("Navigation lost");
            emit navigationStateChanged("已丢失");
            emit navigationStatusChanged("导航目标丢失");
            break;

        default:
            ROS_ERROR("Unknown navigation result");
            emit navigationStateChanged("未知状态");
            emit navigationStatusChanged("未知的导航结果");
            break;
    }
}

void RobotController::navigationActiveCallback()
{
    ROS_INFO("Navigation goal is now active");
    is_navigating_ = true;
    emit navigationStateChanged("进行中");
    emit navigationStatusChanged("正在导航到目标点");
}

void RobotController::navigationFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
    if (!is_navigating_) return;

    // 计算到目标点的距离
    double dx = current_goal_.pose.position.x - feedback->base_position.pose.position.x;
    double dy = current_goal_.pose.position.y - feedback->base_position.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    // 更新导航进度
    double progress = std::max(0.0, std::min(100.0, (1.0 - distance / initial_distance_) * 100.0));
    
    // 检查是否卡住
    static ros::Time last_progress_time = ros::Time::now();
    static double last_distance = distance;
    
    if (std::abs(distance - last_distance) < 0.01) {  // 如果距离变化很小
        if ((ros::Time::now() - last_progress_time).toSec() > 10.0) {  // 超过10秒没有进展
            ROS_WARN("Navigation seems stuck");
            emit navigationStatusChanged(QString("导航似乎卡住了，尝试重新规划路径..."));
            // 清除代价地图
            std_srvs::Empty srv;
            if (clear_costmaps_client_.call(srv)) {
                ROS_INFO("Costmaps cleared due to stuck condition");
            }
            last_progress_time = ros::Time::now();
        }
    } else {
        last_progress_time = ros::Time::now();
        last_distance = distance;
    }
    
    ROS_INFO("Navigation progress: %.1f%%, distance: %.2fm", progress, distance);
    
    emit navigationProgressChanged(progress);
    emit navigationStatusChanged(QString("距离目标点还有 %1 米 (进度: %2%%)").arg(distance, 0, 'f', 2).arg(progress, 0, 'f', 1));
    emit distanceToGoalChanged(distance);
}

void RobotController::setGlobalPlanner(const QString& planner_name)
{
    if (planner_name == current_global_planner_) {
        return;  // 如果规划器没有改变，直接返回
    }

    // 设置全局规划器
    if (nh_.hasParam("/move_base/base_global_planner")) {
        nh_.setParam("/move_base/base_global_planner", planner_name.toStdString());
        ROS_INFO_STREAM("Set global planner to: " << planner_name.toStdString());
        
        // 更新当前规划器
        current_global_planner_ = planner_name;
        
        // 清除代价地图以应用新的规划器
        std_srvs::Empty srv;
        if (clear_costmaps_client_.call(srv)) {
            ROS_INFO("Costmaps cleared after changing global planner");
        }
        
        // 发送信号通知规划器已更改
        emit globalPlannerChanged(planner_name);
    }
}

void RobotController::setLocalPlanner(const QString& planner_name)
{
    if (planner_name == current_local_planner_) {
        return;  // 如果规划器没有改变，直接返回
    }

    // 设置局部规划器
    if (nh_.hasParam("/move_base/base_local_planner")) {
        nh_.setParam("/move_base/base_local_planner", planner_name.toStdString());
        ROS_INFO_STREAM("Set local planner to: " << planner_name.toStdString());
        
        // 更新当前规划器
        current_local_planner_ = planner_name;
        
        // 清除代价地图以应用新的规划器
        std_srvs::Empty srv;
        if (clear_costmaps_client_.call(srv)) {
            ROS_INFO("Costmaps cleared after changing local planner");
        }
        
        // 发送信号通知规划器已更改
        emit localPlannerChanged(planner_name);
    }
}

std::vector<QString> RobotController::getAvailableGlobalPlanners() const
{
    return {
        "navfn/NavfnROS",              // 默认全局规划器，使用Dijkstra算法
        "global_planner/GlobalPlanner", // 改进的A*规划器，支持更多启发式方法
        "carrot_planner/CarrotPlanner"  // 简单的直线规划器，适合简单环境
    };
}

std::vector<QString> RobotController::getAvailableLocalPlanners() const
{
    return {
        "base_local_planner/TrajectoryPlannerROS",    // 传统DWA规划器
        "dwa_local_planner/DWAPlannerROS",            // 改进的DWA规划器，更平滑
        "teb_local_planner/TebLocalPlannerROS",       // TEB时间弹性带规划器，适合动态环境
        "eband_local_planner/EBandPlannerROS"         // 弹性带规划器，路径更平滑
    };
}

void RobotController::publishLocalizationMarkers()
{
    visualization_msgs::MarkerArray marker_array;
    
    // 创建圆形边界标记
    visualization_msgs::Marker circle_marker;
    circle_marker.header.frame_id = "map";
    circle_marker.header.stamp = ros::Time::now();
    circle_marker.ns = "localization_boundary";
    circle_marker.id = 0;
    circle_marker.type = visualization_msgs::Marker::CYLINDER;
    circle_marker.action = visualization_msgs::Marker::ADD;
    
    // 设置圆柱体位置和大小
    circle_marker.pose.position = localization_center_;
    circle_marker.pose.position.z = 0.01;  // 稍微抬高一点以避免与地图重叠
    circle_marker.pose.orientation.w = 1.0;
    circle_marker.scale.x = auto_localization_radius_ * 2;
    circle_marker.scale.y = auto_localization_radius_ * 2;
    circle_marker.scale.z = 0.02;
    
    // 设置颜色(半透明蓝色)
    circle_marker.color.r = 0.0;
    circle_marker.color.g = 0.5;
    circle_marker.color.b = 1.0;
    circle_marker.color.a = 0.3;
    
    marker_array.markers.push_back(circle_marker);
    
    // 创建中心点标记
    visualization_msgs::Marker center_marker;
    center_marker.header = circle_marker.header;
    center_marker.ns = "localization_center";
    center_marker.id = 1;
    center_marker.type = visualization_msgs::Marker::SPHERE;
    center_marker.action = visualization_msgs::Marker::ADD;
    
    // 设置球体位置和大小
    center_marker.pose.position = localization_center_;
    center_marker.pose.position.z = 0.05;
    center_marker.pose.orientation.w = 1.0;
    center_marker.scale.x = 0.1;
    center_marker.scale.y = 0.1;
    center_marker.scale.z = 0.1;
    
    // 设置颜色(红色)
    center_marker.color.r = 1.0;
    center_marker.color.g = 0.0;
    center_marker.color.b = 0.0;
    center_marker.color.a = 1.0;
    
    marker_array.markers.push_back(center_marker);
    
    // 发布标记
    marker_pub_.publish(marker_array);
}

void RobotController::stopAutoLocalization()
{
    if (!is_localizing_) {
        return;
    }

    ROS_INFO("Stopping auto localization...");
    
    // 停止定位监控定时器
    localization_monitor_timer_.stop();
    
    // 停止机器人运动
    publishVelocity(0.0, 0.0);
    
    // 重置状态
    is_localizing_ = false;
    localization_progress_ = 0.0;
    
    // 发送状态更新
    emit localizationProgressChanged(0.0);
    emit localizationStatusChanged("自动定位已停止");
    emit localizationStateChanged("已取消");
    
    ROS_INFO("Auto localization stopped");
}

void RobotController::handleMappingProgress(const std_msgs::Float32::ConstPtr& msg)
{
    // 处理建图进度
    double progress = msg->data * 100.0;  // 将进度转换为百分比
    emit mappingProgressChanged(progress);
}

void RobotController::updateMappingParameters(const std::map<std::string, double>& params)
{
    if (!is_initialized_) {
        ROS_ERROR("RobotController not initialized");
        return;
    }

    try {
        // 根据当前的建图方法更新参数
        if (current_mapping_method_ == "Gmapping SLAM") {
            // 使用 dynamic_reconfigure 更新 Gmapping 参数
            dynamic_reconfigure::ReconfigureRequest req;
            dynamic_reconfigure::ReconfigureResponse res;
            dynamic_reconfigure::DoubleParameter double_param;
            dynamic_reconfigure::Config conf;

            for (const auto& param : params) {
                double_param.name = param.first;
                double_param.value = param.second;
                conf.doubles.push_back(double_param);
            }

            req.config = conf;
            ros::service::call("/slam_gmapping/set_parameters", req, res);
        }
        else if (current_mapping_method_ == "Cartographer SLAM") {
            // 更新 Cartographer 参数
            // TODO: 实现 Cartographer 参数更新
        }
        else if (current_mapping_method_ == "Hector SLAM") {
            // 更新 Hector SLAM 参数
            // TODO: 实现 Hector SLAM 参数更新
        }

        ROS_INFO("Updated mapping parameters for %s", current_mapping_method_.toStdString().c_str());
    }
    catch (const std::exception& e) {
        ROS_ERROR("Failed to update mapping parameters: %s", e.what());
    }
}

bool RobotController::connectToRobot()
{
    if (is_connected_) {
        ROS_WARN("Already connected to robot");
        return true;
    }

    try {
        if (master_uri_.isEmpty()) {
            throw std::runtime_error("ROS_MASTER_URI not set");
        }

        // 设置ROS环境变量
        qputenv("ROS_MASTER_URI", master_uri_.toUtf8());
        if (!hostname_.isEmpty()) {
            qputenv("ROS_HOSTNAME", hostname_.toUtf8());
        }

        // 初始化ROS连接
        if (!setupROSConnection()) {
            throw std::runtime_error("Failed to initialize ROS connection");
        }

        is_connected_ = true;
        emit connectionStateChanged(true);
        ROS_INFO("Connected to robot at %s", master_uri_.toStdString().c_str());
        return true;

    } catch (const std::exception& e) {
        ROS_ERROR("Failed to connect to robot: %s", e.what());
        emit connectionError(QString("连接失败: %1").arg(e.what()));
        return false;
    }
}

void RobotController::disconnectFromRobot()
{
    if (!is_connected_) return;

    cleanupROSConnection();
    is_connected_ = false;
    emit connectionStateChanged(false);
    ROS_INFO("Disconnected from robot");
}

bool RobotController::setupROSConnection()
{
    // 检查ROS Master是否可达
    if (!ros::master::check()) {
        ROS_ERROR("Cannot contact ROS master at %s", master_uri_.toStdString().c_str());
        return false;
    }

    // 初始化所有ROS订阅者和发布者
    try {
        // 导航相关
        move_base_client_.reset(new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true));
        if (!move_base_client_->waitForServer(ros::Duration(5.0))) {
            ROS_WARN("Move base action server not available");
        }

        // 发布者
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);

        // 订阅者
        odom_sub_ = nh_.subscribe("odom", 1, &RobotController::odomCallback, this);
        amcl_pose_sub_ = nh_.subscribe("amcl_pose", 1, &RobotController::amclPoseCallback, this);
        scan_sub_ = nh_.subscribe("scan", 1, &RobotController::scanCallback, this);
        battery_sub_ = nh_.subscribe("battery_state", 1, &RobotController::batteryCallback, this);

        // 服务客户端
        clear_costmaps_client_ = nh_.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");

        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("Error setting up ROS connections: %s", e.what());
        return false;
    }
}

void RobotController::cleanupROSConnection()
{
    // 关闭所有订阅者和发布者
    cmd_vel_pub_.shutdown();
    initial_pose_pub_.shutdown();
    odom_sub_.shutdown();
    amcl_pose_sub_.shutdown();
    scan_sub_.shutdown();
    battery_sub_.shutdown();
    
    // 重置action client
    move_base_client_.reset();
} 