/**
 * @file robot_controller.cpp
 * @brief ROS通信管理类实现
 */

#include "ros/robot_controller.h"
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

RobotController::RobotController(QObject* parent)
    : QObject(parent)
{
    // 设置发布者和订阅者
    setupPublishers();
    setupSubscribers();
    
    // 初始化服务客户端
    global_localization_client_ = nh_.serviceClient<std_srvs::Empty>("/global_localization");
    clear_costmaps_client_ = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    
    // 初始化 action 客户端
    move_base_client_ = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
    
    // 等待 action 服务器启动
    if (!move_base_client_->waitForServer(ros::Duration(5.0))) {
        ROS_WARN("Move base action server not available");
    }
    
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
        
    // 关闭所有发布者和订阅者
    cmd_vel_pub_.shutdown();
    initial_pose_pub_.shutdown();
        odom_sub_.shutdown();
    battery_sub_.shutdown();
    diagnostics_sub_.shutdown();
        scan_sub_.shutdown();
        amcl_pose_sub_.shutdown();
    map_sub_.shutdown();
    
    // 关闭服务客户端
    global_localization_client_.shutdown();
    clear_costmaps_client_.shutdown();
}

void RobotController::setupPublishers()
{
    // 发布速度命令
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    
    // 发布初始位姿
    initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    
    // 发布全局定位请求
    global_localization_pub_ = nh_.advertise<std_msgs::Bool>("/global_localization", 1);
    
    // 发布交互标记
    interactive_marker_server_.reset(new interactive_markers::InteractiveMarkerServer("robot_control_markers", "", false));
    
    // 设置工具管理器发布者
    tool_manager_pub_ = nh_.advertise<std_msgs::String>("/rviz/current_tool", 1);
    
    // 等待所有发布者初始化完成
    ros::Duration(0.5).sleep();
    
    // 检查发布者是否有效
    if (!cmd_vel_pub_ || !initial_pose_pub_ || !global_localization_pub_ || !tool_manager_pub_) {
        ROS_ERROR("Failed to initialize publishers");
        return;
    }
    
    ROS_INFO("All publishers initialized successfully");
}

void RobotController::setupSubscribers()
{
    // 订阅机器人状态信息
    odom_sub_ = nh_.subscribe("/odom", 1, &RobotController::odomCallback, this);
    battery_sub_ = nh_.subscribe("/battery_state", 1, &RobotController::batteryCallback, this);
    diagnostics_sub_ = nh_.subscribe("/diagnostics", 1, &RobotController::diagnosticsCallback, this);
    scan_sub_ = nh_.subscribe("/scan", 1, &RobotController::scanCallback, this);
    amcl_pose_sub_ = nh_.subscribe("/amcl_pose", 1, &RobotController::amclPoseCallback, this);
    map_sub_ = nh_.subscribe("/map", 1, &RobotController::mapCallback, this);
    
    // 订阅RViz的交互反馈
    initial_pose_sub_ = nh_.subscribe("/initialpose", 1, &RobotController::initialPoseCallback, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &RobotController::goalCallback, this);
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
    initial_pose_pub_.publish(pose);
}

void RobotController::startGlobalLocalization()
{
    if (!is_localizing_) {
        std_srvs::Empty srv;
        if (global_localization_client_.call(srv)) {
            is_localizing_ = true;
            localization_progress_ = 0.0;
            emit localizationStateChanged(false);
            emit localizationProgressChanged(0.0);
            emit localizationStatusChanged("正在进行全局定位...");
            
            // 启动定位后的回调
            ros::Duration(2.0).sleep(); // 等待粒子扩散
            // 订阅AMCL位姿以获取定位结果
            amcl_pose_sub_ = nh_.subscribe("/amcl_pose", 1, &RobotController::amclPoseCallback, this);
        }
    }
}

void RobotController::cancelGlobalLocalization()
{
    if (is_localizing_) {
        is_localizing_ = false;
        localization_progress_ = 0.0;
        emit localizationStateChanged(false);
        emit localizationProgressChanged(0.0);
    }
}

void RobotController::startNavigation()
{
    if (!is_localized_) {
        ROS_ERROR("Cannot start navigation: robot not localized");
        emit navigationStatusChanged("请先设置机器人初始位置！");
        return;
    }

    if (is_localizing_) {
        ROS_ERROR("Cannot start navigation: robot is localizing");
        emit navigationStatusChanged("请等待定位完成！");
        return;
    }

    if (!move_base_client_ || !move_base_client_->isServerConnected()) {
        ROS_ERROR("Move base action client not initialized or not connected");
        emit navigationStatusChanged("导航服务未连接，请检查move_base节点是否正常运行");
        return;
    }

    if (is_navigating_) {
        ROS_INFO("Navigation already in progress");
        return;
    }

    // 如果有保存的目标点，重新发送
    if (!current_goal_.header.frame_id.empty()) {
        ROS_INFO("Starting navigation to saved goal: x=%.2f, y=%.2f",
                 current_goal_.pose.position.x,
                 current_goal_.pose.position.y);
        setNavigationGoal(current_goal_);
    } else {
        ROS_ERROR("No navigation goal set");
        emit navigationStatusChanged("请先设置导航目标点！");
    }
}

void RobotController::pauseNavigation()
{
    if (!move_base_client_ || !move_base_client_->isServerConnected()) {
        qDebug() << "Move base action client not initialized or not connected";
        return;
    }

    move_base_client_->cancelGoal();
    emit navigationStateChanged(NavigationState::PAUSED);
}

void RobotController::stopNavigation()
{
    if (!move_base_client_ || !move_base_client_->isServerConnected()) {
        ROS_ERROR("Move base action client not initialized or not connected");
        return;
    }

    move_base_client_->cancelAllGoals();
    is_navigating_ = false;
    emit navigationStateChanged(NavigationState::STOPPED);
    emit navigationStatusChanged("导航已停止");
    
    // 清除代价地图
    std_srvs::Empty srv;
    if (clear_costmaps_client_.call(srv)) {
        ROS_INFO("Costmaps cleared");
    } else {
        ROS_WARN("Failed to clear costmaps");
    }
}

void RobotController::cancelNavigation()
{
    if (!move_base_client_ || !move_base_client_->isServerConnected()) {
        qDebug() << "Move base action client not initialized or not connected";
        return;
    }

    move_base_client_->cancelAllGoals();
    emit navigationStateChanged(NavigationState::CANCELLED);
}

void RobotController::startMapping()
{
    if (!is_mapping_) {
        is_mapping_ = true;
        mapping_progress_ = 0.0;
        emit mappingStateChanged(true);
        emit mappingProgressChanged(0.0);
    }
}

void RobotController::stopMapping()
{
    if (is_mapping_) {
            is_mapping_ = false;
        mapping_progress_ = 0.0;
        emit mappingStateChanged(false);
        emit mappingProgressChanged(0.0);
    }
}

void RobotController::saveMap(const QString& filename)
{
    // TODO: 实现地图保存功能
    ROS_INFO("Saving map to file: %s", filename.toStdString().c_str());
}

void RobotController::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    emit mapUpdated(*map);
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
    // 更新当前位姿
    current_amcl_pose_ = *msg;
    current_pose_ = msg->pose.pose;
    
    // 更新协方差
    for (size_t i = 0; i < 36; ++i) {
        current_pose_cov_[i] = msg->pose.covariance[i];
    }
    
    // 计算yaw角（弧度转角度）
    double yaw = tf2::getYaw(msg->pose.pose.orientation) * 180.0 / M_PI;
    
    // 发送位姿更新信号
    emit poseUpdated(msg->pose);
    emit localizationStatusChanged(QString("当前位置: (%.3f, %.3f) 朝向%.1f°")
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
    if (!move_base_client_) {
        ROS_ERROR("Move base action client not initialized");
        emit navigationStatusChanged("导航服务未初始化");
        return;
    }

    if (!move_base_client_->isServerConnected()) {
        ROS_ERROR("Move base action server not connected");
        emit navigationStatusChanged("导航服务未连接，请检查move_base节点是否正常运行");
        return;
    }

    if (!is_localized_) {
        ROS_ERROR("Robot not localized");
        emit navigationStatusChanged("请先设置机器人初始位置！");
        return;
    }

    if (is_localizing_) {
        ROS_ERROR("Cannot navigate while localizing");
        emit navigationStatusChanged("请等待定位完成！");
        return;
    }

    // 检查目标点是否在地图范围内
    if (std::abs(goal.pose.position.x) > 50.0 || std::abs(goal.pose.position.y) > 50.0) {
        ROS_ERROR("Goal position out of map bounds");
        emit navigationStatusChanged("目标点超出地图范围！");
        return;
    }

    // 如果正在导航，先取消当前导航
    if (is_navigating_) {
        ROS_INFO("Canceling current navigation before starting new one");
        move_base_client_->cancelAllGoals();
        ros::Duration(0.5).sleep();  // 等待取消完成
    }

    // 保存当前目标点
    current_goal_ = goal;

    // 创建导航目标
    move_base_msgs::MoveBaseGoal move_base_goal;
    move_base_goal.target_pose = goal;
    move_base_goal.target_pose.header.stamp = ros::Time::now();  // 更新时间戳

    // 计算初始距离
    double dx = goal.pose.position.x - current_amcl_pose_.pose.pose.position.x;
    double dy = goal.pose.position.y - current_amcl_pose_.pose.pose.position.y;
    initial_distance_ = std::sqrt(dx * dx + dy * dy);

    // 清除代价地图
    std_srvs::Empty srv;
    if (clear_costmaps_client_.call(srv)) {
        ROS_INFO("Costmaps cleared before navigation");
    } else {
        ROS_WARN("Failed to clear costmaps");
    }

    // 发送目标
    ROS_INFO("Sending navigation goal: x=%.2f, y=%.2f, initial distance=%.2fm",
             goal.pose.position.x, goal.pose.position.y, initial_distance_);
             
    move_base_client_->sendGoal(move_base_goal,
        boost::bind(&RobotController::navigationDoneCallback, this, _1, _2),
        boost::bind(&RobotController::navigationActiveCallback, this),
        boost::bind(&RobotController::navigationFeedbackCallback, this, _1));

    // 更新导航状态
    is_navigating_ = true;
    emit navigationStateChanged(NavigationState::NAVIGATING);
    emit navigationStatusChanged(QString("开始导航，距离目标点 %.2f 米").arg(initial_distance_));
    emit navigationProgressChanged(0.0);
    
    // 确保目标点显示被启用
    emit goalDisplayEnabled(true);
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
    ros::master::V_TopicInfo topic_info;
    return ros::master::getTopics(topic_info);
}

void RobotController::setMasterURI(const QString& uri)
{
    setenv("ROS_MASTER_URI", uri.toStdString().c_str(), 1);
    qDebug() << "Set ROS_MASTER_URI to" << uri;
}

void RobotController::setHostname(const QString& hostname)
{
    setenv("ROS_HOSTNAME", hostname.toStdString().c_str(), 1);
    qDebug() << "Set ROS_HOSTNAME to" << hostname;
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
    // 如果已经在所需的模式，不做任何改变
    if ((enable && interaction_mode_ == InteractionMode::SET_INITIAL_POSE) ||
        (!enable && interaction_mode_ != InteractionMode::SET_INITIAL_POSE)) {
        return;
    }

    if (enable) {
        // 如果当前在目标点设置模式，先取消它
        if (interaction_mode_ == InteractionMode::SET_GOAL) {
            ROS_INFO("Canceling goal setting");
            current_goal_ = geometry_msgs::PoseStamped();
        }
        
        // 切换到初始位姿设置工具
        std_msgs::String tool_msg;
        tool_msg.data = "rviz/SetInitialPose";
        tool_manager_pub_.publish(tool_msg);
        interaction_mode_ = InteractionMode::SET_INITIAL_POSE;
        ROS_INFO("Switched to initial pose tool");
    } else {
        // 切换回默认工具
        std_msgs::String tool_msg;
        tool_msg.data = "rviz/MoveCamera";
        tool_manager_pub_.publish(tool_msg);
        interaction_mode_ = InteractionMode::NONE;
        ROS_INFO("Switched back to default tool");
    }
}

void RobotController::enableGoalSetting(bool enable)
{
    // 如果已经在所需的模式，不做任何改变
    if ((enable && interaction_mode_ == InteractionMode::SET_GOAL) ||
        (!enable && interaction_mode_ != InteractionMode::SET_GOAL)) {
        return;
    }

    if (enable) {
        // 如果当前在初始位姿设置模式，先取消它
        if (interaction_mode_ == InteractionMode::SET_INITIAL_POSE) {
            ROS_INFO("Canceling initial pose setting");
        }
        
        // 切换到目标点设置工具
        std_msgs::String tool_msg;
        tool_msg.data = "rviz/SetGoal";
        tool_manager_pub_.publish(tool_msg);
        interaction_mode_ = InteractionMode::SET_GOAL;
        ROS_INFO("Switched to goal tool");
    } else {
        // 切换回默认工具
        std_msgs::String tool_msg;
        tool_msg.data = "rviz/MoveCamera";
        tool_manager_pub_.publish(tool_msg);
        interaction_mode_ = InteractionMode::NONE;
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
        // 使用当前时间戳
        geometry_msgs::PoseWithCovarianceStamped current_pose = *msg;
        current_pose.header.stamp = ros::Time::now();
        
        // 发布初始位姿到AMCL
        initial_pose_pub_.publish(current_pose);
        ROS_INFO("Published initial pose");
        
        // 更新当前位姿
        current_amcl_pose_ = current_pose;
        current_pose_ = current_pose.pose.pose;
        
        // 重置所有相关状态
        is_localized_ = true;
        is_localizing_ = false;
        localization_progress_ = 100.0;
        
        // 如果正在导航，取消导航
        if (is_navigating_) {
            stopNavigation();
        }
        
        // 计算yaw角（弧度转角度）
        double yaw = tf2::getYaw(current_pose.pose.pose.orientation) * 180.0 / M_PI;
        
        // 发送位姿更新信号
        emit poseUpdated(current_pose.pose);
        emit localizationStateChanged(true);
        emit localizationStatusChanged(QString("初始位姿已设置: (%.3f, %.3f) 朝向%.1f°")
            .arg(current_pose_.position.x, 0, 'f', 3)
            .arg(current_pose_.position.y, 0, 'f', 3)
            .arg(yaw, 0, 'f', 1));
        
        // 重新启用设置初始位姿按钮
        emit localizationProgressChanged(100.0);
        
        // 切换回默认工具
        enableInitialPoseSetting(false);
        
        // 清除代价地图
        std_srvs::Empty srv;
        if (clear_costmaps_client_.call(srv)) {
            ROS_INFO("Costmaps cleared after setting initial pose");
        }
        
        // 重置导航相关状态
        current_goal_ = geometry_msgs::PoseStamped();  // 清空当前目标点
        initial_distance_ = 0.0;
        distance_to_goal_ = 0.0;
        
        // 启动自动定位
        startAutoLocalization();
        
        ROS_INFO("Initial pose set and auto localization started");
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in initialPoseCallback: %s", e.what());
    } catch (...) {
        ROS_ERROR("Unknown exception in initialPoseCallback");
    }
}

void RobotController::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (!is_localized_) {
        ROS_WARN("Cannot set goal: robot not localized");
        emit navigationStatusChanged("请先设置机器人初始位置！");
        return;
    }

    if (is_localizing_) {
        ROS_WARN("Cannot set goal: robot is localizing");
        emit navigationStatusChanged("请等待定位完成！");
        return;
    }
    
    // 保存当前目标点
    current_goal_ = *msg;
    
    // 计算目标点的yaw角（弧度转角度）
    double yaw = tf2::getYaw(msg->pose.orientation) * 180.0 / M_PI;
    
    // 发送导航目标
    setNavigationGoal(current_goal_);
    
    emit navigationStatusChanged(QString("导航目标已设置: (%.3f, %.3f) 朝向%.1f°")
        .arg(msg->pose.position.x, 0, 'f', 3)
        .arg(msg->pose.position.y, 0, 'f', 3)
        .arg(yaw, 0, 'f', 1));
    
    // 切换回默认工具
    enableGoalSetting(false);
    
    ROS_INFO("Goal set and navigation started");
}

void RobotController::startAutoLocalization()
{
    if (is_localizing_) {
        ROS_WARN("Auto localization already in progress");
        return;
    }
    
    if (is_navigating_) {
        ROS_INFO("Canceling navigation before starting auto localization");
        stopNavigation();
        ros::Duration(0.5).sleep();  // 等待导航停止
    }
    
    // 重置定位状态
    is_localized_ = false;
    is_localizing_ = true;
    localization_progress_ = 0.0;
    
    // 清除当前目标点
    current_goal_ = geometry_msgs::PoseStamped();
    initial_distance_ = 0.0;
    distance_to_goal_ = 0.0;
    
    // 清除代价地图
    std_srvs::Empty srv;
    if (!clear_costmaps_client_.call(srv)) {
        ROS_WARN("Failed to clear costmaps");
    }
    
    // 发布全局定位请求
    std_msgs::Bool global_loc_msg;
    global_loc_msg.data = true;
    global_localization_pub_.publish(global_loc_msg);
    
    // 设置AMCL参数以进行全局定位
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::Config conf;

    // 创建参数
    dynamic_reconfigure::IntParameter int_param;
    dynamic_reconfigure::DoubleParameter double_param;

    // 设置粒子数量
    int_param.name = "min_particles";
    int_param.value = 5000;
    conf.ints.push_back(int_param);
    
    int_param.name = "max_particles";
    int_param.value = 10000;
    conf.ints.push_back(int_param);
    
    // 设置更新阈值
    double_param.name = "update_min_d";
    double_param.value = 0.1;
    conf.doubles.push_back(double_param);
    
    double_param.name = "update_min_a";
    double_param.value = 0.1;
    conf.doubles.push_back(double_param);
    
    // 设置激光参数
    double_param.name = "laser_max_range";
    double_param.value = 20.0;
    conf.doubles.push_back(double_param);

    int_param.name = "laser_max_beams";
    int_param.value = 100;
    conf.ints.push_back(int_param);

    double_param.name = "laser_z_hit";
    double_param.value = 0.95;
    conf.doubles.push_back(double_param);

    double_param.name = "laser_z_rand";
    double_param.value = 0.05;
    conf.doubles.push_back(double_param);

    double_param.name = "recovery_alpha_slow";
    double_param.value = 0.001;
    conf.doubles.push_back(double_param);

    double_param.name = "recovery_alpha_fast";
    double_param.value = 0.1;
    conf.doubles.push_back(double_param);

    srv_req.config = conf;

    // 调用服务设置参数
    if (!ros::service::call("/amcl/set_parameters", srv_req, srv_resp)) {
        ROS_WARN("Failed to set AMCL parameters");
    }

    // 启动定位监控定时器
    localization_monitor_timer_ = nh_.createTimer(
        ros::Duration(0.5), 
        &RobotController::monitorLocalization,
        this
    );
    
    emit localizationStateChanged(false);
    emit localizationProgressChanged(0.0);
    emit localizationStatusChanged("开始自动定位...");
    
    ROS_INFO("Auto localization started with enhanced parameters");
}

void RobotController::monitorLocalization(const ros::TimerEvent&)
{
    if (!is_localizing_) {
        localization_monitor_timer_.stop();
        return;
    }

    if (!current_amcl_pose_.header.stamp.isZero()) {
        // 计算粒子云的协方差来评估定位质量
        double position_variance = 
            current_pose_cov_[0] + current_pose_cov_[7];  // x和y方向的方差之和
        double orientation_variance = current_pose_cov_[35]; // yaw方向的方差
        
        // 如果方差小于阈值,认为定位成功
        if (position_variance < 0.1 && orientation_variance < 0.1) {
            is_localized_ = true;
            is_localizing_ = false;
            localization_monitor_timer_.stop();
            
            // 恢复正常的AMCL参数
            dynamic_reconfigure::ReconfigureRequest srv_req;
            dynamic_reconfigure::ReconfigureResponse srv_resp;
            dynamic_reconfigure::DoubleParameter double_param;
            dynamic_reconfigure::Config conf;
            
            double_param.name = "min_particles";
            double_param.value = 100;
            conf.doubles.push_back(double_param);
            
            double_param.name = "max_particles";
            double_param.value = 5000;
            conf.doubles.push_back(double_param);
            
            double_param.name = "update_min_d";
            double_param.value = 0.2;  // 恢复默认值
            conf.doubles.push_back(double_param);
            
            double_param.name = "update_min_a";
            double_param.value = 0.5;  // 恢复默认值
            conf.doubles.push_back(double_param);
            
            srv_req.config = conf;
            ros::service::call("/amcl/set_parameters", srv_req, srv_resp);
            
            // 清除代价地图
            std_srvs::Empty srv;
            if (clear_costmaps_client_.call(srv)) {
                ROS_INFO("Costmaps cleared after successful localization");
            }
            
            emit localizationStateChanged(true);
            emit localizationProgressChanged(100.0);
            emit localizationStatusChanged("定位成功");
        } else {
            // 根据方差计算定位进度
            double progress = std::max(0.0, std::min(100.0, 
                (1.0 - position_variance/2.0) * 100.0));
            emit localizationProgressChanged(progress);
            emit localizationStatusChanged(
                QString("正在定位 (位置误差: %1m, 角度误差: %2°)")
                .arg(sqrt(position_variance), 0, 'f', 2)
                .arg(sqrt(orientation_variance) * 180.0 / M_PI, 0, 'f', 1)
            );
        }
    }
}

void RobotController::setupSafety()
{
    // 订阅激光雷达数据
    laser_scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(
        "scan", 1, &RobotController::laserScanCallback, this);
        
    // 设置安全参数
    safety_distance_ = 0.5;  // 安全距离阈值(米)
    is_obstacle_detected_ = false;
}

void RobotController::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    // 检查前方180度范围内是否有障碍物
    int start_index = scan->ranges.size() / 4;  // 左侧45度
    int end_index = scan->ranges.size() * 3 / 4;  // 右侧45度
    
    bool obstacle_detected = false;
    for (int i = start_index; i < end_index; i++) {
        if (scan->ranges[i] < safety_distance_ && 
            scan->ranges[i] > scan->range_min) {
            obstacle_detected = true;
            break;
        }
    }
    
    if (obstacle_detected != is_obstacle_detected_) {
        is_obstacle_detected_ = obstacle_detected;
        if (obstacle_detected) {
            // 发现障碍物时停止运动
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            cmd_vel_pub_.publish(cmd_vel);
            
            emit localizationStatusChanged("检测到障碍物,已停止运动");
        }
    }
}

void RobotController::navigationDoneCallback(const actionlib::SimpleClientGoalState& state,
                                          const move_base_msgs::MoveBaseResultConstPtr& result)
{
    is_navigating_ = false;
    
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Navigation succeeded");
        emit navigationStateChanged(NavigationState::SUCCEEDED);
        emit navigationStatusChanged("导航成功到达目标点");
        emit navigationProgressChanged(100.0);
    } else {
        ROS_WARN("Navigation failed: %s", state.toString().c_str());
        QString error_msg;
        if (state == actionlib::SimpleClientGoalState::ABORTED) {
            error_msg = "导航失败：无法找到有效路径或遇到障碍物";
        } else if (state == actionlib::SimpleClientGoalState::REJECTED) {
            error_msg = "导航失败：目标点无效或不可达";
        } else if (state == actionlib::SimpleClientGoalState::PREEMPTED) {
            error_msg = "导航已取消";
        } else {
            error_msg = QString("导航失败：%1").arg(state.toString().c_str());
        }
        emit navigationStateChanged(NavigationState::FAILED);
        emit navigationStatusChanged(error_msg);
        emit navigationProgressChanged(0.0);
        
        // 清除代价地图
        std_srvs::Empty srv;
        if (clear_costmaps_client_.call(srv)) {
            ROS_INFO("Costmaps cleared after navigation failure");
        }
    }
}

void RobotController::navigationActiveCallback()
{
    ROS_INFO("Navigation active");
    is_navigating_ = true;
    emit navigationStateChanged(NavigationState::NAVIGATING);
    emit navigationStatusChanged("正在导航...");
    emit navigationProgressChanged(0.0);
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
            emit navigationStatusChanged("导航似乎卡住了，尝试重新规划路径...");
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
    emit navigationStatusChanged(QString("距离目标点还有 %.2f 米 (进度: %.1f%%)")
        .arg(distance, 0, 'f', 2)
        .arg(progress, 0, 'f', 1));
    
    // 保存当前距离
    distance_to_goal_ = distance;
} 