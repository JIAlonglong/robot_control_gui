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
#include <amcl/AMCLConfig.h>
#include <rviz/visualization_manager.h>
#include <rviz/tool_manager.h>
#include <rviz/tool.h>

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
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
    tool_manager_pub_ = nh_.advertise<std_msgs::String>("/rviz/current_tool", 1);
    global_localization_pub_ = nh_.advertise<std_msgs::Bool>("/global_localization", 1);
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
        
        emit poseUpdated(pose.pose);
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

    // 发送导航目标
    move_base_client_->sendGoal(
        move_base_goal,
        boost::bind(&RobotController::navigationDoneCallback, this, _1, _2),
        boost::bind(&RobotController::navigationActiveCallback, this),
        boost::bind(&RobotController::navigationFeedbackCallback, this, _1)
    );
    
    is_navigating_ = true;
    ROS_INFO("Navigation goal sent");
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

    static ros::Time last_callback_time = ros::Time(0);
    ros::Time current_time = ros::Time::now();
    
    if ((current_time - last_callback_time).toSec() < 0.5) {
        return;
    }
    last_callback_time = current_time;

    try {
        geometry_msgs::PoseWithCovarianceStamped current_pose = *msg;
        current_pose.header.stamp = current_time;
        
        initial_pose_pub_.publish(current_pose);
        ROS_INFO("Published initial pose");
        
        current_amcl_pose_ = current_pose;
        current_pose_ = current_pose.pose.pose;
        
        double yaw = tf2::getYaw(current_pose.pose.pose.orientation) * 180.0 / M_PI;
        
        emit poseUpdated(current_pose.pose);
        
        QString status_msg = QString("当前位置: (%1, %2) 朝向%3°")
            .arg(current_pose_.position.x, 0, 'f', 3)
            .arg(current_pose_.position.y, 0, 'f', 3)
            .arg(yaw, 0, 'f', 1);
        emit localizationStatusChanged(status_msg);
        
        std_srvs::Empty srv;
        if (clear_costmaps_client_.call(srv)) {
            ROS_INFO("Costmaps cleared after setting initial pose");
        }
        
        // 切换回移动相机工具
        interaction_mode_ = InteractionMode::NONE;
        std_msgs::String tool_msg;
        tool_msg.data = "rviz/MoveCamera";
        tool_manager_pub_.publish(tool_msg);
        
        ROS_INFO("Initial pose set");
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in initialPoseCallback: %s", e.what());
    } catch (...) {
        ROS_ERROR("Unknown exception in initialPoseCallback");
    }
}

void RobotController::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_goal_ = *msg;
    
    double yaw = tf2::getYaw(msg->pose.orientation) * 180.0 / M_PI;
    
    setNavigationGoal(current_goal_);
    
    QString status_msg = QString("导航目标: (%1, %2) 朝向%3°")
        .arg(msg->pose.position.x, 0, 'f', 3)
        .arg(msg->pose.position.y, 0, 'f', 3)
        .arg(yaw, 0, 'f', 1);
    emit navigationStatusChanged(status_msg);
    
    ROS_INFO("Goal set and navigation started");
}

void RobotController::startAutoLocalization()
{
    if (is_navigating_) {
        ROS_INFO("Canceling navigation before starting auto localization");
        stopNavigation();
        ros::Duration(0.5).sleep();
    }
    
    localization_progress_ = 0.0;
    
    std_srvs::Empty srv;
    if (!clear_costmaps_client_.call(srv)) {
        ROS_WARN("Failed to clear costmaps");
    }
    
    std_msgs::Bool global_loc_msg;
    global_loc_msg.data = true;
    global_localization_pub_.publish(global_loc_msg);

    // 使用dynamic_reconfigure服务来设置参数
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::Config conf;

    // 设置整数参数
    dynamic_reconfigure::IntParameter int_param;
    int_param.name = "min_particles";
    int_param.value = 5000;
    conf.ints.push_back(int_param);

    int_param.name = "max_particles";
    int_param.value = 10000;
    conf.ints.push_back(int_param);

    int_param.name = "resample_interval";
    int_param.value = 1;
    conf.ints.push_back(int_param);

    // 设置浮点数参数
    dynamic_reconfigure::DoubleParameter double_param;
    double_param.name = "update_min_d";
    double_param.value = 0.1;
    conf.doubles.push_back(double_param);

    double_param.name = "update_min_a";
    double_param.value = 0.1;
    conf.doubles.push_back(double_param);

    double_param.name = "kld_err";
    double_param.value = 0.01;
    conf.doubles.push_back(double_param);

    double_param.name = "kld_z";
    double_param.value = 0.99;
    conf.doubles.push_back(double_param);

    double_param.name = "recovery_alpha_slow";
    double_param.value = 0.001;
    conf.doubles.push_back(double_param);

    double_param.name = "recovery_alpha_fast";
    double_param.value = 0.1;
    conf.doubles.push_back(double_param);

    srv_req.config = conf;

    if (!ros::service::call("/amcl/set_parameters", srv_req, srv_resp)) {
        ROS_WARN("Failed to set AMCL parameters");
    }

    localization_monitor_timer_ = nh_.createTimer(
        ros::Duration(0.5), 
        &RobotController::monitorLocalization,
        this
    );
    
    emit localizationProgressChanged(0.0);
    emit localizationStatusChanged("开始自动定位...");
    
    ROS_INFO("Auto localization started");
}

void RobotController::monitorLocalization(const ros::TimerEvent&)
{
    if (!current_amcl_pose_.header.stamp.isZero()) {
        // 计算粒子云的协方差来评估定位质量
        double position_variance = 
            current_pose_cov_[0] + current_pose_cov_[7];  // x和y方向的方差之和
        double orientation_variance = current_pose_cov_[35]; // yaw方向的方差
        
        // 如果方差小于阈值,认为定位成功
        if (position_variance < 0.1 && orientation_variance < 0.1) {
            localization_monitor_timer_.stop();
            
            // 恢复正常的AMCL参数
            dynamic_reconfigure::ReconfigureRequest srv_req;
            dynamic_reconfigure::ReconfigureResponse srv_resp;
            dynamic_reconfigure::Config conf;
            
            // 设置整数参数
            dynamic_reconfigure::IntParameter int_param;
            int_param.name = "min_particles";
            int_param.value = 100;
            conf.ints.push_back(int_param);
            
            int_param.name = "max_particles";
            int_param.value = 5000;
            conf.ints.push_back(int_param);
            
            // 设置浮点数参数
            dynamic_reconfigure::DoubleParameter double_param;
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
    is_navigating_ = true;
    emit navigationStateChanged(NavigationState::ACTIVE);
    emit navigationStatusChanged("正在导航到目标点...");
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