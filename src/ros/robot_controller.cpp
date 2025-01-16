/**
 * @file robot_controller.cpp
 * @brief ROS通信管理类实现
 */

#include "ros/robot_controller.h"
#include <ros/master.h>
#include <QDebug>

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
            emit localizationStateChanged(true);
            emit localizationProgressChanged(0.0);
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
        ROS_WARN("Move base action server not available");
        return;
    }

    if (!is_navigating_) {
        is_navigating_ = true;
        navigation_progress_ = 0.0;
        distance_to_goal_ = 0.0;
        estimated_time_to_goal_ = 0.0;
        emit navigationStateChanged(true);
        emit navigationProgressChanged(0.0);
        emit distanceToGoalChanged(0.0);
        emit estimatedTimeToGoalChanged(0.0);
    }
}

void RobotController::pauseNavigation()
{
    if (move_base_client_ && move_base_client_->isServerConnected() && is_navigating_) {
        move_base_client_->cancelGoal();
        is_navigating_ = false;
        emit navigationStateChanged(false);
    }
}

void RobotController::stopNavigation()
{
    if (move_base_client_ && move_base_client_->isServerConnected()) {
        move_base_client_->cancelAllGoals();
        is_navigating_ = false;
        navigation_progress_ = 0.0;
        distance_to_goal_ = 0.0;
        estimated_time_to_goal_ = 0.0;
        emit navigationStateChanged(false);
        emit navigationProgressChanged(0.0);
        emit distanceToGoalChanged(0.0);
        emit estimatedTimeToGoalChanged(0.0);
    }
}

void RobotController::cancelNavigation()
{
    stopNavigation();
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
    // 更新定位位姿
    current_amcl_pose_ = msg->pose.pose;
    emit poseUpdated(msg->pose);
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
    if (!move_base_client_ || !move_base_client_->isServerConnected()) {
        ROS_WARN("Move base action server not available");
        return;
    }

    move_base_msgs::MoveBaseGoal move_base_goal;
    move_base_goal.target_pose = goal;

    // 设置回调函数
    move_base_client_->sendGoal(
        move_base_goal,
        [this](const actionlib::SimpleClientGoalState& state,
               const move_base_msgs::MoveBaseResultConstPtr& result) {
            // 目标完成回调
            is_navigating_ = false;
            navigation_progress_ = 1.0;
            distance_to_goal_ = 0.0;
            estimated_time_to_goal_ = 0.0;
            emit navigationStateChanged(false);
            emit navigationProgressChanged(1.0);
            emit distanceToGoalChanged(0.0);
            emit estimatedTimeToGoalChanged(0.0);
        },
        [this]() {
            // 目标激活回调
            is_navigating_ = true;
            navigation_progress_ = 0.0;
            emit navigationStateChanged(true);
            emit navigationProgressChanged(0.0);
        },
        [this](const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
            // 反馈回调
            if (!is_navigating_) return;
            
            // 计算到目标的距离
            double dx = feedback->base_position.pose.position.x - 
                       feedback->base_position.pose.position.x;
            double dy = feedback->base_position.pose.position.y - 
                       feedback->base_position.pose.position.y;
            distance_to_goal_ = std::sqrt(dx * dx + dy * dy);
            
            // 估计剩余时间（假设平均速度为0.5m/s）
            estimated_time_to_goal_ = distance_to_goal_ / 0.5;
            
            // 更新进度（基于距离）
            navigation_progress_ = std::max(0.0, std::min(1.0, 1.0 - distance_to_goal_ / 10.0));
            
            emit navigationProgressChanged(navigation_progress_);
            emit distanceToGoalChanged(distance_to_goal_);
            emit estimatedTimeToGoalChanged(estimated_time_to_goal_);
        }
    );
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

void RobotController::setMasterURI(const std::string& uri)
{
    // 设置ROS_MASTER_URI环境变量
    setenv("ROS_MASTER_URI", uri.c_str(), 1);
    qDebug() << "ROS Master URI set to:" << QString::fromStdString(uri);
}

void RobotController::setHostname(const std::string& hostname)
{
    // 设置ROS_HOSTNAME环境变量
    setenv("ROS_HOSTNAME", hostname.c_str(), 1);
    qDebug() << "ROS Hostname set to:" << QString::fromStdString(hostname);
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