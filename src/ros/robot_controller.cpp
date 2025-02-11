/**
 * Copyright (c) 2024 JIAlonglong
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
 * 
 * @file robot_controller.cpp
 * @brief 机器人控制器类的实现
 * @author JIAlonglong
 */

#include "robot_controller.h"
#include <QDebug>
#include <QTimer>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

struct RobotController::Private {
    ros::NodeHandle                                                nh;
    ros::Publisher                                                cmd_vel_pub;
    ros::Publisher                                                initial_pose_pub;
    ros::Subscriber                                               odom_sub;
    ros::Subscriber                                               amcl_pose_sub;
    ros::Subscriber                                               laser_scan_sub;
    ros::Subscriber                                               battery_sub;
    ros::Subscriber                                               map_sub;
    ros::Subscriber                                               path_sub;
    std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> move_base_client;
    
    QString              robot_state{"就绪"};
    QString              navigation_state{"空闲"};
    QString              localization_state{"未定位"};
    QString              mapping_state{"未启动"};
    QString              battery_status{"正常"};
    double              battery_level{100.0};
    double              battery_voltage{12.0};
    double              battery_current{0.0};
    double              battery_temperature{25.0};
    double              linear_velocity{0.0};
    double              angular_velocity{0.0};
    bool                is_connected{false};
    bool                auto_localization_enabled{false};
    ros::Subscriber     localization_quality_sub;
    QTimer*             velocity_timer{nullptr};
    geometry_msgs::Twist current_cmd_vel;
};

RobotController::RobotController(QObject* parent) : QObject(parent), d_(new Private)
{
    // 初始化ROS发布者和订阅者
    d_->cmd_vel_pub = d_->nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    d_->initial_pose_pub = d_->nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
    d_->odom_sub = d_->nh.subscribe("odom", 1, &RobotController::odomCallback, this);
    d_->amcl_pose_sub = d_->nh.subscribe("amcl_pose", 1, &RobotController::amclPoseCallback, this);
    d_->laser_scan_sub = d_->nh.subscribe("scan", 1, &RobotController::laserScanCallback, this);
    d_->battery_sub = d_->nh.subscribe("battery_state", 1, &RobotController::batteryCallback, this);
    d_->map_sub = d_->nh.subscribe("map", 1, &RobotController::mapCallback, this);
    d_->path_sub = d_->nh.subscribe("move_base/NavfnROS/plan", 1, &RobotController::pathCallback, this);
    
    // 初始化move_base动作客户端
    d_->move_base_client = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
    
    // 初始化速度发布定时器
    d_->velocity_timer = new QTimer(this);
    connect(d_->velocity_timer, &QTimer::timeout, this, &RobotController::publishVelocity);
    d_->velocity_timer->setInterval(100);  // 10Hz
}

RobotController::~RobotController()
{
    if (d_->velocity_timer) {
        d_->velocity_timer->stop();
        delete d_->velocity_timer;
    }
    stopRobot();
}

bool RobotController::initialize()
{
    // 等待move_base服务
    if (!d_->move_base_client->waitForServer(ros::Duration(5.0))) {
        emit error("无法连接到move_base服务");
        return false;
    }
    
    // 检查传感器状态
    if (!d_->laser_scan_sub) {
        emit error("无法订阅激光扫描数据");
        return false;
    }
    
    if (!d_->odom_sub) {
        emit error("无法订阅里程计数据");
        return false;
    }
    
    d_->is_connected = true;
    d_->robot_state = "就绪";
    emit connected();
    emit robotStateChanged(d_->robot_state);
    
    return true;
}

void RobotController::cleanup()
{
    stopRobot();
    d_->is_connected = false;
    d_->robot_state = "未连接";
    emit disconnected();
    emit robotStateChanged(d_->robot_state);
}

void RobotController::navigateTo(double x, double y, double theta, double tolerance)
{
    if (!d_->is_connected) {
        emit error("机器人未连接");
        return;
    }
    
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
    
    move_base_msgs::MoveBaseGoal mb_goal;
    mb_goal.target_pose = goal;
    
    d_->move_base_client->sendGoal(
        mb_goal,
        boost::bind(&RobotController::navigationDoneCallback, this, _1, _2),
        boost::bind(&RobotController::navigationActiveCallback, this),
        boost::bind(&RobotController::navigationFeedbackCallback, this, _1)
    );
    
    d_->robot_state = "导航中";
    emit robotStateChanged(d_->robot_state);
    emit navigationStarted();
}

void RobotController::stopNavigation()
{
    if (!d_->is_connected) {
        return;
    }
    
    d_->move_base_client->cancelGoal();
    d_->robot_state = "就绪";
    emit robotStateChanged(d_->robot_state);
    emit navigationStopped();
}

void RobotController::setVelocity(double linear, double angular)
{
    if (!d_->is_connected) {
        return;
    }
    
    d_->current_cmd_vel.linear.x = linear;
    d_->current_cmd_vel.angular.z = angular;
    
    if (!d_->velocity_timer->isActive()) {
        d_->velocity_timer->start();
    }
    
    emit linearVelocityChanged(linear);
    emit angularVelocityChanged(angular);
}

void RobotController::stopRobot()
{
    setVelocity(0.0, 0.0);
    d_->velocity_timer->stop();
}

void RobotController::enableAutoLocalization(bool enable)
{
    if (!d_->is_connected) {
        return;
    }
    
    d_->auto_localization_enabled = enable;
    if (enable) {
        d_->localization_quality_sub = d_->nh.subscribe(
            "amcl/particle_cloud", 1,
            &RobotController::localizationQualityCallback, this);
    } else {
        d_->localization_quality_sub.shutdown();
    }
}

void RobotController::publishVelocity()
{
    if (!d_->is_connected) {
        return;
    }
    
    d_->cmd_vel_pub.publish(d_->current_cmd_vel);
}

void RobotController::navigationDoneCallback(
    const actionlib::SimpleClientGoalState& state,
    const move_base_msgs::MoveBaseResultConstPtr& result)
{
    Q_UNUSED(result);
    
    bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
    d_->robot_state = success ? "就绪" : "导航失败";
    emit robotStateChanged(d_->robot_state);
    emit navigationCompleted(success);
}

void RobotController::navigationActiveCallback()
{
    d_->robot_state = "导航中";
    emit robotStateChanged(d_->robot_state);
}

void RobotController::navigationFeedbackCallback(
    const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
    // 计算导航进度
    double dx = feedback->base_position.pose.position.x - d_->current_cmd_vel.linear.x;
    double dy = feedback->base_position.pose.position.y - d_->current_cmd_vel.linear.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    emit navigationProgress(std::max(0.0, std::min(100.0, (1.0 - distance / 5.0) * 100.0)));
}

void RobotController::amclPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    emit poseUpdated(msg);
}

void RobotController::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    emit scanUpdated(msg);
}

void RobotController::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    emit mapUpdated(msg);
}

void RobotController::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    emit pathUpdated(msg);
}

void RobotController::batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
    d_->battery_level = msg->percentage;
    d_->battery_voltage = msg->voltage;
    d_->battery_current = msg->current;
    d_->battery_temperature = msg->temperature;
    
    QString status;
    if (msg->power_supply_status == sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING) {
        status = "充电中";
    } else if (msg->power_supply_status == sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING) {
        status = "放电中";
    } else if (msg->power_supply_status == sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL) {
        status = "已充满";
    } else {
        status = "未知";
    }
    
    if (d_->battery_status != status) {
        d_->battery_status = status;
        emit batteryStatusChanged(status);
    }
    
    emit batteryLevelChanged(d_->battery_level);
    emit batteryVoltageChanged(d_->battery_voltage);
    emit batteryCurrentChanged(d_->battery_current);
    emit batteryTemperatureChanged(d_->battery_temperature);
}

void RobotController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    d_->linear_velocity = msg->twist.twist.linear.x;
    d_->angular_velocity = msg->twist.twist.angular.z;
    
    emit linearVelocityChanged(d_->linear_velocity);
    emit angularVelocityChanged(d_->angular_velocity);
}

// Getter方法实现
bool RobotController::isConnected() const { return d_->is_connected; }
QString RobotController::robotState() const { return d_->robot_state; }
QString RobotController::navigationState() const { return d_->navigation_state; }
QString RobotController::localizationState() const { return d_->localization_state; }
QString RobotController::mappingState() const { return d_->mapping_state; }
QString RobotController::batteryStatus() const { return d_->battery_status; }
double RobotController::batteryLevel() const { return d_->battery_level; }
double RobotController::batteryVoltage() const { return d_->battery_voltage; }
double RobotController::batteryCurrent() const { return d_->battery_current; }
double RobotController::batteryTemperature() const { return d_->battery_temperature; }
double RobotController::linearVelocity() const { return d_->linear_velocity; }
double RobotController::angularVelocity() const { return d_->angular_velocity; }

#include "robot_controller.moc"