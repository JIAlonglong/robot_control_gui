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
 * @file navigation_controller.cpp
 * @brief 导航控制器类的实现,负责机器人导航功能
 * @author JIAlonglong
 */

#include "navigation_controller.h"
#include <QDebug>
#include <QTimer>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

struct NavigationController::Private {
    ros::NodeHandle nh;
    ros::Publisher  goal_pub;
    ros::Publisher  initial_pose_pub;
    ros::Subscriber path_sub;
    ros::ServiceClient get_plan_client;
    std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> move_base_client;
    
    QString navigation_state{"空闲"};
    QString navigation_status{"就绪"};
    double  navigation_progress{0.0};
    bool    is_navigating{false};
    bool    is_paused{false};
    
    // 导航参数
    QString global_planner{"navfn"};
    QString local_planner{"dwa"};
    double  max_velocity{0.5};
    double  min_velocity{0.1};
    double  max_rotation{1.0};
    double  min_rotation{0.1};
    double  goal_tolerance{0.1};
    double  yaw_tolerance{0.1};
    
    // 路径规划参数
    double inflation_radius{0.3};
    double cost_scaling_factor{10.0};
    double path_distance_bias{32.0};
    double goal_distance_bias{24.0};
    double occdist_scale{0.01};
    
    // 当前目标点
    geometry_msgs::PoseStamped current_goal;
    nav_msgs::Path             current_path;
    
    QTimer* progress_timer{nullptr};
};

NavigationController::NavigationController(QObject* parent)
    : QObject(parent), d_(new Private)
{
    // 初始化ROS通信
    d_->goal_pub = d_->nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    d_->initial_pose_pub = d_->nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
    d_->path_sub = d_->nh.subscribe("move_base/NavfnROS/plan", 1, &NavigationController::pathCallback, this);
    d_->get_plan_client = d_->nh.serviceClient<nav_msgs::GetPlan>("move_base/make_plan");
    
    // 初始化move_base客户端
    d_->move_base_client = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
    
    // 创建进度更新定时器
    d_->progress_timer = new QTimer(this);
    connect(d_->progress_timer, &QTimer::timeout, this, &NavigationController::updateNavigationProgress);
    d_->progress_timer->setInterval(100);  // 10Hz
}

NavigationController::~NavigationController()
{
    if (d_->is_navigating) {
        stopNavigation();
    }
    delete d_->progress_timer;
}

void NavigationController::setGoal(const geometry_msgs::PoseStamped& goal)
{
    if (d_->is_navigating) {
        stopNavigation();
    }
    
    d_->current_goal = goal;
    move_base_msgs::MoveBaseGoal mb_goal;
    mb_goal.target_pose = goal;
    
    // 设置导航参数
    ros::NodeHandle nh("move_base");
    nh.setParam("DWAPlannerROS/max_vel_x", d_->max_velocity);
    nh.setParam("DWAPlannerROS/min_vel_x", d_->min_velocity);
    nh.setParam("DWAPlannerROS/max_rot_vel", d_->max_rotation);
    nh.setParam("DWAPlannerROS/min_rot_vel", d_->min_rotation);
    nh.setParam("DWAPlannerROS/xy_goal_tolerance", d_->goal_tolerance);
    nh.setParam("DWAPlannerROS/yaw_goal_tolerance", d_->yaw_tolerance);
    
    // 发送导航目标
    d_->move_base_client->sendGoal(
        mb_goal,
        boost::bind(&NavigationController::navigationDoneCallback, this, _1, _2),
        boost::bind(&NavigationController::navigationActiveCallback, this),
        boost::bind(&NavigationController::navigationFeedbackCallback, this, _1)
    );
    
    d_->is_navigating = true;
    d_->navigation_state = "导航中";
    d_->navigation_progress = 0.0;
    d_->progress_timer->start();
    
    emit navigationStateChanged(d_->navigation_state);
    emit navigationStarted();
}

void NavigationController::stopNavigation()
{
    if (!d_->is_navigating) {
        return;
    }
    
    d_->move_base_client->cancelGoal();
    d_->progress_timer->stop();
    d_->is_navigating = false;
    d_->is_paused = false;
    d_->navigation_state = "已停止";
    d_->navigation_progress = 0.0;
    
    emit navigationStateChanged(d_->navigation_state);
    emit navigationProgressChanged(d_->navigation_progress);
    emit navigationStopped();
}

void NavigationController::pauseNavigation()
{
    if (!d_->is_navigating || d_->is_paused) {
        return;
    }
    
    d_->move_base_client->cancelGoal();
    d_->progress_timer->stop();
    d_->is_paused = true;
    d_->navigation_state = "已暂停";
    
    emit navigationStateChanged(d_->navigation_state);
    emit navigationPaused();
}

void NavigationController::resumeNavigation()
{
    if (!d_->is_paused) {
        return;
    }
    
    setGoal(d_->current_goal);
    d_->is_paused = false;
    d_->navigation_state = "导航中";
    
    emit navigationStateChanged(d_->navigation_state);
    emit navigationResumed();
}

void NavigationController::setInitialPose(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
    d_->initial_pose_pub.publish(pose);
}

bool NavigationController::planPath(const geometry_msgs::PoseStamped& start,
                                    const geometry_msgs::PoseStamped& goal)
{
    nav_msgs::GetPlan srv;
    srv.request.start = start;
    srv.request.goal = goal;
    srv.request.tolerance = d_->goal_tolerance;
    
    if (d_->get_plan_client.call(srv)) {
        emit pathPlanned(srv.response.plan);
        return true;
    }
    
    emit error("路径规划失败");
    return false;
}

void NavigationController::updateNavigationProgress()
{
    if (!d_->is_navigating || d_->is_paused) {
        return;
    }
    
    // 计算到目标点的距离
    tf2::Vector3 current_pos(d_->current_path.poses.back().pose.position.x,
                           d_->current_path.poses.back().pose.position.y,
                           0.0);
    tf2::Vector3 goal_pos(d_->current_goal.pose.position.x,
                        d_->current_goal.pose.position.y,
                        0.0);
    double distance = current_pos.distance(goal_pos);
    
    // 更新进度
    double progress = std::max(0.0, std::min(100.0, (1.0 - distance / 10.0) * 100.0));
    if (progress != d_->navigation_progress) {
        d_->navigation_progress = progress;
        emit navigationProgressChanged(progress);
    }
}

void NavigationController::navigationDoneCallback(
    const actionlib::SimpleClientGoalState& state,
    const move_base_msgs::MoveBaseResultConstPtr& result)
{
    d_->progress_timer->stop();
    d_->is_navigating = false;
    d_->is_paused = false;
    
    bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
    d_->navigation_state = success ? "已完成" : "失败";
    d_->navigation_progress = success ? 100.0 : 0.0;
    
    emit navigationStateChanged(d_->navigation_state);
    emit navigationProgressChanged(d_->navigation_progress);
    emit navigationCompleted(success);
}

void NavigationController::navigationActiveCallback()
{
    d_->navigation_status = "正在执行导航";
    emit navigationStatusChanged(d_->navigation_status);
}

void NavigationController::navigationFeedbackCallback(
    const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
    // 更新导航状态信息
    QString status = QString("距离目标点: %.2f米").arg(
        std::sqrt(feedback->base_position.pose.position.x * feedback->base_position.pose.position.x +
                 feedback->base_position.pose.position.y * feedback->base_position.pose.position.y));
    
    if (d_->navigation_status != status) {
        d_->navigation_status = status;
        emit navigationStatusChanged(status);
    }
}

void NavigationController::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    d_->current_path = *msg;
    emit pathUpdated(*msg);
}

// Getter方法实现
bool NavigationController::isNavigating() const { return d_->is_navigating; }
bool NavigationController::isPaused() const { return d_->is_paused; }
QString NavigationController::navigationState() const { return d_->navigation_state; }
QString NavigationController::navigationStatus() const { return d_->navigation_status; }
double NavigationController::navigationProgress() const { return d_->navigation_progress; }

// Setter方法实现
void NavigationController::setGlobalPlanner(const QString& planner) { d_->global_planner = planner; }
void NavigationController::setLocalPlanner(const QString& planner) { d_->local_planner = planner; }
void NavigationController::setMaxVelocity(double velocity) { d_->max_velocity = velocity; }
void NavigationController::setMinVelocity(double velocity) { d_->min_velocity = velocity; }
void NavigationController::setMaxRotation(double rotation) { d_->max_rotation = rotation; }
void NavigationController::setMinRotation(double rotation) { d_->min_rotation = rotation; }
void NavigationController::setGoalTolerance(double tolerance) { d_->goal_tolerance = tolerance; }
void NavigationController::setYawTolerance(double tolerance) { d_->yaw_tolerance = tolerance; }
void NavigationController::setInflationRadius(double radius) { d_->inflation_radius = radius; }
void NavigationController::setCostScalingFactor(double factor) { d_->cost_scaling_factor = factor; }
void NavigationController::setPathDistanceBias(double bias) { d_->path_distance_bias = bias; }
void NavigationController::setGoalDistanceBias(double bias) { d_->goal_distance_bias = bias; }
void NavigationController::setOccdistScale(double scale) { d_->occdist_scale = scale; }

#include "navigation_controller.moc" 