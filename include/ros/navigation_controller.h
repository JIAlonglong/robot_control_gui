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
 * @file navigation_controller.h
 * @brief 导航控制器类的头文件
 * @author JIAlonglong
 */

#pragma once

#ifndef NAVIGATION_CONTROLLER_H
#define NAVIGATION_CONTROLLER_H

#include <QObject>
#include <QString>
#include <QTimer>
#include <memory>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class NavigationController : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString navigationState READ navigationState NOTIFY navigationStateChanged)
    Q_PROPERTY(QString navigationStatus READ navigationStatus NOTIFY navigationStatusChanged)
    Q_PROPERTY(double navigationProgress READ navigationProgress NOTIFY navigationProgressChanged)

public:
    explicit NavigationController(QObject* parent = nullptr);
    ~NavigationController() override;

    // 导航控制
    void setGoal(const geometry_msgs::PoseStamped& goal);
    void stopNavigation();
    void pauseNavigation();
    void resumeNavigation();
    void setInitialPose(const geometry_msgs::PoseWithCovarianceStamped& pose);

    // 路径规划
    bool planPath(const geometry_msgs::PoseStamped& start,
                 const geometry_msgs::PoseStamped& goal);

    // 状态查询
    QString navigationState() const;
    QString navigationStatus() const;
    double  navigationProgress() const;

    // 导航参数设置
    void setGlobalPlanner(const QString& planner);
    void setLocalPlanner(const QString& planner);
    void setMaxVelocity(double velocity);
    void setMinVelocity(double velocity);
    void setMaxRotation(double rotation);
    void setMinRotation(double rotation);
    void setGoalTolerance(double tolerance);
    void setYawTolerance(double tolerance);

    // 路径规划参数设置
    void setInflationRadius(double radius);
    void setCostScalingFactor(double factor);
    void setPathDistanceBias(double bias);
    void setGoalDistanceBias(double bias);
    void setOccdistScale(double scale);

Q_SIGNALS:
    void navigationStateChanged(const QString& state);
    void navigationStatusChanged(const QString& status);
    void navigationProgressChanged(double progress);
    void navigationStarted();
    void navigationStopped();
    void navigationPaused();
    void navigationResumed();
    void navigationCompleted(bool success);
    void pathPlanned(const nav_msgs::Path& path);
    void pathUpdated(const nav_msgs::Path& path);
    void error(const QString& message);

private Q_SLOTS:
    void updateNavigationProgress();
    void navigationDoneCallback(const actionlib::SimpleClientGoalState& state,
                              const move_base_msgs::MoveBaseResultConstPtr& result);
    void navigationActiveCallback();
    void navigationFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);

private:
    struct Private;
    std::unique_ptr<Private> d_;
};

#endif  // NAVIGATION_CONTROLLER_H 