/**
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
 * 
 * @file path_action_block.cpp
 * @brief 动作块基类,定义动作的基本接口
 * @author JIAlonglong
 */

#include "path_action_block.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <QDebug>
#include <QIcon>
#include <QJsonArray>
#include <QJsonObject>
#include <QTimer>
#include <cmath>
#include "robot_controller.h"

PathActionBlock::PathActionBlock(std::shared_ptr<RobotController> robot_controller, QObject* parent)
    : ActionBlock(parent), robot_controller_(robot_controller)
{
    connect(robot_controller_.get(), &RobotController::navigationCompleted, this,
            &PathActionBlock::onNavigationCompleted);
    connect(robot_controller_.get(), &RobotController::navigationFeedback, this,
            &PathActionBlock::onNavigationFeedback);
}

PathActionBlock::~PathActionBlock() = default;

bool PathActionBlock::isValid() const
{
    return !waypoints_.isEmpty();
}

QIcon PathActionBlock::icon() const
{
    return QIcon(":/icons/path.png");
}

QList<QPointF> PathActionBlock::waypoints() const
{
    return waypoints_;
}

void PathActionBlock::setWaypoints(const QList<QPointF>& waypoints)
{
    if (waypoints_ == waypoints) {
        return;
    }
    waypoints_ = waypoints;
    Q_EMIT waypointsChanged();
}

void PathActionBlock::addWaypoint(const QPointF& point)
{
    waypoints_.append(point);
    Q_EMIT waypointsChanged();
}

void PathActionBlock::removeWaypoint(int index)
{
    if (index >= 0 && index < waypoints_.size()) {
        waypoints_.removeAt(index);
        Q_EMIT waypointsChanged();
    }
}

void PathActionBlock::clearWaypoints()
{
    if (!waypoints_.isEmpty()) {
        waypoints_.clear();
        Q_EMIT waypointsChanged();
    }
}

QVariantMap PathActionBlock::properties() const
{
    QVariantList points;
    for (const auto& point : waypoints_) {
        points.append(QVariantMap{{"x", point.x()}, {"y", point.y()}});
    }
    return {{"waypoints", points}};
}

void PathActionBlock::setProperty(const QString& name, const QVariant& value)
{
    if (name == "waypoints") {
        QList<QPointF> points;
        const auto     list = value.toList();
        for (const auto& item : list) {
            const auto map = item.toMap();
            points.append(QPointF(map["x"].toDouble(), map["y"].toDouble()));
        }
        setWaypoints(points);
    }
}

QJsonObject PathActionBlock::toJson() const
{
    QJsonArray points;
    for (const auto& point : waypoints_) {
        points.append(QJsonObject{{"x", point.x()}, {"y", point.y()}});
    }
    return QJsonObject{{"type", type()}, {"waypoints", points}};
}

void PathActionBlock::fromJson(const QJsonObject& json)
{
    QList<QPointF> points;
    const auto     array = json["waypoints"].toArray();
    for (const auto& value : array) {
        const auto obj = value.toObject();
        points.append(QPointF(obj["x"].toDouble(), obj["y"].toDouble()));
    }
    setWaypoints(points);
}

void PathActionBlock::execute()
{
    if (waypoints_.isEmpty()) {
        Q_EMIT failed(tr("路径点为空"));
        return;
    }

    if (!robot_controller_) {
        Q_EMIT failed(tr("机器人控制器未初始化"));
        return;
    }

    is_running_       = true;
    current_waypoint_ = 0;
    Q_EMIT started();
    moveToNextWaypoint();
}

void PathActionBlock::stop()
{
    if (!robot_controller_ || !is_running_) {
        return;
    }

    robot_controller_->stopNavigation();
    is_running_       = false;
    current_waypoint_ = -1;
    Q_EMIT stopped();
}

void PathActionBlock::pause()
{
    if (!robot_controller_ || !is_running_) {
        return;
    }

    robot_controller_->pauseNavigation();
    Q_EMIT paused();
}

void PathActionBlock::resume()
{
    if (!robot_controller_ || !is_running_) {
        return;
    }

    robot_controller_->resumeNavigation();
    Q_EMIT resumed();
}

void PathActionBlock::reset()
{
    stop();
    clearWaypoints();
}

void PathActionBlock::onNavigationCompleted(bool success)
{
    if (!is_running_) {
        return;
    }

    if (success) {
        current_waypoint_++;
        if (current_waypoint_ < waypoints_.size()) {
            moveToNextWaypoint();
        } else {
            is_running_       = false;
            current_waypoint_ = -1;
            Q_EMIT completed(true);
        }
    } else {
        is_running_       = false;
        current_waypoint_ = -1;
        Q_EMIT failed(tr("导航失败"));
    }
}

void PathActionBlock::onNavigationFeedback(const QString& status)
{
    Q_EMIT stateChanged(status);
}

void PathActionBlock::moveToNextWaypoint()
{
    if (!robot_controller_) {
        Q_EMIT failed(tr("机器人控制器未初始化"));
        return;
    }

    if (current_waypoint_ >= waypoints_.size()) {
        is_running_       = false;
        current_waypoint_ = -1;
        Q_EMIT completed(true);
        return;
    }

    const auto& point = waypoints_[current_waypoint_];
    robot_controller_->navigateTo(point.x(), point.y(), 0.0, 0.1);
}

#include "path_action_block.moc"