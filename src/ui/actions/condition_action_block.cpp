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
 * @file condition_action_block.cpp
 * @brief 动作块基类,定义动作的基本接口
 * @author JIAlonglong
 */

#include "condition_action_block.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <QDebug>
#include <QIcon>
#include <QJsonObject>
#include <cmath>
#include <functional>
#include "robot_controller.h"

struct ConditionActionBlock::Private {
    bool                             is_executing{false};
    bool                             condition_met{false};
    ActionBlock*                     current_action{nullptr};
    std::shared_ptr<RobotController> robot_controller;
    QString                          condition_type;
    double                           target_x{0.0};
    double                           target_y{0.0};
    double                           target_theta{0.0};
    double                           position_tolerance{0.1};
    double                           angle_tolerance{0.1};
    QString                          sensor_name;
    double                           sensor_threshold{0.0};
    bool                             is_running{false};
    ActionBlock*                     true_action{nullptr};
    ActionBlock*                     false_action{nullptr};
};

ConditionActionBlock::ConditionActionBlock(const std::shared_ptr<RobotController>& controller,
                                           QObject*                                parent)
    : ActionBlock(parent), d_(std::make_unique<Private>())
{
    d_->robot_controller = controller;

    if (!d_->robot_controller) {
        qWarning() << "ConditionActionBlock: 机器人控制器未初始化";
        return;
    }

    // 连接必要的信号和槽
    connect(d_->robot_controller.get(), &RobotController::obstacleDetected, this,
            &ConditionActionBlock::onObstacleDetected);
    connect(d_->robot_controller.get(), &RobotController::batteryLevelChanged, this,
            &ConditionActionBlock::onBatteryLevelChanged);

    connectSignals();
}

ConditionActionBlock::~ConditionActionBlock() = default;

QIcon ConditionActionBlock::icon() const
{
    return QIcon::fromTheme("dialog-question");
}

QVariantMap ConditionActionBlock::properties() const
{
    QVariantMap props;
    props["conditionType"]     = d_->condition_type;
    props["targetX"]           = d_->target_x;
    props["targetY"]           = d_->target_y;
    props["targetTheta"]       = d_->target_theta;
    props["positionTolerance"] = d_->position_tolerance;
    props["angleTolerance"]    = d_->angle_tolerance;
    props["sensorName"]        = d_->sensor_name;
    props["sensorThreshold"]   = d_->sensor_threshold;
    props["trueAction"]        = QVariant::fromValue(d_->true_action);
    props["falseAction"]       = QVariant::fromValue(d_->false_action);
    return props;
}

void ConditionActionBlock::setProperty(const QString& name, const QVariant& value)
{
    if (name == "condition_type") {
        setConditionType(value.toString());
    } else if (name == "target_x") {
        setTargetX(value.toDouble());
    } else if (name == "target_y") {
        setTargetY(value.toDouble());
    } else if (name == "target_theta") {
        setTargetTheta(value.toDouble());
    } else if (name == "position_tolerance") {
        setPositionTolerance(value.toDouble());
    } else if (name == "angle_tolerance") {
        setAngleTolerance(value.toDouble());
    } else if (name == "sensor_name") {
        setSensorName(value.toString());
    } else if (name == "sensor_threshold") {
        setSensorThreshold(value.toDouble());
    }
    Q_EMIT propertyChanged(name, value);
}

void ConditionActionBlock::setTrueAction(ActionBlock* action)
{
    if (d_->true_action == action) {
        return;
    }

    if (d_->true_action) {
        disconnect(d_->true_action, &ActionBlock::started, this,
                   &ConditionActionBlock::onActionStarted);
        disconnect(d_->true_action, &ActionBlock::completed, this,
                   &ConditionActionBlock::onActionCompleted);
        disconnect(d_->true_action, &ActionBlock::failed, this,
                   &ConditionActionBlock::onActionError);
    }

    d_->true_action = action;

    if (d_->true_action) {
        connect(d_->true_action, &ActionBlock::started, this,
                &ConditionActionBlock::onActionStarted);
        connect(d_->true_action, &ActionBlock::completed, this,
                &ConditionActionBlock::onActionCompleted);
        connect(d_->true_action, &ActionBlock::failed, this, &ConditionActionBlock::onActionError);
    }

    Q_EMIT trueActionChanged(action);
}

void ConditionActionBlock::setFalseAction(ActionBlock* action)
{
    if (d_->false_action == action) {
        return;
    }

    if (d_->false_action) {
        disconnect(d_->false_action, &ActionBlock::started, this,
                   &ConditionActionBlock::onActionStarted);
        disconnect(d_->false_action, &ActionBlock::completed, this,
                   &ConditionActionBlock::onActionCompleted);
        disconnect(d_->false_action, &ActionBlock::failed, this,
                   &ConditionActionBlock::onActionError);
    }

    d_->false_action = action;

    if (d_->false_action) {
        connect(d_->false_action, &ActionBlock::started, this,
                &ConditionActionBlock::onActionStarted);
        connect(d_->false_action, &ActionBlock::completed, this,
                &ConditionActionBlock::onActionCompleted);
        connect(d_->false_action, &ActionBlock::failed, this, &ConditionActionBlock::onActionError);
    }

    Q_EMIT falseActionChanged(action);
}

bool ConditionActionBlock::validateParameters() const
{
    if (d_->condition_type.isEmpty()) {
        qWarning() << "ConditionActionBlock: 条件类型未设置";
        return false;
    }

    if (d_->condition_type == "at_position") {
        if (d_->position_tolerance <= 0) {
            qWarning() << "ConditionActionBlock: 位置容差必须大于0";
            return false;
        }
    } else if (d_->condition_type == "at_orientation") {
        if (d_->angle_tolerance <= 0) {
            qWarning() << "ConditionActionBlock: 角度容差必须大于0";
            return false;
        }
    } else if (d_->condition_type == "sensor_threshold") {
        if (d_->sensor_name.isEmpty()) {
            qWarning() << "ConditionActionBlock: 传感器名称未设置";
            return false;
        }
    } else if (d_->condition_type == "obstacle_detected" || d_->condition_type == "battery_low") {
        // 这些条件类型不需要额外参数
        return true;
    } else {
        qWarning() << "ConditionActionBlock: 未知的条件类型:" << d_->condition_type;
        return false;
    }

    return true;
}

void ConditionActionBlock::execute()
{
    if (!d_->robot_controller) {
        QString error_msg = tr("机器人控制器未初始化");
        qWarning() << "ConditionActionBlock:" << error_msg;
        Q_EMIT failed(error_msg);
        return;
    }

    if (d_->is_executing) {
        QString error_msg = tr("动作正在执行中");
        qWarning() << "ConditionActionBlock:" << error_msg;
        Q_EMIT failed(error_msg);
        return;
    }

    if (!validateParameters()) {
        QString error_msg = tr("参数验证失败");
        qWarning() << "ConditionActionBlock:" << error_msg;
        Q_EMIT failed(error_msg);
        return;
    }

    d_->is_executing = true;
    Q_EMIT runningChanged(true);
    Q_EMIT started();

    try {
        d_->condition_met              = evaluateCondition();
        ActionBlock* action_to_execute = d_->condition_met ? d_->true_action : d_->false_action;
        d_->current_action             = action_to_execute;

        if (!action_to_execute) {
            d_->is_executing = false;
            Q_EMIT runningChanged(false);
            Q_EMIT completed(true);
            return;
        }

        // 断开之前的连接
        if (d_->true_action) {
            disconnect(d_->true_action, &ActionBlock::completed, this,
                       &ConditionActionBlock::onActionCompleted);
            disconnect(d_->true_action, &ActionBlock::failed, this,
                       &ConditionActionBlock::onActionError);
        }
        if (d_->false_action) {
            disconnect(d_->false_action, &ActionBlock::completed, this,
                       &ConditionActionBlock::onActionCompleted);
            disconnect(d_->false_action, &ActionBlock::failed, this,
                       &ConditionActionBlock::onActionError);
        }

        // 建立新的连接
        connect(action_to_execute, &ActionBlock::completed, this,
                &ConditionActionBlock::onActionCompleted, Qt::UniqueConnection);
        connect(action_to_execute, &ActionBlock::failed, this, &ConditionActionBlock::onActionError,
                Qt::UniqueConnection);

        action_to_execute->reset();
        action_to_execute->execute();

    } catch (const std::exception& e) {
        QString error_msg = tr("执行过程中发生异常: %1").arg(e.what());
        qWarning() << "ConditionActionBlock:" << error_msg;
        d_->is_executing = false;
        Q_EMIT runningChanged(false);
        Q_EMIT failed(error_msg);
    }
}

void ConditionActionBlock::stop()
{
    if (!d_->is_executing) {
        return;
    }

    if (d_->current_action) {
        d_->current_action->stop();
    }

    d_->is_executing   = false;
    d_->current_action = nullptr;
    Q_EMIT runningChanged(false);
    Q_EMIT stopped();
}

void ConditionActionBlock::reset()
{
    if (d_->true_action) {
        d_->true_action->reset();
    }
    if (d_->false_action) {
        d_->false_action->reset();
    }
    d_->is_executing   = false;
    d_->condition_met  = false;
    d_->current_action = nullptr;
}

void ConditionActionBlock::onActionCompleted(bool success)
{
    if (d_->is_executing) {
        d_->is_executing = false;
        Q_EMIT runningChanged(false);
        Q_EMIT completed(success);
    }
}

void ConditionActionBlock::onActionError(const QString& error)
{
    if (d_->is_executing) {
        d_->is_executing = false;
        Q_EMIT runningChanged(false);
        Q_EMIT failed(error);
    }
}

void ConditionActionBlock::onObstacleDetected(bool detected)
{
    if (d_->condition_type == "obstacle_detected" && d_->is_executing) {
        d_->condition_met = detected;
        execute();
    }
}

void ConditionActionBlock::onBatteryLevelChanged(double level)
{
    if (d_->condition_type == "battery_low" && d_->is_executing) {
        d_->condition_met = level < 0.2;  // 20%电量作为低电量阈值
        execute();
    }
}

void ConditionActionBlock::connectSignals()
{
    if (!d_->robot_controller) {
        qWarning() << "ConditionActionBlock: 机器人控制器未初始化";
        return;
    }

    connect(d_->robot_controller.get(), &RobotController::poseUpdated, this,
            &ConditionActionBlock::onPoseUpdated);
    connect(d_->robot_controller.get(), &RobotController::obstacleDetected, this,
            &ConditionActionBlock::onObstacleDetected);
    connect(d_->robot_controller.get(), &RobotController::batteryLevelChanged, this,
            &ConditionActionBlock::onBatteryLevelChanged);
}

void ConditionActionBlock::onPoseUpdated(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
    if (!d_->is_executing) {
        return;
    }

    if (d_->condition_type == "at_position" || d_->condition_type == "at_orientation") {
        bool new_condition_met = evaluateCondition();
        if (new_condition_met != d_->condition_met) {
            d_->condition_met = new_condition_met;
            Q_EMIT conditionChanged(d_->condition_met);
        }
    }
}

void ConditionActionBlock::onObstacleDetected(bool detected)
{
    if (!d_->is_executing) {
        return;
    }

    if (d_->condition_type == "obstacle_detected") {
        if (detected != d_->condition_met) {
            d_->condition_met = detected;
            Q_EMIT conditionChanged(d_->condition_met);
        }
    }
}

void ConditionActionBlock::onBatteryLevelChanged(double level)
{
    if (!d_->is_executing) {
        return;
    }

    if (d_->condition_type == "battery_low") {
        bool new_condition_met = (level < d_->sensor_threshold);
        if (new_condition_met != d_->condition_met) {
            d_->condition_met = new_condition_met;
            Q_EMIT conditionChanged(d_->condition_met);
        }
    }
}

void ConditionActionBlock::onActionStarted()
{
    Q_EMIT statusChanged(tr("执行动作中"));
}

void ConditionActionBlock::onActionCompleted()
{
    if (!d_->is_executing) {
        return;
    }

    d_->is_executing   = false;
    d_->current_action = nullptr;
    Q_EMIT runningChanged(false);
    Q_EMIT completed(true);
}

void ConditionActionBlock::onActionError(const QString& error)
{
    if (!d_->is_executing) {
        return;
    }

    d_->is_executing   = false;
    d_->current_action = nullptr;
    Q_EMIT runningChanged(false);
    Q_EMIT failed(error);
}

void ConditionActionBlock::pause()
{
    if (!d_->is_executing) {
        return;
    }

    if (d_->current_action) {
        d_->current_action->pause();
    }

    Q_EMIT paused();
}

void ConditionActionBlock::resume()
{
    if (!d_->is_executing) {
        return;
    }

    if (d_->current_action) {
        d_->current_action->resume();
    }

    Q_EMIT resumed();
}

QJsonObject ConditionActionBlock::toJson() const
{
    QJsonObject json           = ActionBlock::toJson();
    json["condition_type"]     = d_->condition_type;
    json["target_x"]           = d_->target_x;
    json["target_y"]           = d_->target_y;
    json["target_theta"]       = d_->target_theta;
    json["position_tolerance"] = d_->position_tolerance;
    json["angle_tolerance"]    = d_->angle_tolerance;
    json["sensor_name"]        = d_->sensor_name;
    json["sensor_threshold"]   = d_->sensor_threshold;
    return json;
}

void ConditionActionBlock::fromJson(const QJsonObject& json)
{
    ActionBlock::fromJson(json);
    setConditionType(json["condition_type"].toString());
    setTargetX(json["target_x"].toDouble());
    setTargetY(json["target_y"].toDouble());
    setTargetTheta(json["target_theta"].toDouble());
    setPositionTolerance(json["position_tolerance"].toDouble());
    setAngleTolerance(json["angle_tolerance"].toDouble());
    setSensorName(json["sensor_name"].toString());
    setSensorThreshold(json["sensor_threshold"].toDouble());
}

void ConditionActionBlock::setConditionType(const QString& type)
{
    if (d_->condition_type != type) {
        d_->condition_type = type;
        Q_EMIT conditionTypeChanged(type);
    }
}

void ConditionActionBlock::setTargetX(double x)
{
    if (!qFuzzyCompare(d_->target_x, x)) {
        d_->target_x = x;
        Q_EMIT targetXChanged(x);
    }
}

void ConditionActionBlock::setTargetY(double y)
{
    if (!qFuzzyCompare(d_->target_y, y)) {
        d_->target_y = y;
        Q_EMIT targetYChanged(y);
    }
}

void ConditionActionBlock::setTargetTheta(double theta)
{
    if (!qFuzzyCompare(d_->target_theta, theta)) {
        d_->target_theta = theta;
        Q_EMIT targetThetaChanged(theta);
    }
}

void ConditionActionBlock::setPositionTolerance(double tolerance)
{
    if (tolerance <= 0) {
        qWarning() << "ConditionActionBlock: 位置容差必须大于0";
        return;
    }

    if (!qFuzzyCompare(d_->position_tolerance, tolerance)) {
        d_->position_tolerance = tolerance;
        Q_EMIT positionToleranceChanged(tolerance);
    }
}

void ConditionActionBlock::setAngleTolerance(double tolerance)
{
    if (tolerance <= 0) {
        qWarning() << "ConditionActionBlock: 角度容差必须大于0";
        return;
    }

    if (!qFuzzyCompare(d_->angle_tolerance, tolerance)) {
        d_->angle_tolerance = tolerance;
        Q_EMIT angleToleranceChanged(tolerance);
    }
}

void ConditionActionBlock::setSensorName(const QString& name)
{
    if (d_->sensor_name != name) {
        d_->sensor_name = name;
        Q_EMIT sensorNameChanged(name);
    }
}

void ConditionActionBlock::setSensorThreshold(double threshold)
{
    if (!qFuzzyCompare(d_->sensor_threshold, threshold)) {
        d_->sensor_threshold = threshold;
        Q_EMIT sensorThresholdChanged(threshold);
    }
}

bool ConditionActionBlock::robotAtPosition(double x, double y, double tolerance) const
{
    if (!d_->robot_controller) {
        qWarning() << "ConditionActionBlock: 机器人控制器未初始化";
        return false;
    }

    geometry_msgs::PoseWithCovarianceStamped current_pose;
    try {
        current_pose = d_->robot_controller->getCurrentPose();
    } catch (const std::exception& e) {
        qWarning() << "ConditionActionBlock: 获取机器人位姿失败:" << e.what();
        return false;
    }

    double dx       = current_pose.pose.pose.position.x - x;
    double dy       = current_pose.pose.pose.position.y - y;
    double distance = std::sqrt(dx * dx + dy * dy);

    return distance <= tolerance;
}

bool ConditionActionBlock::robotAtOrientation(double angle, double tolerance) const
{
    if (!d_->robot_controller) {
        qWarning() << "ConditionActionBlock: 机器人控制器未初始化";
        return false;
    }

    geometry_msgs::PoseWithCovarianceStamped current_pose;
    try {
        current_pose = d_->robot_controller->getCurrentPose();
    } catch (const std::exception& e) {
        qWarning() << "ConditionActionBlock: 获取机器人位姿失败:" << e.what();
        return false;
    }

    tf2::Quaternion q(current_pose.pose.pose.orientation.x, current_pose.pose.pose.orientation.y,
                      current_pose.pose.pose.orientation.z, current_pose.pose.pose.orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    double angle_diff = std::abs(yaw - angle);
    while (angle_diff > M_PI) {
        angle_diff -= 2 * M_PI;
    }
    angle_diff = std::abs(angle_diff);

    return angle_diff <= tolerance;
}

bool ConditionActionBlock::sensorValue(const QString& name, double threshold) const
{
    if (!d_->robot_controller) {
        qWarning() << "ConditionActionBlock: 机器人控制器未初始化";
        return false;
    }

    try {
        double value = d_->robot_controller->getSensorValue(name);
        return value >= threshold;
    } catch (const std::exception& e) {
        qWarning() << "ConditionActionBlock: 获取传感器值失败:" << e.what();
        return false;
    }
}

bool ConditionActionBlock::isObstacleDetected() const
{
    if (!d_->robot_controller) {
        qWarning() << "ConditionActionBlock: 机器人控制器未初始化";
        return false;
    }

    try {
        return d_->robot_controller->isObstacleDetected();
    } catch (const std::exception& e) {
        qWarning() << "ConditionActionBlock: 检测障碍物失败:" << e.what();
        return false;
    }
}

bool ConditionActionBlock::isBatteryLow() const
{
    if (!d_->robot_controller) {
        qWarning() << "ConditionActionBlock: 机器人控制器未初始化";
        return false;
    }

    try {
        double battery_level = d_->robot_controller->getBatteryPercentage();
        return battery_level < 0.2;  // 20%电量作为低电量阈值
    } catch (const std::exception& e) {
        qWarning() << "ConditionActionBlock: 获取电池电量失败:" << e.what();
        return false;
    }
}

bool ConditionActionBlock::evaluateCondition()
{
    if (!d_->robot_controller) {
        return false;
    }

    if (d_->condition_type == "at_position") {
        geometry_msgs::PoseWithCovarianceStamped current_pose =
            d_->robot_controller->getCurrentPose();
        double dx       = current_pose.pose.pose.position.x - d_->target_x;
        double dy       = current_pose.pose.pose.position.y - d_->target_y;
        double distance = std::sqrt(dx * dx + dy * dy);
        return distance <= d_->position_tolerance;
    } else if (d_->condition_type == "at_orientation") {
        geometry_msgs::PoseWithCovarianceStamped current_pose =
            d_->robot_controller->getCurrentPose();
        tf2::Quaternion q(
            current_pose.pose.pose.orientation.x, current_pose.pose.pose.orientation.y,
            current_pose.pose.pose.orientation.z, current_pose.pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double         roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        double angle_diff = std::abs(yaw - d_->target_theta);
        while (angle_diff > M_PI) {
            angle_diff -= 2 * M_PI;
        }
        return std::abs(angle_diff) <= d_->angle_tolerance;
    } else if (d_->condition_type == "obstacle_detected") {
        return d_->robot_controller->isObstacleDetected();
    } else if (d_->condition_type == "battery_low") {
        return d_->robot_controller->getBatteryLevel() < d_->sensor_threshold;
    }
    return false;
}

#include "condition_action_block.moc"