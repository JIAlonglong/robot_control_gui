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
 * @file navigation_action_block.cpp
 * @brief 导航动作块的实现
 * @author JIAlonglong
 */

#include "navigation_action_block.h"
#include <QDebug>
#include <QIcon>
#include <cmath>
#include "robot_controller.h"

NavigationActionBlock::NavigationActionBlock(std::shared_ptr<RobotController> robot_controller,
                                             QObject*                         parent)
    : ActionBlock(parent), robot_controller_(robot_controller)
{
    // 连接导航相关信号
    connect(robot_controller_.get(), &RobotController::navigationCompleted, this,
            &NavigationActionBlock::onNavigationCompleted);
    connect(robot_controller_.get(), &RobotController::navigationStateChanged, this,
            &NavigationActionBlock::stateChanged);
    connect(robot_controller_.get(), &RobotController::navigationProgressChanged, this,
            &NavigationActionBlock::onNavigationFeedback);

    // 初始化导航参数
    setParameter("x", 0.0);
    setParameter("y", 0.0);
    setParameter("theta", 0.0);
    setParameter("tolerance", 0.1);
}

NavigationActionBlock::~NavigationActionBlock() = default;

QIcon NavigationActionBlock::icon() const
{
    return QIcon(":/icons/navigation.png");
}

QVariantMap NavigationActionBlock::properties() const
{
    QVariantMap props = parameters();
    props["状态"] = isRunning() ? tr("执行中") : tr("就绪");
    props["目标位置"] = QString("(%.2f, %.2f)").arg(x()).arg(y());
    props["目标朝向"] = QString("%.2f°").arg(theta() * 180.0 / M_PI);
    props["容差"] = QString("%.2f m").arg(tolerance());
    return props;
}

void NavigationActionBlock::setProperty(const QString& name, const QVariant& value)
{
    setParameter(name, value);
    emit propertyChanged(name, value);
    
    // 更新导航参数
    if (name == "x" || name == "y" || name == "theta" || name == "tolerance") {
        emit parameterChanged(name, value);
    }
}

QJsonObject NavigationActionBlock::toJson() const
{
    QJsonObject json = ActionBlock::toJson();
    json["type"] = type();
    json["x"] = x();
    json["y"] = y();
    json["theta"] = theta();
    json["tolerance"] = tolerance();
    return json;
}

void NavigationActionBlock::fromJson(const QJsonObject& json)
{
    ActionBlock::fromJson(json);
    setX(json["x"].toDouble());
    setY(json["y"].toDouble());
    setTheta(json["theta"].toDouble());
    setTolerance(json["tolerance"].toDouble());
}

void NavigationActionBlock::execute()
{
    if (!robot_controller_) {
        emit error(tr("未连接到机器人"));
        return;
    }

    if (!isValid()) {
        emit error(tr("导航参数无效"));
        return;
    }

    // 创建导航目标
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = x();
    goal.pose.position.y = y();
    goal.pose.orientation.z = sin(theta() / 2.0);
    goal.pose.orientation.w = cos(theta() / 2.0);

    onExecutionStarted();
    robot_controller_->setNavigationGoal(goal);
}

void NavigationActionBlock::stop()
{
    if (!robot_controller_) {
        return;
    }

    robot_controller_->stopNavigation();
    onExecutionStopped();
}

void NavigationActionBlock::pause()
{
    if (!robot_controller_) {
        return;
    }

    robot_controller_->pauseNavigation();
    onExecutionPaused();
}

void NavigationActionBlock::resume()
{
    if (!robot_controller_) {
        return;
    }

    robot_controller_->resumeNavigation();
    onExecutionResumed();
}

void NavigationActionBlock::reset()
{
    ActionBlock::reset();
    setX(0.0);
    setY(0.0);
    setTheta(0.0);
    setTolerance(0.1);
}

void NavigationActionBlock::onNavigationCompleted(bool success)
{
    if (success) {
        onExecutionFinished(true);
        emit statusChanged(tr("导航完成"));
    } else {
        onExecutionError(tr("导航失败"));
        emit statusChanged(tr("导航失败"));
    }
}

void NavigationActionBlock::onNavigationFeedback(double progress)
{
    emit progressChanged(progress);
    
    QString status;
    if (progress < 30.0) {
        status = tr("正在规划路径...");
    } else if (progress < 60.0) {
        status = tr("正在导航中...");
    } else if (progress < 90.0) {
        status = tr("接近目标点...");
    } else {
        status = tr("微调位姿...");
    }
    
    emit statusChanged(status);
}