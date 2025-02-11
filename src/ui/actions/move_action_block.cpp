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
 * @file move_action_block.cpp
 * @brief 动作块基类,定义动作的基本接口
 * @author JIAlonglong
 */

#include "move_action_block.h"
#include <QDebug>
#include <QIcon>

MoveActionBlock::MoveActionBlock(const std::shared_ptr<RobotController>& robot_controller,
                                 QObject*                                parent)
    : ActionBlock(parent), robot_controller_(robot_controller)
{
    connect(&timer_, &QTimer::timeout, this, &MoveActionBlock::onTimeout);
}

MoveActionBlock::~MoveActionBlock() = default;

QIcon MoveActionBlock::icon() const
{
    return QIcon::fromTheme("go-next");
}

QVariantMap MoveActionBlock::properties() const
{
    QVariantMap props;
    props["linear_velocity"]  = linear_velocity_;
    props["angular_velocity"] = angular_velocity_;
    props["duration"]         = duration_;
    return props;
}

void MoveActionBlock::setProperty(const QString& name, const QVariant& value)
{
    if (name == "linear_velocity") {
        setLinearVelocity(value.toDouble());
    } else if (name == "angular_velocity") {
        setAngularVelocity(value.toDouble());
    } else if (name == "duration") {
        setDuration(value.toDouble());
    }
    emit propertyChanged(name, value);
}

void MoveActionBlock::setLinearVelocity(double velocity)
{
    if (qFuzzyCompare(linear_velocity_, velocity)) return;
    linear_velocity_ = velocity;
    emit linearVelocityChanged(velocity);
}

void MoveActionBlock::setAngularVelocity(double velocity)
{
    if (qFuzzyCompare(angular_velocity_, velocity)) return;
    angular_velocity_ = velocity;
    emit angularVelocityChanged(velocity);
}

void MoveActionBlock::setDuration(double duration)
{
    if (qFuzzyCompare(duration_, duration)) return;
    duration_ = duration;
    emit durationChanged(duration);
}

void MoveActionBlock::execute()
{
    if (duration_ <= 0) {
        emit error(tr("移动时间必须大于0"));
        emit completed(false);
        return;
    }

    if (is_running_) {
        emit error(tr("移动动作正在执行中"));
        emit completed(false);
        return;
    }

    is_running_ = true;
    emit runningChanged(true);
    emit started();

    // 设置机器人速度
    robot_controller_->setLinearVelocity(linear_velocity_);
    robot_controller_->setAngularVelocity(angular_velocity_);

    timer_.setInterval(duration_ * 1000);  // 转换为毫秒
    timer_.setSingleShot(true);
    timer_.start();
}

void MoveActionBlock::stop()
{
    if (is_running_) {
        timer_.stop();
        robot_controller_->setLinearVelocity(0);
        robot_controller_->setAngularVelocity(0);
        is_running_ = false;
        emit runningChanged(false);
        emit stopped();
        emit completed(false);
    }
}

void MoveActionBlock::reset()
{
    stop();
}

void MoveActionBlock::onTimeout()
{
    if (is_running_) {
        robot_controller_->setLinearVelocity(0);
        robot_controller_->setAngularVelocity(0);
        is_running_ = false;
        emit runningChanged(false);
        emit stopped();
        emit completed(true);
    }
}