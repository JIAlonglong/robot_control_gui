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
 * @file movement_action_block.cpp
 * @brief 动作块基类,定义动作的基本接口
 * @author JIAlonglong
 */

#include "movement_action_block.h"
#include <QDebug>
#include <QIcon>
#include <QTimer>
#include "robot_controller.h"

struct MovementActionBlock::Private {
    std::shared_ptr<RobotController> robot_controller;
    bool                             is_running{false};
    bool                             is_paused{false};
    double                           linear_velocity{0.0};
    double                           angular_velocity{0.0};
    double                           duration{1.0};
    double                           elapsed_time{0.0};
    QTimer*                          timer{nullptr};
};

MovementActionBlock::MovementActionBlock(const std::shared_ptr<RobotController>& controller,
                                         QObject*                                parent)
    : ActionBlock(parent), d_(std::make_unique<Private>())
{
    d_->robot_controller = controller;
    d_->timer            = new QTimer(this);
    d_->timer->setInterval(100);  // 100ms更新一次
    connect(d_->timer, &QTimer::timeout, this, &MovementActionBlock::onTimeout);
}

MovementActionBlock::~MovementActionBlock() = default;

QIcon MovementActionBlock::icon() const
{
    return QIcon::fromTheme("system-run");
}

bool MovementActionBlock::isRunning() const
{
    return d_->is_running;
}

double MovementActionBlock::linearVelocity() const
{
    return d_->linear_velocity;
}

double MovementActionBlock::angularVelocity() const
{
    return d_->angular_velocity;
}

double MovementActionBlock::duration() const
{
    return d_->duration;
}

void MovementActionBlock::setLinearVelocity(double velocity)
{
    if (d_->linear_velocity != velocity) {
        d_->linear_velocity = velocity;
        Q_EMIT linearVelocityChanged(velocity);
    }
}

void MovementActionBlock::setAngularVelocity(double velocity)
{
    if (d_->angular_velocity != velocity) {
        d_->angular_velocity = velocity;
        Q_EMIT angularVelocityChanged(velocity);
    }
}

void MovementActionBlock::setDuration(double duration)
{
    if (d_->duration != duration) {
        d_->duration = duration;
        Q_EMIT durationChanged(duration);
    }
}

QVariantMap MovementActionBlock::properties() const
{
    QVariantMap props;
    props["linear_velocity"]  = d_->linear_velocity;
    props["angular_velocity"] = d_->angular_velocity;
    props["duration"]         = d_->duration;
    return props;
}

void MovementActionBlock::setProperty(const QString& name, const QVariant& value)
{
    if (name == "linear_velocity") {
        setLinearVelocity(value.toDouble());
    } else if (name == "angular_velocity") {
        setAngularVelocity(value.toDouble());
    } else if (name == "duration") {
        setDuration(value.toDouble());
    }
    Q_EMIT propertyChanged(name, value);
}

void MovementActionBlock::execute()
{
    if (!d_->robot_controller || d_->is_running) {
        return;
    }

    d_->is_running   = true;
    d_->is_paused    = false;
    d_->elapsed_time = 0.0;

    // 设置机器人速度
    d_->robot_controller->setVelocity(d_->linear_velocity, d_->angular_velocity);

    // 启动定时器
    d_->timer->start();

    Q_EMIT started();
}

void MovementActionBlock::stop()
{
    if (!d_->is_running) {
        return;
    }

    d_->timer->stop();
    d_->is_running = false;
    d_->is_paused  = false;

    // 停止机器人
    if (d_->robot_controller) {
        d_->robot_controller->setVelocity(0.0, 0.0);
    }

    Q_EMIT stopped();
}

void MovementActionBlock::pause()
{
    if (!d_->is_running || d_->is_paused) {
        return;
    }

    d_->is_paused = true;
    d_->timer->stop();

    // 暂停机器人
    if (d_->robot_controller) {
        d_->robot_controller->setVelocity(0.0, 0.0);
    }

    Q_EMIT paused();
}

void MovementActionBlock::resume()
{
    if (!d_->is_running || !d_->is_paused) {
        return;
    }

    d_->is_paused = false;

    // 恢复机器人速度
    if (d_->robot_controller) {
        d_->robot_controller->setVelocity(d_->linear_velocity, d_->angular_velocity);
    }

    d_->timer->start();
    Q_EMIT resumed();
}

void MovementActionBlock::reset()
{
    stop();
    d_->elapsed_time = 0.0;
}

void MovementActionBlock::onTimeout()
{
    if (!d_->is_running || d_->is_paused) {
        return;
    }

    d_->elapsed_time += d_->timer->interval() / 1000.0;

    if (d_->elapsed_time >= d_->duration) {
        stop();
        Q_EMIT completed(true);
    }
}