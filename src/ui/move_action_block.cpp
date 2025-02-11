#include "ui/move_action_block.h"
#include <QDebug>
#include <QIcon>

MoveActionBlock::MoveActionBlock(const std::shared_ptr<RobotController>& robot_controller, QObject* parent)
    : ActionBlock(parent)
    , robot_controller_(robot_controller)
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
    props["linear_velocity"] = linear_velocity_;
    props["angular_velocity"] = angular_velocity_;
    props["duration"] = duration_;
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
    if (qFuzzyCompare(linear_velocity_, velocity))
        return;
    linear_velocity_ = velocity;
    emit linearVelocityChanged(velocity);
}

void MoveActionBlock::setAngularVelocity(double velocity)
{
    if (qFuzzyCompare(angular_velocity_, velocity))
        return;
    angular_velocity_ = velocity;
    emit angularVelocityChanged(velocity);
}

void MoveActionBlock::setDuration(double duration)
{
    if (qFuzzyCompare(duration_, duration))
        return;
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