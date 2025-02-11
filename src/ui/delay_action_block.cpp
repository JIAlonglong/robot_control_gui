#include "ui/delay_action_block.h"
#include <QDebug>

DelayActionBlock::DelayActionBlock(QObject* parent)
    : ActionBlock(parent)
{
    connect(&timer_, &QTimer::timeout, this, &DelayActionBlock::onTimeout);
}

DelayActionBlock::~DelayActionBlock() = default;

QIcon DelayActionBlock::icon() const
{
    return QIcon(":/icons/delay.png");
}

QVariantMap DelayActionBlock::properties() const
{
    QVariantMap props;
    props["duration"] = duration_;
    return props;
}

void DelayActionBlock::setProperty(const QString& name, const QVariant& value)
{
    if (name == "duration") {
        setDuration(value.toDouble());
    }
    emit propertyChanged(name, value);
}

void DelayActionBlock::setDuration(double duration)
{
    if (qFuzzyCompare(duration_, duration))
        return;
    duration_ = duration;
    emit durationChanged(duration);
}

void DelayActionBlock::execute()
{
    if (duration_ <= 0) {
        emit error(tr("延时时长必须大于0"));
        emit completed(false);
        return;
    }

    if (is_running_) {
        emit error(tr("延时动作正在执行中"));
        emit completed(false);
        return;
    }

    is_running_ = true;
    emit runningChanged(true);
    emit started();

    timer_.setInterval(duration_ * 1000);  // 转换为毫秒
    timer_.setSingleShot(true);
    timer_.start();
}

void DelayActionBlock::stop()
{
    if (is_running_) {
        timer_.stop();
        is_running_ = false;
        emit runningChanged(false);
        emit stopped();
        emit completed(false);
    }
}

void DelayActionBlock::reset()
{
    stop();
}

void DelayActionBlock::onTimeout()
{
    if (is_running_) {
        is_running_ = false;
        emit runningChanged(false);
        emit stopped();
        emit completed(true);
    }
}