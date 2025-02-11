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
 * @file delay_action_block.cpp
 * @brief 动作块基类,定义动作的基本接口
 * @author JIAlonglong
 */

#include "delay_action_block.h"
#include <QDebug>

DelayActionBlock::DelayActionBlock(QObject* parent) : ActionBlock(parent)
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
    if (qFuzzyCompare(duration_, duration)) return;
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