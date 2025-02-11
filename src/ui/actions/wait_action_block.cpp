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
 * @file wait_action_block.cpp
 * @brief 动作块基类,定义动作的基本接口
 * @author JIAlonglong
 */

#include "wait_action_block.h"
#include <QDebug>

struct WaitActionBlock::Private {
    double duration{1.0};
    QTimer timer;
};

WaitActionBlock::WaitActionBlock(QObject* parent)
    : ActionBlock(parent), d_(std::make_unique<Private>())
{
    d_->timer.setSingleShot(true);
    connect(&d_->timer, &QTimer::timeout, this, &WaitActionBlock::onTimeout);
}

WaitActionBlock::~WaitActionBlock() = default;

QIcon WaitActionBlock::icon() const
{
    return QIcon::fromTheme("chronometer");
}

double WaitActionBlock::duration() const
{
    return d_->duration;
}

void WaitActionBlock::setDuration(double duration)
{
    if (qFuzzyCompare(d_->duration, duration)) {
        return;
    }
    d_->duration = duration;
    Q_EMIT durationChanged(duration);
}

QVariantMap WaitActionBlock::properties() const
{
    QVariantMap props;
    props["duration"] = d_->duration;
    return props;
}

void WaitActionBlock::setProperty(const QString& name, const QVariant& value)
{
    if (name == "duration") {
        setDuration(value.toDouble());
    }
    Q_EMIT propertyChanged(name, value);
}

void WaitActionBlock::execute()
{
    if (d_->duration <= 0.0) {
        Q_EMIT error(tr("等待时间必须大于0"));
        Q_EMIT completed(false);
        return;
    }

    if (isRunning()) {
        Q_EMIT error(tr("等待动作正在执行中"));
        Q_EMIT completed(false);
        return;
    }

    onExecutionStarted();
    d_->timer.start(static_cast<int>(d_->duration * 1000));
}

void WaitActionBlock::stop()
{
    if (isRunning()) {
        d_->timer.stop();
        onExecutionStopped();
        Q_EMIT stopped();
        Q_EMIT completed(false);
    }
}

void WaitActionBlock::reset()
{
    stop();
}

void WaitActionBlock::onTimeout()
{
    if (isRunning()) {
        onExecutionFinished(true);
    }
}