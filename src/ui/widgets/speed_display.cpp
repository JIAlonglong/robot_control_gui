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
 * @file speed_display.cpp
 * @brief 速度显示组件的实现,用于显示机器人的线速度和角速度
 * @author JIAlonglong
 */

#include "widgets/speed_display.h"
#include <QFontMetrics>
#include <QPaintEvent>
#include <QPainter>
#include <cmath>

SpeedDisplay::SpeedDisplay(QWidget* parent) : QWidget(parent)
{
    setMinimumSize(200, 100);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

SpeedDisplay::~SpeedDisplay() = default;

void SpeedDisplay::updateSpeed(double linear, double angular)
{
    linear_speed_  = linear;
    angular_speed_ = angular;
    update();
}

QSize SpeedDisplay::sizeHint() const
{
    return QSize(300, 150);
}

QSize SpeedDisplay::minimumSizeHint() const
{
    return QSize(200, 100);
}

void SpeedDisplay::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    drawBackground(painter);
    drawSpeedGauge(painter);
    drawSpeedText(painter);
}

void SpeedDisplay::drawBackground(QPainter& painter)
{
    painter.save();
    painter.setPen(QPen(border_color_, border_width_));
    painter.setBrush(background_color_);
    painter.drawRect(rect().adjusted(border_width_ / 2, border_width_ / 2, -border_width_ / 2,
                                     -border_width_ / 2));
    painter.restore();
}

void SpeedDisplay::drawSpeedGauge(QPainter& painter)
{
    painter.save();

    int gauge_height = height() / 3;
    int gauge_width  = width() - 2 * border_width_ - 20;
    int gauge_y      = height() / 2 - gauge_height / 2;

    // 线速度仪表
    int linear_width = static_cast<int>(gauge_width * std::abs(linear_speed_) / max_linear_speed_);
    QRect linear_rect(border_width_ + 10, gauge_y, linear_width, gauge_height / 2 - 5);
    painter.fillRect(linear_rect, gauge_color_);

    // 角速度仪表
    int angular_width =
        static_cast<int>(gauge_width * std::abs(angular_speed_) / max_angular_speed_);
    QRect angular_rect(border_width_ + 10, gauge_y + gauge_height / 2 + 5, angular_width,
                       gauge_height / 2 - 5);
    painter.fillRect(angular_rect, gauge_color_);

    painter.restore();
}

void SpeedDisplay::drawSpeedText(QPainter& painter)
{
    painter.save();

    QFont font = painter.font();
    font.setPointSize(10);
    painter.setFont(font);

    QString linear_text  = QString("线速度: %1 m/s").arg(linear_speed_, 0, 'f', 2);
    QString angular_text = QString("角速度: %1 rad/s").arg(angular_speed_, 0, 'f', 2);

    int text_y = height() / 6;
    painter.drawText(10, text_y, linear_text);
    painter.drawText(10, height() - text_y, angular_text);

    painter.restore();
}