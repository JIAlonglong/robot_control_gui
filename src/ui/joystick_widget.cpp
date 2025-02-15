/*
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
 */

/**
 * @file joystick_widget.cpp
 * @brief 虚拟摇杆控件实现
 */

#include "ui/joystick_widget.h"
#include <QPainter>
#include <QMouseEvent>
#include <cmath>

JoystickWidget::JoystickWidget(QWidget* parent)
    : QWidget(parent)
    , is_pressed_(false)
{
    // 设置固定大小
    setMinimumSize(200, 200);
    
    // 计算中心点和摇杆大小
    center_ = rect().center();
    stick_pos_ = center_;  // 初始化摇杆位置为中心
}

JoystickWidget::~JoystickWidget() = default;

void JoystickWidget::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);
    
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    // 更新中心点（以防大小改变）
    center_ = rect().center();
    
    // 绘制底座圆
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(240, 240, 240));  // 浅灰色背景
    painter.drawEllipse(center_, base_radius_, base_radius_);
    
    // 绘制十字线
    painter.setPen(QPen(QColor(200, 200, 200), 1));  // 浅灰色线
    painter.drawLine(center_.x() - base_radius_, center_.y(), 
                    center_.x() + base_radius_, center_.y());
    painter.drawLine(center_.x(), center_.y() - base_radius_,
                    center_.x(), center_.y() + base_radius_);
    
    // 绘制摇杆
    painter.setPen(Qt::NoPen);
    // 绘制摇杆阴影
    painter.setBrush(QColor(0, 0, 0, 30));
    painter.drawEllipse(stick_pos_ + QPoint(2, 2), stick_radius_, stick_radius_);
    // 绘制摇杆本体
    QLinearGradient gradient(stick_pos_ - QPoint(stick_radius_, stick_radius_),
                            stick_pos_ + QPoint(stick_radius_, stick_radius_));
    gradient.setColorAt(0, QColor(51, 154, 255));  // 浅蓝色
    gradient.setColorAt(1, QColor(0, 119, 255));   // 深蓝色
    painter.setBrush(gradient);
    painter.drawEllipse(stick_pos_, stick_radius_, stick_radius_);
    
    // 绘制摇杆高光
    painter.setBrush(QColor(255, 255, 255, 80));
    painter.drawEllipse(stick_pos_ - QPoint(stick_radius_/3, stick_radius_/3),
                       stick_radius_/3, stick_radius_/3);
}

void JoystickWidget::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        QPoint pos = event->pos();
        if (QLineF(pos, center_).length() <= base_radius_) {
            is_pressed_ = true;
            stick_pos_ = pos;
            update();
            updateJoystickPosition();
        }
    }
}

void JoystickWidget::mouseMoveEvent(QMouseEvent* event)
{
    if (is_pressed_) {
        QPoint pos = event->pos();
        QLineF line(center_, pos);
        if (line.length() > base_radius_) {
            // 限制在底座圆内
            line.setLength(base_radius_);
            stick_pos_ = line.p2().toPoint();
        } else {
            stick_pos_ = pos;
        }
        update();
        updateJoystickPosition();
    }
}

void JoystickWidget::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton && is_pressed_) {
        is_pressed_ = false;
        stick_pos_ = center_;  // 回到中心位置
        update();
        updateJoystickPosition();
    }
}

void JoystickWidget::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    
    // 更新中心点和半径
    center_ = rect().center();
    
    // 确保摇杆在中心
    if (!is_pressed_) {
        stick_pos_ = center_;
    }
}

void JoystickWidget::updateJoystickPosition()
{
    QPointF normalized = normalizePosition(stick_pos_);
    double dx = normalized.x();
    double dy = normalized.y();
    
    emit moved(dx, dy);  // 使用统一的 moved 信号
    update();
}

void JoystickWidget::reset()
{
    is_pressed_ = false;
    stick_pos_ = center_;
    emit moved(0.0, 0.0);  // 使用统一的 moved 信号
    update();
}

QPointF JoystickWidget::normalizePosition(const QPoint& pos) const
{
    // 计算相对于中心的偏移
    QPointF offset = pos - center_;
    
    // 归一化到 [-1, 1] 范围
    double dx = qBound(-1.0, offset.x() / base_radius_, 1.0);
    double dy = qBound(-1.0, offset.y() / base_radius_, 1.0);
    
    return QPointF(dx, dy);
} 