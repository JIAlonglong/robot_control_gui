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
 * @file joystick_widget.cpp
 * @brief 虚拟摇杆组件的实现,用于通过鼠标控制机器人移动
 * @author JIAlonglong
 */

#include "joystick_widget.h"
#include <QMouseEvent>
#include <QPainter>
#include <QResizeEvent>
#include <cmath>

struct JoystickWidget::Private {
    QPointF normalized_pos_;
    double  max_speed_{1.0};
    bool    is_pressed_{false};
};

JoystickWidget::JoystickWidget(QWidget* parent) : QWidget(parent), d_(new Private)
{
    setMinimumSize(100, 100);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setFocusPolicy(Qt::StrongFocus);
    reset();
}

JoystickWidget::~JoystickWidget()
{
    delete d_;
}

void JoystickWidget::reset()
{
    d_->is_pressed_     = false;
    d_->normalized_pos_ = QPointF(0.0, 0.0);
    update();
}

double JoystickWidget::getLinearVelocity() const
{
    return -d_->normalized_pos_.y() * d_->max_speed_;
}

double JoystickWidget::getAngularVelocity() const
{
    return -d_->normalized_pos_.x() * d_->max_speed_;
}

void JoystickWidget::paintEvent(QPaintEvent* /*event*/)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // 绘制底座圆
    painter.setPen(Qt::black);
    painter.setBrush(Qt::lightGray);
    painter.drawEllipse(rect().center(), base_radius_, base_radius_);

    // 绘制十字线
    painter.setPen(QPen(Qt::gray, 1, Qt::DashLine));
    painter.drawLine(rect().center().x() - base_radius_, rect().center().y(),
                     rect().center().x() + base_radius_, rect().center().y());
    painter.drawLine(rect().center().x(), rect().center().y() - base_radius_, rect().center().x(),
                     rect().center().y() + base_radius_);

    // 绘制摇杆
    painter.setPen(Qt::black);
    painter.setBrush(d_->is_pressed_ ? Qt::darkGray : Qt::gray);
    painter.drawEllipse(stick_pos_, stick_radius_, stick_radius_);
}

void JoystickWidget::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        QPoint relative_pos = event->pos() - rect().center();
        if (relative_pos.x() * relative_pos.x() + relative_pos.y() * relative_pos.y() <=
            base_radius_ * base_radius_) {
            is_pressed_     = true;
            stick_pos_      = event->pos();
            normalized_pos_ = normalizePosition(stick_pos_);
            emit linearJoystickMoved(normalized_pos_.x(), normalized_pos_.y());
            update();
        }
    }
}

void JoystickWidget::mouseMoveEvent(QMouseEvent* event)
{
    if (is_pressed_) {
        QPoint relative_pos = event->pos() - center_;
        double length =
            std::sqrt(relative_pos.x() * relative_pos.x() + relative_pos.y() * relative_pos.y());

        if (length > base_radius_) {
            // 限制在底座圆内
            double scale = base_radius_ / length;
            relative_pos.setX(static_cast<int>(relative_pos.x() * scale));
            relative_pos.setY(static_cast<int>(relative_pos.y() * scale));
            stick_pos_ = center_ + relative_pos;
        } else {
            stick_pos_ = event->pos();
        }

        normalized_pos_ = normalizePosition(stick_pos_);
        emit linearJoystickMoved(normalized_pos_.x(), normalized_pos_.y());
        update();
    }
}

void JoystickWidget::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton && is_pressed_) {
        reset();
        emit linearJoystickMoved(0.0, 0.0);
    }
}

void JoystickWidget::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    center_ = rect().center();

    // 调整底座和摇杆的大小
    int min_dimension = std::min(width(), height());
    base_radius_      = min_dimension / 3;
    stick_radius_     = base_radius_ / 4;

    if (!is_pressed_) {
        stick_pos_ = center_;
    }
    update();
}

QPointF JoystickWidget::normalizePosition(const QPoint& pos) const
{
    QPoint relative_pos = pos - center_;
    return QPointF(static_cast<double>(relative_pos.x()) / base_radius_,
                   static_cast<double>(relative_pos.y()) / base_radius_);
}