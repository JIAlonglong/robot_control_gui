/**
 * @file joystick_widget.cpp
 * @brief 虚拟摇杆控件实现
 */

#include "ui/joystick_widget.h"
#include <QPainter>
#include <QMouseEvent>
#include <QTimer>
#include <cmath>
#include <QDebug>

JoystickWidget::JoystickWidget(QWidget* parent)
    : QWidget(parent)
    , is_pressed_(false)
    , radius_(50)  // 默认摇杆活动半径为50像素
{
    // 设置固定大小
    setFixedSize(150, 150);
    // 初始化摇杆位置为中心
    center_pos_ = QPoint(width() / 2, height() / 2);
    stick_pos_ = center_pos_;

    // 初始化定时器
    update_timer_ = new QTimer(this);
    update_timer_->setInterval(50);  // 20Hz的更新频率
    connect(update_timer_, &QTimer::timeout, this, [this]() {
        // 只有在按下状态才发送位置信息
        if (is_pressed_) {
            emitPosition();
        }
    });

    // 启动定时器
    update_timer_->start();

    // 启用鼠标追踪
    setMouseTracking(true);

    // 设置is_pressed属性，用于外部查询状态
    setProperty("is_pressed", false);
}

void JoystickWidget::paintEvent(QPaintEvent* /*event*/)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // 绘制外圈（基座）
    QRadialGradient baseGradient(center_pos_, radius_);
    baseGradient.setColorAt(0, QColor(245, 246, 250));
    baseGradient.setColorAt(1, QColor(235, 236, 240));
    
    painter.setPen(Qt::NoPen);
    painter.setBrush(baseGradient);
    painter.drawEllipse(center_pos_, radius_, radius_);
    
    // 绘制内圈（参考线）
    painter.setPen(QPen(QColor(200, 200, 200), 1, Qt::DashLine));
    painter.setBrush(Qt::NoBrush);
    painter.drawEllipse(center_pos_, static_cast<int>(radius_ * 0.7), 
                       static_cast<int>(radius_ * 0.7));
    
    // 绘制十字准线
    painter.setPen(QPen(QColor(200, 200, 200), 1));
    painter.drawLine(center_pos_.x() - radius_, center_pos_.y(), 
                    center_pos_.x() + radius_, center_pos_.y());
    painter.drawLine(center_pos_.x(), center_pos_.y() - radius_,
                    center_pos_.x(), center_pos_.y() + radius_);

    // 绘制摇杆
    QRadialGradient stickGradient(stick_pos_, 20);
    if (is_pressed_) {
        stickGradient.setColorAt(0, QColor(52, 152, 219));
        stickGradient.setColorAt(1, QColor(41, 128, 185));
    } else {
        stickGradient.setColorAt(0, QColor(41, 128, 185));
        stickGradient.setColorAt(1, QColor(32, 102, 148));
    }
    
    painter.setPen(Qt::NoPen);
    painter.setBrush(stickGradient);
    painter.drawEllipse(stick_pos_, 20, 20);
    
    // 绘制摇杆高光效果
    QRadialGradient highlightGradient(
        QPointF(stick_pos_.x() - 5, stick_pos_.y() - 5), 10);
    highlightGradient.setColorAt(0, QColor(255, 255, 255, 100));
    highlightGradient.setColorAt(1, QColor(255, 255, 255, 0));
    
    painter.setBrush(highlightGradient);
    painter.drawEllipse(stick_pos_, 15, 15);
}

void JoystickWidget::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        QPoint pos = event->pos();
        // 计算点击位置到中心的距离
        QPoint delta = pos - center_pos_;
        double distance = std::sqrt(delta.x() * delta.x() + delta.y() * delta.y());
        
        // 如果点击在活动范围内，就开始拖动
        if (distance <= radius_) {
            is_pressed_ = true;
            setProperty("is_pressed", true);  // 更新属性
            // 直接更新摇杆位置到点击位置，但要限制在活动范围内
            if (distance > radius_) {
                double scale = radius_ / distance;
                delta.setX(static_cast<int>(delta.x() * scale));
                delta.setY(static_cast<int>(delta.y() * scale));
            }
            stick_pos_ = center_pos_ + delta;
            update();
            
            // 立即发送一次位置信息
            emitPosition();
            
            // 捕获鼠标
            grabMouse();
        }
    }
}

void JoystickWidget::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        // 释放鼠标捕获
        releaseMouse();
        
        if (is_pressed_) {
            is_pressed_ = false;
            setProperty("is_pressed", false);  // 更新属性
            stick_pos_ = center_pos_;  // 回到中心位置
            update();
            
            // 发送零位置信息
            emitPosition();
        }
    }
}

void JoystickWidget::mouseMoveEvent(QMouseEvent* event)
{
    if (is_pressed_) {
        QPoint pos = event->pos();
        // 计算相对于中心的偏移
        QPoint delta = pos - center_pos_;
        double distance = std::sqrt(delta.x() * delta.x() + delta.y() * delta.y());
        
        // 限制在范围内
        if (distance > radius_) {
            double scale = radius_ / distance;
            delta.setX(static_cast<int>(delta.x() * scale));
            delta.setY(static_cast<int>(delta.y() * scale));
        }
        
        // 更新摇杆位置
        stick_pos_ = center_pos_ + delta;
        update();
        
        // 立即发送一次位置信息
        emitPosition();
    }
}

void JoystickWidget::leaveEvent(QEvent* event)
{
    // 如果鼠标离开窗口时摇杆还在按下状态，保持发送当前位置
    QWidget::leaveEvent(event);
}

void JoystickWidget::enterEvent(QEvent* event)
{
    // 如果鼠标重新进入窗口，保持当前状态
    QWidget::enterEvent(event);
}

void JoystickWidget::emitPosition()
{
    // 计算相对位置
    QPointF delta = stick_pos_ - center_pos_;
    double x = qBound(-1.0, delta.x() / radius_, 1.0);
    double y = qBound(-1.0, delta.y() / radius_, 1.0);
    
    // 如果不是按下状态，确保发送零位置
    if (!is_pressed_) {
        x = 0.0;
        y = 0.0;
    }
    
    // 发送位置信息
    emit positionChanged(x, y);
}

void JoystickWidget::setPosition(double x, double y)
{
    // 将输入的x,y限制在[-1,1]范围内
    x = qBound(-1.0, x, 1.0);
    y = qBound(-1.0, y, 1.0);
    
    // 计算摇杆位置
    QPoint delta(static_cast<int>(x * radius_), static_cast<int>(y * radius_));
    stick_pos_ = center_pos_ + delta;
    
    // 更新界面并发送信号
    update();
    emitPosition();
} 