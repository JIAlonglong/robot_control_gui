/**
 * @file joystick_widget.cpp
 * @brief 虚拟摇杆控件实现
 */

#include "ui/joystick_widget.h"
#include <QPainter>
#include <QMouseEvent>
#include <cmath>
#include <QDebug>
#include <QTimer>

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

    // 创建并配置更新定时器
    update_timer_ = new QTimer(this);
    update_timer_->setInterval(50);  // 20Hz
    connect(update_timer_, &QTimer::timeout, this, &JoystickWidget::onUpdateTimer);
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
        is_pressed_ = true;
        updateStickPosition(event->pos());
        emitPosition();  // 立即发送一次位置
        update_timer_->start();  // 启动定时器
    }
}

void JoystickWidget::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        is_pressed_ = false;
        update_timer_->stop();  // 停止定时器
        stick_pos_ = center_pos_;  // 回到中心位置
        update();
        emitPosition();  // 发送中心位置
    }
}

void JoystickWidget::mouseMoveEvent(QMouseEvent* event)
{
    if (is_pressed_) {
        updateStickPosition(event->pos());
        emitPosition();  // 直接发送位置信号
    }
}

void JoystickWidget::updateStickPosition(const QPoint& pos)
{
    // 计算相对于中心的偏移
    QPoint delta = pos - center_pos_;
    
    // 计算到中心的距离
    double distance = std::sqrt(delta.x() * delta.x() + delta.y() * delta.y());
    
    // 如果超出范围，进行限制
    if (distance > radius_) {
        // 使用浮点数计算以保持精度
        double scale = radius_ / distance;
        delta.setX(static_cast<int>(delta.x() * scale));
        delta.setY(static_cast<int>(delta.y() * scale));
    }
    
    // 更新摇杆位置
    stick_pos_ = center_pos_ + delta;
    update();  // 重绘界面
}

void JoystickWidget::onUpdateTimer()
{
    if (is_pressed_) {
        emitPosition();  // 定时发送位置信号作为备份
    }
}

void JoystickWidget::emitPosition()
{
    // 使用浮点数计算以保持精度
    QPointF delta = stick_pos_ - center_pos_;
    double x = qBound(-1.0, delta.x() / radius_, 1.0);
    double y = qBound(-1.0, delta.y() / radius_, 1.0);
    emit positionChanged(x, y);
} 