/**
 * @file map_view_widget.cpp
 * @brief SLAM地图显示组件实现
 */

#include "ui/map_view_widget.h"
#include <QPainter>
#include <QWheelEvent>
#include <cmath>

MapViewWidget::MapViewWidget(QWidget* parent)
    : QWidget(parent)
    , resolution_(0.05)  // 默认分辨率 5cm/pixel
    , is_panning_(false)
{
    // 初始化变换矩阵
    transform_.reset();
    transform_.scale(1.0, 1.0);
    
    // 设置背景色
    setBackgroundRole(QPalette::Base);
    setAutoFillBackground(true);
    
    // 设置焦点策略以接收键盘事件
    setFocusPolicy(Qt::StrongFocus);
}

void MapViewWidget::updateMap(const QImage& map_data, double resolution)
{
    map_ = map_data;
    resolution_ = resolution;
    update();
}

void MapViewWidget::setRobotPose(double x, double y, double theta)
{
    robot_pose_.x = x;
    robot_pose_.y = y;
    robot_pose_.theta = theta;
    update();
}

void MapViewWidget::paintEvent(QPaintEvent* /*event*/)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    // 应用变换
    painter.setTransform(transform_);
    
    // 绘制地图
    if (!map_.isNull()) {
        painter.drawImage(0, 0, map_);
    }
    
    // 绘制机器人
    drawRobot(painter);
}

void MapViewWidget::drawRobot(QPainter& painter)
{
    // 转换机器人位置到像素坐标
    int pixel_x = robot_pose_.x / resolution_;
    int pixel_y = robot_pose_.y / resolution_;
    
    // 保存当前变换
    painter.save();
    
    // 移动到机器人位置并旋转
    painter.translate(pixel_x, pixel_y);
    painter.rotate(robot_pose_.theta * 180.0 / M_PI);
    
    // 绘制机器人图标（一个带方向的三角形）
    static const QPointF robot_shape[3] = {
        QPointF(10, 0),
        QPointF(-5, 8),
        QPointF(-5, -8),
    };
    
    painter.setPen(Qt::red);
    painter.setBrush(Qt::red);
    painter.drawPolygon(robot_shape, 3);
    
    // 恢复变换
    painter.restore();
}

void MapViewWidget::wheelEvent(QWheelEvent* event)
{
    // 计算缩放因子
    double factor = event->angleDelta().y() > 0 ? 1.1 : 0.9;
    
    // 更新变换矩阵
    transform_.scale(factor, factor);
    update();
}

void MapViewWidget::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        is_panning_ = true;
        last_mouse_pos_ = event->pos();
    }
}

void MapViewWidget::mouseMoveEvent(QMouseEvent* event)
{
    if (is_panning_) {
        QPoint delta = event->pos() - last_mouse_pos_;
        transform_.translate(delta.x() / transform_.m11(), 
                           delta.y() / transform_.m22());
        last_mouse_pos_ = event->pos();
        update();
    }
}

void MapViewWidget::mouseReleaseEvent(QMouseEvent* /*event*/)
{
    is_panning_ = false;
} 