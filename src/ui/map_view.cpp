/**
 * @file map_view.cpp
 * @brief 地图显示组件的实现
 */

#include "ui/map_view.h"
#include <QMouseEvent>
#include <QWheelEvent>
#include <QPainter>
#include <cmath>

MapView::MapView(QWidget* parent)
    : QWidget(parent)
    , robot_x_(0.0)
    , robot_y_(0.0)
    , robot_theta_(0.0)
    , scale_(50.0)  // 1米 = 50像素
    , is_dragging_(false)
    , resolution_(0.05)  // 默认分辨率5cm
    , origin_x_(0.0)
    , origin_y_(0.0)
{
    // 设置背景色和其他颜色
    background_color_ = Qt::white;
    wall_color_ = Qt::black;
    unknown_color_ = Qt::gray;
    robot_color_ = Qt::red;
    laser_color_ = Qt::blue;

    // 设置窗口属性
    setMinimumSize(400, 400);
    setFocusPolicy(Qt::StrongFocus);
    setAttribute(Qt::WA_OpaquePaintEvent);
    
    // 初始化视图中心为窗口中心
    view_center_ = QPointF(width() / 2.0, height() / 2.0);
}

void MapView::updateMap(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    map_data_ = map;
    resolution_ = map->info.resolution;
    origin_x_ = map->info.origin.position.x;
    origin_y_ = map->info.origin.position.y;

    // 创建QImage
    map_image_ = QImage(map->info.width, map->info.height, QImage::Format_RGB32);
    
    // 将地图数据转换为图像
    for (unsigned int y = 0; y < map->info.height; ++y) {
        for (unsigned int x = 0; x < map->info.width; ++x) {
            int value = map->data[y * map->info.width + x];
            QColor color;
            if (value == -1) {
                color = unknown_color_;
            } else if (value >= 50) {
                color = wall_color_;
            } else {
                color = background_color_;
            }
            map_image_.setPixel(x, map->info.height - y - 1, color.rgb());
        }
    }
    
    update();  // 触发重绘
}

void MapView::updateRobotPose(const nav_msgs::Odometry::ConstPtr& odom)
{
    robot_x_ = odom->pose.pose.position.x;
    robot_y_ = odom->pose.pose.position.y;
    
    // 从四元数转换为欧拉角
    tf::Quaternion q(
        odom->pose.pose.orientation.x,
        odom->pose.pose.orientation.y,
        odom->pose.pose.orientation.z,
        odom->pose.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, robot_theta_);
    
    update();  // 触发重绘
}

void MapView::updateLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    laser_data_ = scan;
    update();  // 触发重绘
}

void MapView::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    // 填充背景
    painter.fillRect(rect(), background_color_);
    
    // 应用视图变换
    painter.translate(view_center_);
    painter.scale(scale_, -scale_);  // Y轴向上为正
    
    // 绘制地图
    drawMap(painter);
    
    // 绘制激光数据
    drawLaserScan(painter);
    
    // 绘制机器人
    drawRobot(painter);
}

void MapView::drawMap(QPainter& painter)
{
    if (map_image_.isNull()) return;
    
    // 计算地图在世界坐标系中的位置和大小
    double map_width = map_image_.width() * resolution_;
    double map_height = map_image_.height() * resolution_;
    
    // 绘制地图图像
    painter.save();
    painter.translate(origin_x_, origin_y_);
    painter.scale(map_width / map_image_.width(), map_height / map_image_.height());
    painter.drawImage(0, 0, map_image_);
    painter.restore();
}

void MapView::drawRobot(QPainter& painter)
{
    const double robot_radius = 0.2;  // 机器人半径（米）
    const double arrow_length = robot_radius * 1.5;  // 方向指示箭头长度
    
    painter.save();
    
    // 移动到机器人位置
    painter.translate(robot_x_, robot_y_);
    painter.rotate(robot_theta_ * 180.0 / M_PI);  // 旋转到机器人朝向
    
    // 绘制机器人主体（圆形）
    painter.setPen(Qt::NoPen);
    painter.setBrush(robot_color_);
    painter.drawEllipse(QPointF(0, 0), robot_radius, robot_radius);
    
    // 绘制方向指示箭头
    painter.setPen(QPen(robot_color_, 0.05));
    painter.drawLine(QPointF(0, 0), QPointF(arrow_length, 0));
    painter.drawLine(QPointF(arrow_length, 0), QPointF(arrow_length * 0.7, arrow_length * 0.3));
    painter.drawLine(QPointF(arrow_length, 0), QPointF(arrow_length * 0.7, -arrow_length * 0.3));
    
    painter.restore();
}

void MapView::drawLaserScan(QPainter& painter)
{
    if (!laser_data_) return;
    
    painter.save();
    painter.setPen(QPen(laser_color_, 0.02));
    
    // 移动到机器人位置
    painter.translate(robot_x_, robot_y_);
    painter.rotate(robot_theta_ * 180.0 / M_PI);
    
    // 绘制每个激光点
    double angle = laser_data_->angle_min;
    for (const float range : laser_data_->ranges) {
        if (std::isfinite(range) && range >= laser_data_->range_min && range <= laser_data_->range_max) {
            double x = range * cos(angle);
            double y = range * sin(angle);
            painter.drawPoint(QPointF(x, y));
        }
        angle += laser_data_->angle_increment;
    }
    
    painter.restore();
}

QPointF MapView::mapToScreen(double x, double y) const
{
    return QPointF(x * scale_ + view_center_.x(), -y * scale_ + view_center_.y());
}

QPointF MapView::screenToMap(double x, double y) const
{
    return QPointF((x - view_center_.x()) / scale_, -(y - view_center_.y()) / scale_);
}

void MapView::wheelEvent(QWheelEvent* event)
{
    // 计算缩放因子
    double factor = event->delta() > 0 ? 1.1 : 0.9;
    
    // 获取鼠标位置对应的地图坐标
    QPointF mouse_map_pos = screenToMap(event->pos().x(), event->pos().y());
    
    // 更新缩放比例
    scale_ *= factor;
    
    // 调整视图中心，使鼠标指向的地图点保持不变
    QPointF new_screen_pos = mapToScreen(mouse_map_pos.x(), mouse_map_pos.y());
    view_center_ += event->pos() - new_screen_pos;
    
    update();
}

void MapView::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        is_dragging_ = true;
        drag_start_ = event->pos();
    }
}

void MapView::mouseMoveEvent(QMouseEvent* event)
{
    if (is_dragging_) {
        QPointF delta = event->pos() - drag_start_;
        view_center_ += delta;
        drag_start_ = event->pos();
        update();
    }
}

void MapView::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        is_dragging_ = false;
    }
}

void MapView::resizeEvent(QResizeEvent* event)
{
    // 更新视图中心为窗口中心
    view_center_ = QPointF(width() / 2.0, height() / 2.0);
    QWidget::resizeEvent(event);
} 