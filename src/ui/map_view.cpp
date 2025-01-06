/**
 * @file map_view.cpp
 * @brief 地图显示组件的实现
 */

#include "ui/map_view.h"
#include <QPaintEvent>
#include <QResizeEvent>
#include <cmath>

MapView::MapView(QWidget* parent)
    : QWidget(parent)
    , scale_(50.0)  // 1米 = 50像素
    , map_image_dirty_(true)
{
    // 设置背景色
    setBackgroundRole(QPalette::Base);
    setAutoFillBackground(true);

    // 设置初始视图中心为窗口中心
    view_center_ = QPointF(width() / 2.0, height() / 2.0);

    // 启用鼠标追踪
    setMouseTracking(true);
}

void MapView::updateMap(const std::shared_ptr<nav_msgs::OccupancyGrid>& map)
{
    if (!map) return;

    // 检查是否真的需要更新地图
    bool need_update = false;
    if (!map_) {
        need_update = true;
    } else {
        // 只有当地图数据真的改变时才更新
        if (map_->info.width != map->info.width ||
            map_->info.height != map->info.height ||
            map_->info.resolution != map->info.resolution ||
            map_->info.origin.position.x != map->info.origin.position.x ||
            map_->info.origin.position.y != map->info.origin.position.y ||
            map_->data != map->data) {
            need_update = true;
        }
    }

    map_ = map;
    if (need_update) {
        map_image_dirty_ = true;
        update();  // 请求重绘
    }
}

void MapView::updateRobotPose(const std::shared_ptr<nav_msgs::Odometry>& odom)
{
    if (!odom) return;

    robot_pose_ = odom;
    update();  // 请求重绘
}

void MapView::updateLaserScan(const std::shared_ptr<sensor_msgs::LaserScan>& scan)
{
    if (!scan) return;

    laser_scan_ = scan;
    update();  // 请求重绘
}

void MapView::paintEvent(QPaintEvent* /*event*/)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // 填充背景
    painter.fillRect(rect(), QColor(245, 245, 247));

    if (!map_) {
        // 如果没有地图数据，显示提示信息
        painter.setPen(Qt::gray);
        painter.drawText(rect(), Qt::AlignCenter, "等待地图数据...");
        return;
    }

    // 计算缩放因子，使地图适应视图大小
    double map_width = map_->info.width * map_->info.resolution;
    double map_height = map_->info.height * map_->info.resolution;
    double scale_x = (width() - 40) / map_width;
    double scale_y = (height() - 40) / map_height;
    scale_ = std::min(scale_x, scale_y);

    // 保存当前变换
    painter.save();

    // 移动到视图中心
    painter.translate(width() / 2, height() / 2);

    // 应用缩放
    painter.scale(scale_, -scale_);  // 注意Y轴需要翻转

    // 移动到地图原点
    painter.translate(-map_width/2, -map_height/2);

    // 绘制地图
    if (map_image_dirty_) {
        updateMapImage();
    }

    // 使用插值来实现平滑绘制
    painter.setRenderHint(QPainter::SmoothPixmapTransform);
    painter.drawImage(QPointF(0, 0), map_image_);

    // 绘制网格（可选，帮助观察）
    painter.setPen(QPen(QColor(200, 200, 200, 50), 0.02));  // 半透明灰色，2cm宽度
    double grid_size = 1.0;  // 1米一个网格
    for (double x = 0; x <= map_width; x += grid_size) {
        painter.drawLine(QPointF(x, 0), QPointF(x, map_height));
    }
    for (double y = 0; y <= map_height; y += grid_size) {
        painter.drawLine(QPointF(0, y), QPointF(map_width, y));
    }

    // 恢复变换以绘制激光扫描数据
    painter.restore();
    painter.save();
    painter.translate(width() / 2, height() / 2);
    painter.scale(scale_, -scale_);

    // 绘制激光扫描数据
    if (laser_scan_ && robot_pose_) {
        drawLaserScan(painter);
    }

    // 恢复变换以绘制机器人
    painter.restore();
    painter.save();
    painter.translate(width() / 2, height() / 2);
    painter.scale(scale_, -scale_);

    // 绘制机器人
    if (robot_pose_) {
        drawRobot(painter);
    }

    painter.restore();
}

void MapView::resizeEvent(QResizeEvent* event)
{
    // 更新视图中心为窗口中心
    view_center_ = QPointF(width() / 2.0, height() / 2.0);
    QWidget::resizeEvent(event);
}

QPointF MapView::mapToScreen(double x, double y) const
{
    // 将地图坐标转换为屏幕坐标
    return QPointF(x * scale_, -y * scale_);  // 注意Y轴需要翻转
}

QPointF MapView::screenToMap(double x, double y) const
{
    // 将屏幕坐标转换为地图坐标
    return QPointF(x / scale_, -y / scale_);  // 注意Y轴需要翻转
}

void MapView::updateMapImage()
{
    if (!map_) return;

    int width = map_->info.width;
    int height = map_->info.height;
    
    // 如果图像尺寸不对，重新创建图像
    if (map_image_.isNull() || map_image_.width() != width || map_image_.height() != height) {
        map_image_ = QImage(width, height, QImage::Format_ARGB32_Premultiplied);
    }
    
    map_image_.fill(Qt::transparent);
    
    // 填充地图数据
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int value = map_->data[y * width + x];
            QColor color;
            if (value == -1) {
                color = QColor(200, 200, 200, 100);  // 未知区域，半透明灰色
            } else if (value == 0) {
                color = QColor(255, 255, 255, 0);    // 空闲区域，透明
            } else {
                color = QColor(40, 40, 40);          // 障碍物，深灰色
            }
            map_image_.setPixelColor(x, height - y - 1, color);  // 注意Y轴翻转
        }
    }
    
    map_image_dirty_ = false;
}

void MapView::drawRobot(QPainter& painter)
{
    if (!robot_pose_) return;

    // 获取机器人位置
    double x = robot_pose_->pose.pose.position.x;
    double y = robot_pose_->pose.pose.position.y;
    
    // 计算机器人朝向
    double qz = robot_pose_->pose.pose.orientation.z;
    double qw = robot_pose_->pose.pose.orientation.w;
    double yaw = 2.0 * std::atan2(qz, qw);

    // 设置机器人外观
    painter.save();
    painter.translate(x, y);
    painter.rotate(yaw * 180.0 / M_PI);

    // 绘制机器人主体（红色圆形）
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(231, 76, 60));  // 红色
    painter.drawEllipse(QPointF(0, 0), 0.2, 0.2);  // 20cm半径

    // 绘制朝向指示器（白色三角形）
    painter.setBrush(Qt::white);
    QPolygonF direction;
    direction << QPointF(0.1, 0) << QPointF(-0.1, 0.1) << QPointF(-0.1, -0.1);
    painter.drawPolygon(direction);

    painter.restore();
}

void MapView::drawLaserScan(QPainter& painter)
{
    if (!laser_scan_ || !robot_pose_) return;

    // 获取机器人位置和朝向
    double robot_x = robot_pose_->pose.pose.position.x;
    double robot_y = robot_pose_->pose.pose.position.y;
    double qz = robot_pose_->pose.pose.orientation.z;
    double qw = robot_pose_->pose.pose.orientation.w;
    double robot_yaw = 2.0 * std::atan2(qz, qw);

    // 设置激光点的样式
    painter.setPen(QPen(QColor(52, 152, 219, 150), 0.02));  // 半透明蓝色，2cm宽度

    // 绘制激光点
    float angle = laser_scan_->angle_min;
    for (const float& range : laser_scan_->ranges) {
        if (std::isfinite(range) && range >= laser_scan_->range_min && range <= laser_scan_->range_max) {
            // 计算激光点在地图坐标系中的位置
            double point_x = robot_x + range * std::cos(robot_yaw + angle);
            double point_y = robot_y + range * std::sin(robot_yaw + angle);
            
            // 绘制激光点
            painter.drawPoint(QPointF(point_x, point_y));
        }
        angle += laser_scan_->angle_increment;
    }
} 