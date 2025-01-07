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

    odom_ = odom;
    
    // 更新机器人朝向角度
    double qz = odom->pose.pose.orientation.z;
    double qw = odom->pose.pose.orientation.w;
    robot_theta_ = 2.0 * std::atan2(qz, qw);
    
    update();  // 请求重绘
}

void MapView::updateLaserScan(const std::shared_ptr<sensor_msgs::LaserScan>& scan)
{
    if (!scan) return;

    scan_ = scan;
    update();  // 请求重绘
}

void MapView::paintEvent(QPaintEvent* /*event*/)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    // 填充背景
    painter.fillRect(rect(), QColor(240, 240, 240));
    
    // 保存当前变换
    painter.save();
    
    // 移动到视图中心
    painter.translate(view_center_);
    
    // 绘制网格
    if (display_options_.show_grid) {
        drawGrid(painter);
    }
    
    // 绘制地图
    if (display_options_.show_map && map_) {
        if (map_image_dirty_) {
            updateMapImage();
        }
        painter.drawImage(mapToScreen(-map_->info.width * map_->info.resolution / 2.0,
                                    -map_->info.height * map_->info.resolution / 2.0),
                         map_image_);
    }
    
    // 绘制路径
    drawPath(painter);
    
    // 绘制激光扫描数据
    if (display_options_.show_laser && scan_) {
        drawLaserScan(painter);
    }
    
    // 绘制机器人位置
    if (display_options_.show_robot && odom_) {
        drawRobot(painter);
    }
    
    // 绘制目标点
    drawGoal(painter);
    
    // 恢复变换
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
    // 设置机器人的颜色和大小
    double robot_radius = 0.2 * scale_;  // 机器人半径（米）
    
    // 绘制机器人主体（红色圆形）
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(255, 0, 0, 180));
    painter.drawEllipse(QPointF(0, 0), robot_radius, robot_radius);
    
    // 绘制朝向指示线（白色）
    painter.setPen(QPen(Qt::white, 2));
    painter.drawLine(QPointF(0, 0),
                    QPointF(robot_radius * std::cos(robot_theta_),
                           robot_radius * std::sin(robot_theta_)));
}

void MapView::drawLaserScan(QPainter& painter)
{
    if (!scan_ || !odom_) return;

    // 获取机器人位置和朝向
    double robot_x = odom_->pose.pose.position.x;
    double robot_y = odom_->pose.pose.position.y;
    double qz = odom_->pose.pose.orientation.z;
    double qw = odom_->pose.pose.orientation.w;
    double robot_yaw = 2.0 * std::atan2(qz, qw);

    // 设置激光点的样式
    painter.setPen(QPen(QColor(52, 152, 219, 150), 0.02));  // 半透明蓝色，2cm宽度

    // 绘制激光点
    float angle = scan_->angle_min;
    for (const float& range : scan_->ranges) {
        if (std::isfinite(range) && range >= scan_->range_min && range <= scan_->range_max) {
            // 计算激光点在地图坐标系中的位置
            double point_x = robot_x + range * std::cos(robot_yaw + angle);
            double point_y = robot_y + range * std::sin(robot_yaw + angle);
            
            // 绘制激光点
            painter.drawPoint(QPointF(point_x, point_y));
        }
        angle += scan_->angle_increment;
    }
}

void MapView::drawGrid(QPainter& painter)
{
    // 设置网格线的颜色和样式
    painter.setPen(QPen(QColor(200, 200, 200), 1, Qt::DotLine));
    
    // 计算网格大小（1米一格）
    double grid_size = 1.0 * scale_;
    int grid_count = 20;  // 显示的网格数量
    
    // 绘制水平线
    for (int i = -grid_count; i <= grid_count; ++i) {
        painter.drawLine(QPointF(-grid_count * grid_size, i * grid_size),
                        QPointF(grid_count * grid_size, i * grid_size));
    }
    
    // 绘制垂直线
    for (int i = -grid_count; i <= grid_count; ++i) {
        painter.drawLine(QPointF(i * grid_size, -grid_count * grid_size),
                        QPointF(i * grid_size, grid_count * grid_size));
    }
    
    // 绘制坐标轴
    painter.setPen(QPen(Qt::red, 2));
    painter.drawLine(QPointF(-0.5 * grid_size, 0), QPointF(0.5 * grid_size, 0));  // X轴
    painter.drawLine(QPointF(0, -0.5 * grid_size), QPointF(0, 0.5 * grid_size));  // Y轴
}

void MapView::wheelEvent(QWheelEvent* event)
{
    // 处理缩放
    double factor = event->angleDelta().y() > 0 ? 1.1 : 0.9;
    scale_ *= factor;
    
    // 限制缩放范围
    scale_ = qBound(10.0, scale_, 500.0);
    
    update();
}

void MapView::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        if (event->modifiers() & Qt::ControlModifier) {
            // Ctrl + 左键设置目标点
            is_setting_goal_ = true;
            
            // 计算目标点位置
            QPointF map_pos = screenToMap(event->pos().x() - view_center_.x(),
                                        event->pos().y() - view_center_.y());
            
            // 创建目标点消息
            geometry_msgs::PoseStamped goal;
            goal.header.frame_id = "map";
            goal.header.stamp = ros::Time::now();
            goal.pose.position.x = map_pos.x();
            goal.pose.position.y = map_pos.y();
            goal.pose.orientation.w = 1.0;
            
            emit goalSelected(goal);
        } else {
            // 普通左键拖动
            last_mouse_pos_ = event->pos();
            is_panning_ = true;
        }
    }
}

void MapView::mouseMoveEvent(QMouseEvent* event)
{
    if (is_panning_) {
        QPointF delta = event->pos() - last_mouse_pos_;
        view_center_ += delta;
        last_mouse_pos_ = event->pos();
        update();
    }
}

void MapView::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        is_panning_ = false;
    }
}

void MapView::updateGoal(const geometry_msgs::PoseStamped& goal)
{
    goal_ = goal;
    update();
}

void MapView::updatePath(const std::vector<geometry_msgs::PoseStamped>& path)
{
    path_ = path;
    update();
}

void MapView::setDisplayOptions(const DisplayOptions& options)
{
    display_options_ = options;
    update();
}

void MapView::drawPath(QPainter& painter)
{
    if (!display_options_.show_path || path_.empty()) return;

    painter.setPen(QPen(display_options_.path_color, 3));
    
    QPointF last_point;
    bool first = true;
    
    for (const auto& pose : path_) {
        QPointF current_point = mapToScreen(pose.pose.position.x, pose.pose.position.y);
        if (!first) {
            painter.drawLine(last_point, current_point);
        }
        last_point = current_point;
        first = false;
    }
}

void MapView::drawGoal(QPainter& painter)
{
    if (!display_options_.show_goal) return;

    // 绘制目标点
    double arrow_size = 0.3 * scale_;  // 箭头大小
    QPointF goal_pos = mapToScreen(goal_.pose.position.x, goal_.pose.position.y);
    
    // 计算目标朝向
    double goal_yaw = 2.0 * std::atan2(goal_.pose.orientation.z, goal_.pose.orientation.w);
    
    // 绘制圆形底座
    painter.setPen(Qt::NoPen);
    painter.setBrush(display_options_.goal_color);
    painter.drawEllipse(goal_pos, arrow_size/2, arrow_size/2);
    
    // 绘制朝向箭头
    painter.setPen(QPen(display_options_.goal_color, 2));
    QPointF arrow_end(goal_pos.x() + arrow_size * std::cos(goal_yaw),
                     goal_pos.y() + arrow_size * std::sin(goal_yaw));
    painter.drawLine(goal_pos, arrow_end);
    
    // 绘制箭头头部
    double arrow_head_size = arrow_size * 0.3;
    double arrow_angle = std::atan2(arrow_end.y() - goal_pos.y(),
                                  arrow_end.x() - goal_pos.x());
    QPointF arrow_head1(arrow_end.x() - arrow_head_size * std::cos(arrow_angle + M_PI/6),
                       arrow_end.y() - arrow_head_size * std::sin(arrow_angle + M_PI/6));
    QPointF arrow_head2(arrow_end.x() - arrow_head_size * std::cos(arrow_angle - M_PI/6),
                       arrow_end.y() - arrow_head_size * std::sin(arrow_angle - M_PI/6));
    painter.drawLine(arrow_end, arrow_head1);
    painter.drawLine(arrow_end, arrow_head2);
} 