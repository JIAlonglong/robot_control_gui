/**
 * @file map_view.h
 * @brief 地图显示组件的声明
 */

#ifndef MAP_VIEW_H
#define MAP_VIEW_H

#include <QWidget>
#include <QImage>
#include <QPainter>
#include <QTimer>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>

class MapView : public QWidget {
    Q_OBJECT
public:
    explicit MapView(QWidget* parent = nullptr);

    // 更新地图数据
    void updateMap(const nav_msgs::OccupancyGrid::ConstPtr& map);
    // 更新机器人位置
    void updateRobotPose(const nav_msgs::Odometry::ConstPtr& odom);
    // 更新激光雷达数据
    void updateLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan);

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    // 绘图函数
    void drawMap(QPainter& painter);
    void drawRobot(QPainter& painter);
    void drawLaserScan(QPainter& painter);
    
    // 坐标转换函数
    QPointF mapToScreen(double x, double y) const;
    QPointF screenToMap(double x, double y) const;

    // 地图数据
    QImage map_image_;
    nav_msgs::OccupancyGrid::ConstPtr map_data_;
    
    // 机器人位置
    double robot_x_;
    double robot_y_;
    double robot_theta_;
    
    // 激光雷达数据
    sensor_msgs::LaserScan::ConstPtr laser_data_;
    
    // 视图变换
    double scale_;          // 缩放比例
    QPointF view_center_;   // 视图中心
    QPointF drag_start_;    // 拖动起点
    bool is_dragging_;      // 是否正在拖动
    
    // 地图参数
    double resolution_;     // 地图分辨率
    double origin_x_;       // 地图原点X
    double origin_y_;       // 地图原点Y
    
    // 颜色设置
    QColor background_color_;
    QColor wall_color_;
    QColor unknown_color_;
    QColor robot_color_;
    QColor laser_color_;
};

#endif // MAP_VIEW_H 