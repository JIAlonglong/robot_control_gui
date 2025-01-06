#ifndef MAP_VIEW_H
#define MAP_VIEW_H

#include <QWidget>
#include <QPainter>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <memory>

class MapView : public QWidget {
    Q_OBJECT

public:
    explicit MapView(QWidget* parent = nullptr);

    // 更新显示数据
    void updateMap(const std::shared_ptr<nav_msgs::OccupancyGrid>& map);
    void updateRobotPose(const std::shared_ptr<nav_msgs::Odometry>& odom);
    void updateLaserScan(const std::shared_ptr<sensor_msgs::LaserScan>& scan);

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    // 坐标转换
    QPointF mapToScreen(double x, double y) const;
    QPointF screenToMap(double x, double y) const;

    // 绘制函数
    void updateMapImage();
    void drawRobot(QPainter& painter);
    void drawLaserScan(QPainter& painter);

    // 数据
    std::shared_ptr<nav_msgs::OccupancyGrid> map_;
    std::shared_ptr<nav_msgs::Odometry> robot_pose_;
    std::shared_ptr<sensor_msgs::LaserScan> laser_scan_;

    // 显示参数
    double scale_;              // 缩放比例
    QPointF view_center_;       // 视图中心
    QImage map_image_;         // 地图图像缓存
    bool map_image_dirty_;     // 地图图像是否需要更新
};

#endif // MAP_VIEW_H 