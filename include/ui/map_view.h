#ifndef MAP_VIEW_H
#define MAP_VIEW_H

#include <QWidget>
#include <QPainter>
#include <QMouseEvent>
#include <QWheelEvent>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <vector>

// 显示选项结构体
struct DisplayOptions {
    bool show_grid = true;
    bool show_map = true;
    bool show_robot = true;
    bool show_laser = true;
    bool show_path = true;
    bool show_goal = true;
    
    // 颜色设置
    QColor grid_color = QColor(200, 200, 200);
    QColor robot_color = QColor(255, 0, 0, 180);
    QColor laser_color = QColor(52, 152, 219, 150);
    QColor path_color = QColor(0, 255, 0, 200);
    QColor goal_color = QColor(255, 165, 0, 200);
};

class MapView : public QWidget {
    Q_OBJECT

public:
    explicit MapView(QWidget* parent = nullptr);

    // 更新显示数据
    void updateMap(const std::shared_ptr<nav_msgs::OccupancyGrid>& map);
    void updateRobotPose(const std::shared_ptr<nav_msgs::Odometry>& odom);
    void updateLaserScan(const std::shared_ptr<sensor_msgs::LaserScan>& scan);
    void updateGoal(const geometry_msgs::PoseStamped& goal);
    void updatePath(const std::vector<geometry_msgs::PoseStamped>& path);

    // 显示选项设置
    void setDisplayOptions(const DisplayOptions& options);
    const DisplayOptions& displayOptions() const { return display_options_; }

signals:
    void goalSelected(const geometry_msgs::PoseStamped& goal);
    void pointSelected(const geometry_msgs::Point& point);

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    // 坐标转换
    QPointF mapToScreen(double x, double y) const;
    QPointF screenToMap(double x, double y) const;

    // 绘制函数
    void updateMapImage();
    void drawRobot(QPainter& painter);
    void drawLaserScan(QPainter& painter);
    void drawGrid(QPainter& painter);
    void drawPath(QPainter& painter);
    void drawGoal(QPainter& painter);

    // 数据
    std::shared_ptr<nav_msgs::OccupancyGrid> map_;
    std::shared_ptr<nav_msgs::Odometry> odom_;
    std::shared_ptr<sensor_msgs::LaserScan> scan_;
    geometry_msgs::PoseStamped goal_;
    std::vector<geometry_msgs::PoseStamped> path_;

    // 显示参数
    double scale_;              // 缩放比例
    QPointF view_center_;       // 视图中心
    QImage map_image_;         // 地图图像缓存
    bool map_image_dirty_;     // 地图图像是否需要更新
    DisplayOptions display_options_;  // 显示选项
    
    // 鼠标交互
    bool is_panning_ = false;  // 是否正在平移
    bool is_setting_goal_ = false;  // 是否正在设置目标点
    QPoint last_mouse_pos_;    // 上一次鼠标位置
    
    // 机器人姿态
    double robot_theta_ = 0.0;  // 机器人朝向角度
};

#endif // MAP_VIEW_H 