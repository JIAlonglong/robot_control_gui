#ifndef RVIZ_VIEW_H
#define RVIZ_VIEW_H

#include <QWidget>
#include <QTimer>
#include <QLabel>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/view_manager.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <memory>

class RVizView : public QWidget
{
    Q_OBJECT

public:
    explicit RVizView(QWidget* parent = nullptr);
    ~RVizView();

    void setDisplayEnabled(const QString& display_name, bool enabled);
    void setFixedFrame(const QString& frame);
    void setCurrentTool(const QString& tool_name);
    void setDefaultView();

    // 更新数据的方法
    void updateMap(const std::shared_ptr<nav_msgs::OccupancyGrid>& map);
    void updateRobotPose(const std::shared_ptr<nav_msgs::Odometry>& odom);
    void updateLaserScan(const std::shared_ptr<sensor_msgs::LaserScan>& scan);
    void updatePath(const std::vector<geometry_msgs::PoseStamped>& poses);

signals:
    void goalSelected(const geometry_msgs::PoseStamped& goal);

private slots:
    void checkRobotModelStatus();

private:
    void setupDisplays();
    void setupTools();

    rviz::VisualizationManager* manager_;
    rviz::RenderPanel* render_panel_;
    rviz::Display* grid_display_;
    rviz::Display* map_display_;
    rviz::Display* robot_model_display_;
    rviz::Display* laser_scan_display_;
    rviz::Display* path_display_;
    rviz::Display* pose_display_;

    // 状态检查相关
    QTimer* status_check_timer_;
    QLabel* status_label_;

    // TF相关
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif // RVIZ_VIEW_H 