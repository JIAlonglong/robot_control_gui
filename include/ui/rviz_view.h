#ifndef ROBOT_CONTROL_GUI_RVIZ_VIEW_H
#define ROBOT_CONTROL_GUI_RVIZ_VIEW_H

#include <QWidget>
#include <QLabel>
#include <QMutex>
#include <memory>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool.h>
#include <rviz/tool_manager.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

class RVizView : public QWidget {
    Q_OBJECT
public:
    explicit RVizView(QWidget* parent = nullptr);
    ~RVizView() override;

    void setDisplayEnabled(const QString& display_name, bool enabled);
    void setRobotModel(const QString& model);
    void setFixedFrame(const QString& frame);
    void setTargetFrame(const QString& frame);
    void updateMap(const nav_msgs::OccupancyGridConstPtr& map);
    void updateRobotPose(const nav_msgs::OdometryConstPtr& odom);
    void updateLaserScan(const sensor_msgs::LaserScanConstPtr& scan);
    void updatePath(const std::vector<geometry_msgs::PoseStamped>& poses);

public slots:
    void onGoalToolActivated();
    void onInitialPoseToolActivated();
    void onGoalPositionSelected(const Ogre::Vector3& position, const Ogre::Quaternion& orientation);
    void onInitialPoseSelected(const Ogre::Vector3& position, const Ogre::Quaternion& orientation);

signals:
    void goalSelected(const geometry_msgs::PoseStamped& goal);
    void initialPoseSelected(const geometry_msgs::PoseWithCovarianceStamped& pose);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    void cleanup();
    void setupDisplays();

    QLabel* status_label_{nullptr};
    rviz::RenderPanel* render_panel_{nullptr};
    rviz::VisualizationManager* manager_{nullptr};
    rviz::Display* grid_display_{nullptr};
    rviz::Display* robot_model_display_{nullptr};
    rviz::Display* map_display_{nullptr};
    rviz::Display* laser_scan_display_{nullptr};
    rviz::Display* global_path_display_{nullptr};  // 全局路径显示
    rviz::Display* local_path_display_{nullptr};   // 局部路径显示
    rviz::Display* goal_display_{nullptr};
    rviz::Tool* goal_tool_{nullptr};
    rviz::Tool* initial_pose_tool_{nullptr};
    QMutex mutex_;
};

#endif // ROBOT_CONTROL_GUI_RVIZ_VIEW_H 