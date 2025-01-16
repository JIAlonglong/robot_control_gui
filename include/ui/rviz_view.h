#ifndef ROBOT_CONTROL_GUI_RVIZ_VIEW_H
#define ROBOT_CONTROL_GUI_RVIZ_VIEW_H

#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>
#include <QMutex>

#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/tool.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

class RVizView : public QWidget
{
    Q_OBJECT

public:
    explicit RVizView(QWidget* parent = nullptr);
    ~RVizView() override;

    void setupDisplays();
    void setRobotModel(const QString& model);
    void setFixedFrame(const QString& frame);
    void setTargetFrame(const QString& frame);
    void setDisplayEnabled(const QString& display_name, bool enabled);

    void updateMap(const nav_msgs::OccupancyGridConstPtr& map);
    void updateRobotPose(const nav_msgs::OdometryConstPtr& odom);
    void updateLaserScan(const sensor_msgs::LaserScanConstPtr& scan);
    void updatePath(const std::vector<geometry_msgs::PoseStamped>& poses);

    void onGoalToolActivated();
    void onInitialPoseToolActivated();

signals:
    void goalSelected(const geometry_msgs::PoseStamped& goal);
    void initialPoseSelected(const geometry_msgs::PoseWithCovarianceStamped& pose);

protected:
    void paintEvent(QPaintEvent* event) override;

private slots:
    void onGoalPositionSelected(const Ogre::Vector3& position, const Ogre::Quaternion& orientation);
    void onInitialPoseSelected(const Ogre::Vector3& position, const Ogre::Quaternion& orientation);

private:
    void cleanup();

    QLabel* status_label_{nullptr};
    rviz::RenderPanel* render_panel_{nullptr};
    rviz::VisualizationManager* manager_{nullptr};
    rviz::Tool* goal_tool_{nullptr};
    rviz::Tool* initial_pose_tool_{nullptr};
    rviz::Display* grid_display_{nullptr};
    rviz::Display* robot_model_display_{nullptr};
    rviz::Display* map_display_{nullptr};
    rviz::Display* laser_scan_display_{nullptr};
    rviz::Display* path_display_{nullptr};
    rviz::Display* goal_display_{nullptr};

    QMutex mutex_;
};

#endif // ROBOT_CONTROL_GUI_RVIZ_VIEW_H 