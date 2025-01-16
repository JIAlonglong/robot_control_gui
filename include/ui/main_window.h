#ifndef ROBOT_CONTROL_GUI_MAIN_WINDOW_H
#define ROBOT_CONTROL_GUI_MAIN_WINDOW_H

#include <QMainWindow>
#include <memory>
#include <QTimer>
#include <QToolBar>
#include <QAction>
#include <QLabel>
#include <QCheckBox>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QDockWidget>
#include <QStackedWidget>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "ros/robot_controller.h"
#include "ui/navigation_panel.h"
#include "ui/rviz_view.h"
#include "ui/speed_dashboard.h"
#include "ui/control_panel.h"
#include "ui/settings_panel.h"

class MainWindowPrivate
{
public:
    ros::NodeHandle nh_;
    QWidget* central_widget_{nullptr};
    QToolBar* tool_bar_{nullptr};
    QDockWidget* display_options_dock_{nullptr};
    QStackedWidget* stacked_widget_{nullptr};
    std::shared_ptr<SpeedDashboard> speed_dashboard_;

    std::shared_ptr<RobotController> robot_controller_;
    std::shared_ptr<NavigationPanel> navigation_panel_;
    std::shared_ptr<ControlPanel> control_panel_;
    std::shared_ptr<RVizView> rviz_view_;
    std::shared_ptr<SettingsPanel> settings_panel_;

    ros::Publisher goal_pub_;
    ros::Subscriber path_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber battery_sub_;
    ros::Subscriber diagnostics_sub_;
    ros::Subscriber camera_sub_;

    QMap<int, bool> key_states_;
    double current_linear_vel_{0.0};
    double current_angular_vel_{0.0};
    double max_linear_vel_{0.5};
    double max_angular_vel_{1.0};
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

protected:
    void keyPressEvent(QKeyEvent* event) override;
    void keyReleaseEvent(QKeyEvent* event) override;

private slots:
    void onGoalSelected(const geometry_msgs::PoseStamped& goal);
    void onInitialPoseSelected(const geometry_msgs::PoseWithCovarianceStamped& pose);
    void onDisplayOptionsChanged();
    void onRobotStatusChanged(const QString& status);
    void onLinearJoystickMoved(double x, double y);
    void onAngularJoystickMoved(double x, double y);

private:
    void setupUi();
    void createToolBar();
    void createDisplayOptionsPanel();
    void createFloatingControlPanel();
    void connectSignals();
    void setupSubscribers();
    void setupRosConnections();
    void updateKeyboardControl();
    void updateRobotState();
    void updateRobotVelocity();

    void handleMapUpdate(const nav_msgs::OccupancyGridConstPtr& msg);
    void handleOdomUpdate(const nav_msgs::OdometryConstPtr& msg);
    void handleScanUpdate(const sensor_msgs::LaserScanConstPtr& msg);
    void pathCallback(const nav_msgs::PathConstPtr& msg);
    void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
    void scanCallback(const sensor_msgs::LaserScanConstPtr& msg);
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

    std::unique_ptr<MainWindowPrivate> d_;
};

#endif // ROBOT_CONTROL_GUI_MAIN_WINDOW_H 