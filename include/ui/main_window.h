/*
 * Copyright (c) 2025 JIAlonglong
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

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
#include <QTabWidget>

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
#include "ui/mapping_panel.h"

class MainWindowPrivate
{
public:
    ros::NodeHandle nh_;
    QWidget* central_widget_{nullptr};
    QToolBar* tool_bar_{nullptr};
    QDockWidget* display_options_dock_{nullptr};
    QStackedWidget* stacked_widget_{nullptr};
    QLabel* status_label_{nullptr};
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

    MappingPanel* mapping_panel_{nullptr};
    QTabWidget* tab_widget_{nullptr};
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
    // UI 设置相关
    void setupUi();
    void createToolBar();
    void createDisplayOptionsPanel();
    void setupConnections();

    // ROS 相关
    void setupSubscribers();
    void setupRosConnections();
    
    // 控制相关
    void updateKeyboardControl();
    void updateRobotState();
    void updateRobotVelocity();

    // ROS 回调函数
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