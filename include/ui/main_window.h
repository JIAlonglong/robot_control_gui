#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QDockWidget>
#include <QStatusBar>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QGroupBox>
#include <QRadioButton>
#include <QPushButton>
#include <memory>
#include <map>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include "ros/robot_controller.h"
#include "ui/joystick_widget.h"
#include "ui/rviz_view.h"
#include "ui/robot_status_panel.h"
#include "ui/speed_dashboard.h"
#include "ui/navigation_panel.h"

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

protected:
    void keyPressEvent(QKeyEvent* event) override;
    void keyReleaseEvent(QKeyEvent* event) override;

private slots:
    void updateRobotState();
    void onGoalSelected(const geometry_msgs::PoseStamped& goal);
    void onDisplayOptionsChanged();
    void updateKeyboardControl();
    void handleMapUpdate(const std::shared_ptr<nav_msgs::OccupancyGrid>& map);
    void handleOdomUpdate(const std::shared_ptr<nav_msgs::Odometry>& odom);
    void handleScanUpdate(const std::shared_ptr<sensor_msgs::LaserScan>& scan);
    void onJoystickMoved(double x, double y);

private:
    void setupUi();
    void setupConnections();
    void setupRosConnections();
    void createDisplayOptionsPanel();

    // ROS 相关
    ros::NodeHandle nh_;
    ros::Publisher goal_pub_;
    ros::Subscriber path_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber odom_sub_;

    // 回调函数
    void pathCallback(const nav_msgs::Path::ConstPtr& path);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);

    // UI 组件
    QWidget* central_widget_;
    RVizView* rviz_view_;
    RobotStatusPanel* status_panel_;
    SpeedDashboard* speed_dashboard_;
    NavigationPanel* navigation_panel_;
    JoystickWidget* linear_joystick_;
    JoystickWidget* angular_joystick_;
    QDockWidget* display_options_dock_;

    // 定时器
    QTimer* update_timer_;
    QTimer* keyboard_timer_;

    // 控制器
    std::shared_ptr<RobotController> robot_controller_;

    // 键盘控制相关
    double current_linear_vel_ = 0.0;
    double current_angular_vel_ = 0.0;
    const double max_linear_vel_ = 0.5;  // 最大线速度 (m/s)
    const double max_angular_vel_ = 1.0;  // 最大角速度 (rad/s)

    // 按键状态存储
    std::map<int, bool> key_states_;

    void updateRobotVelocity();
};

#endif // MAIN_WINDOW_H 