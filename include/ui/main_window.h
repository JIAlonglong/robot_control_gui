#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QLabel>
#include <memory>
#include "ui/map_view.h"
#include "ui/joystick_widget.h"
#include "ui/robot_status_panel.h"
#include "ui/speed_dashboard.h"
#include "ui/navigation_panel.h"
#include "ros/robot_controller.h"

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

protected:
    void keyPressEvent(QKeyEvent* event) override;
    void keyReleaseEvent(QKeyEvent* event) override;

private:
    // UI组件
    MapView* map_view_;
    JoystickWidget* joystick_;
    RobotStatusPanel* status_panel_;
    SpeedDashboard* speed_dashboard_;
    NavigationPanel* navigation_panel_;

    // ROS控制器
    std::shared_ptr<RobotController> robot_controller_;

    // 定时器
    QTimer* keyboard_timer_;

    // 初始化函数
    void setupUi();
    void setupConnections();

private slots:
    // 处理来自ROS的数据更新
    void handleMapUpdate(const std::shared_ptr<nav_msgs::OccupancyGrid>& map);
    void handleOdomUpdate(const std::shared_ptr<nav_msgs::Odometry>& odom);
    void handleScanUpdate(const std::shared_ptr<sensor_msgs::LaserScan>& scan);

    // 处理用户输入
    void onJoystickMoved(double x, double y);
    void updateKeyboardControl();
};

#endif // MAIN_WINDOW_H 