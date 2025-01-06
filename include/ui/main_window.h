/**
 * @file main_window.h
 * @brief 主窗口类的声明
 */

#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QLabel>
#include <QKeyEvent>
#include <QSet>
#include "ui/joystick_widget.h"
#include "ui/robot_status_panel.h"
#include "ui/speed_dashboard.h"
#include "ui/map_view.h"
#include "ros/robot_controller.h"

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);

protected:
    void keyPressEvent(QKeyEvent* event) override;
    void keyReleaseEvent(QKeyEvent* event) override;

private slots:
    void onJoystickMoved(double x, double y);      // 处理移动摇杆
    void onRotationJoystickMoved(double x, double y); // 处理旋转摇杆
    void updateRobotStatus();                      // 更新机器人状态
    void updateKeyboardControl();                  // 更新键盘控制

private:
    // UI组件
    JoystickWidget* joystick_;          // 移动控制摇杆
    JoystickWidget* rotation_joystick_; // 旋转控制摇杆
    RobotStatusPanel* status_panel_;
    SpeedDashboard* speed_dashboard_;
    MapView* map_view_;                 // 地图显示组件

    // 状态更新定时器
    QTimer* status_timer_;
    QTimer* keyboard_timer_;            // 键盘控制更新定时器

    // 当前速度
    double current_linear_speed_;
    double current_angular_speed_;

    // 机器人状态
    int battery_level_;
    bool is_auto_mode_;

    // 添加ROS控制器
    std::unique_ptr<RobotController> robot_controller_;

    // 键盘状态
    QSet<int> pressed_keys_;            // 当前按下的键
};

#endif // MAIN_WINDOW_H 