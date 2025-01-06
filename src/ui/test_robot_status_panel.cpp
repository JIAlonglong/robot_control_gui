/**
 * @file test_robot_status_panel.cpp
 * @brief 测试机器人状态面板类的实现
 */

#include "ui/test_robot_status_panel.h"
#include <QVBoxLayout>
#include <QTimer>
#include <QDebug>

TestRobotStatusPanel::TestRobotStatusPanel(QWidget* parent)
    : QMainWindow(parent)
{
    // 创建中心部件
    QWidget* central = new QWidget(this);
    central->setStyleSheet("background-color: white;");  // 设置白色背景
    setCentralWidget(central);

    // 创建布局
    QVBoxLayout* layout = new QVBoxLayout(central);
    layout->setContentsMargins(10, 10, 10, 10);  // 设置边距

    // 创建机器人状态面板
    status_panel_ = new RobotStatusPanel(this);
    
    // 添加到布局
    layout->addWidget(status_panel_);
    layout->addStretch();  // 添加弹性空间

    // 设置窗口属性
    setWindowTitle("机器人状态测试");
    resize(400, 300);

    // 初始化状态
    status_panel_->updateBatteryLevel(100);
    status_panel_->updateVelocity(0.0, 0.0);
    status_panel_->updateWorkMode("待机");

    // 创建定时器用于模拟数据更新
    QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &TestRobotStatusPanel::simulateData);
    timer->start(1000); // 每秒更新一次

    qDebug() << "TestRobotStatusPanel initialized";  // 添加调试输出
}

void TestRobotStatusPanel::simulateData()
{
    static int battery = 100;
    static double linear_velocity = 0.0;
    static double angular_velocity = 0.0;
    static bool mode_toggle = false;

    // Update battery level (cycle from 100% to 0%)
    battery = (battery > 0) ? battery - 1 : 100;
    status_panel_->updateBatteryLevel(battery);

    // Update velocity (simulate acceleration and deceleration)
    linear_velocity = (linear_velocity < 1.0) ? linear_velocity + 0.1 : 0.0;
    angular_velocity = (angular_velocity < 0.5) ? angular_velocity + 0.05 : 0.0;
    status_panel_->updateVelocity(linear_velocity, angular_velocity);

    // Update work mode
    mode_toggle = !mode_toggle;
    status_panel_->updateWorkMode(mode_toggle ? "Auto" : "Manual");

    qDebug() << "Updated - Battery:" << battery 
             << "Linear:" << linear_velocity 
             << "Angular:" << angular_velocity;
} 