/**
 * @file main.cpp
 * @brief 主程序入口
 */

#include <QApplication>
#include <ros/ros.h>
#include "ui/main_window.h"

int main(int argc, char *argv[])
{
    // 初始化ROS节点
    ros::init(argc, argv, "robot_control_gui");

    // 初始化Qt应用
    QApplication app(argc, argv);

    // 创建主窗口
    MainWindow window;
    window.show();

    // 创建定时器用于处理ROS消息
    QTimer ros_timer;
    QObject::connect(&ros_timer, &QTimer::timeout, []() {
        ros::spinOnce();
    });
    ros_timer.start(100);  // 10Hz

    return app.exec();
} 