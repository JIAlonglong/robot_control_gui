/**
 * @file main.cpp
 * @brief 主程序入口
 */

#include "ui/main_window.h"
#include <QApplication>
#include <QTimer>
#include <ros/ros.h>

int main(int argc, char *argv[]) {
    // 初始化ROS
    ros::init(argc, argv, "robot_control_gui");
    
    // 初始化Qt
    QApplication app(argc, argv);
    app.setStyle("Fusion");

    // 创建主窗口
    MainWindow window;
    window.show();

    // 创建ROS定时器
    QTimer ros_timer;
    ros_timer.connect(&ros_timer, &QTimer::timeout, []() {
        ros::spinOnce();
    });
    ros_timer.start(100);  // 10Hz

    return app.exec();
} 