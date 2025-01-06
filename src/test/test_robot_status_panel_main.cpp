/**
 * @file test_robot_status_panel_main.cpp
 * @brief 机器人状态面板测试程序入口
 */

#include "ui/test_robot_status_panel.h"
#include <QApplication>
#include <QDebug>

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    
    // 设置应用程序样式
    app.setStyle("Fusion");  // 使用Fusion样式
    
    // 设置默认字体
    QFont defaultFont("Arial", 10);
    app.setFont(defaultFont);
    
    // 创建并显示测试窗口
    TestRobotStatusPanel window;
    window.show();
    
    qDebug() << "Application started";
    
    return app.exec();
} 