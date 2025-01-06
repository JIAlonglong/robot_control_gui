/**
 * @file test_dashboard_main.cpp
 * @brief 仪表盘测试程序入口
 */

#include "ui/test_dashboard_window.h"
#include <QApplication>
#include <QDebug>

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    
    // 设置应用程序样式
    app.setStyle("Fusion");
    
    // 创建并显示测试窗口
    TestDashboardWindow window;
    window.show();
    
    qDebug() << "Dashboard test application started";
    
    return app.exec();
} 