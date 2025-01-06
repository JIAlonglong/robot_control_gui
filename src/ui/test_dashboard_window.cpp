/**
 * @file test_dashboard_window.cpp
 * @brief 测试仪表盘窗口类的实现
 */

#include "ui/test_dashboard_window.h"
#include <QVBoxLayout>
#include <QTimer>
#include <QDebug>
#include <cmath>

TestDashboardWindow::TestDashboardWindow(QWidget* parent)
    : QMainWindow(parent)
{
    // 创建中心部件
    QWidget* central = new QWidget(this);
    central->setStyleSheet("background-color: white;");
    setCentralWidget(central);

    // 创建布局
    QVBoxLayout* layout = new QVBoxLayout(central);
    layout->setContentsMargins(20, 20, 20, 20);

    // 创建仪表盘
    dashboard_ = new SpeedDashboard(this);
    layout->addWidget(dashboard_);

    // 设置窗口属性
    setWindowTitle("Speed Dashboard Test");
    resize(600, 300);

    // 创建定时器用于模拟数据更新
    QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &TestDashboardWindow::simulateData);
    timer->start(50);  // 每50ms更新一次

    qDebug() << "TestDashboardWindow initialized";
}

void TestDashboardWindow::simulateData()
{
    static double time = 0.0;
    
    // 使用正弦函数生成平滑的速度变化
    double linear_speed = 1.0 + sin(time) * 0.8;  // 范围在0.2到1.8之间
    double angular_speed = 0.5 + cos(time) * 0.4;  // 范围在0.1到0.9之间
    
    dashboard_->setLinearSpeed(linear_speed);
    dashboard_->setAngularSpeed(angular_speed);
    
    time += 0.05;  // 增加时间
} 