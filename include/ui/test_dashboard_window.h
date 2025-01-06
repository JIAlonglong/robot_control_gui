/**
 * @file test_dashboard_window.h
 * @brief 测试仪表盘窗口类的声明
 */

#ifndef TEST_DASHBOARD_WINDOW_H
#define TEST_DASHBOARD_WINDOW_H

#include <QMainWindow>
#include "ui/speed_dashboard.h"

class TestDashboardWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit TestDashboardWindow(QWidget* parent = nullptr);

private slots:
    void simulateData();  // 模拟数据更新

private:
    SpeedDashboard* dashboard_;
};

#endif // TEST_DASHBOARD_WINDOW_H 