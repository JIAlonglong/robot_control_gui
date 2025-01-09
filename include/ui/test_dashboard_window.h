#ifndef TEST_DASHBOARD_WINDOW_H
#define TEST_DASHBOARD_WINDOW_H

#include <QWidget>
#include <QTest>
#include "ui/dashboard_window.h"

class TestDashboardWindow : public QWidget
{
    Q_OBJECT

public:
    explicit TestDashboardWindow(QWidget* parent = nullptr);

private:
    DashboardWindow* dashboard_;
};

#endif // TEST_DASHBOARD_WINDOW_H 