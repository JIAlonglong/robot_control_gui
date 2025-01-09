#include "ui/test_dashboard_window.h"
#include <QVBoxLayout>

TestDashboardWindow::TestDashboardWindow(QWidget* parent)
    : QWidget(parent)
    , dashboard_(new DashboardWindow(this))
{
    setWindowTitle("仪表盘测试");
    setMinimumSize(800, 600);

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(dashboard_);
    setLayout(layout);
} 