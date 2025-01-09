#include "ui/test_robot_status_panel.h"
#include <QVBoxLayout>

TestRobotStatusPanel::TestRobotStatusPanel(QWidget* parent)
    : QWidget(parent)
    , panel_(new RobotStatusPanel(this))
{
    setWindowTitle("机器人状态面板测试");
    setMinimumSize(400, 300);

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(panel_);
    setLayout(layout);
} 