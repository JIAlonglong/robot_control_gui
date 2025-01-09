#ifndef TEST_ROBOT_STATUS_PANEL_H
#define TEST_ROBOT_STATUS_PANEL_H

#include <QWidget>
#include <QTest>
#include "ui/robot_status_panel.h"

class TestRobotStatusPanel : public QWidget
{
    Q_OBJECT

public:
    explicit TestRobotStatusPanel(QWidget* parent = nullptr);

private:
    RobotStatusPanel* panel_;
};

#endif // TEST_ROBOT_STATUS_PANEL_H 