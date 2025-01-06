/**
 * @file test_robot_status_panel.h
 * @brief 测试机器人状态面板类的声明
 */

#ifndef TEST_ROBOT_STATUS_PANEL_H
#define TEST_ROBOT_STATUS_PANEL_H

#include <QMainWindow>
#include "ui/robot_status_panel.h"

class TestRobotStatusPanel : public QMainWindow {
    Q_OBJECT
public:
    explicit TestRobotStatusPanel(QWidget* parent = nullptr);

private slots:
    void simulateData();

private:
    RobotStatusPanel* status_panel_;
};

#endif // TEST_ROBOT_STATUS_PANEL_H 