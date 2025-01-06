/**
 * @file test_window.h
 * @brief 测试窗口类的声明
 */

#ifndef TEST_WINDOW_H
#define TEST_WINDOW_H

#include <QMainWindow>
#include <QLabel>
#include "ui/joystick_widget.h"

class TestWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit TestWindow(QWidget* parent = nullptr);
    virtual ~TestWindow() = default;

private slots:
    void onPositionChanged(double x, double y);

private:
    JoystickWidget* joystick_;
    QLabel* position_label_;
};

#endif // TEST_WINDOW_H 