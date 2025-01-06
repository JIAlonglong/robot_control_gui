/**
 * @file test_joystick.cpp
 * @brief 虚拟摇杆测试程序
 */

#include "ui/test_window.h"
#include <QApplication>
#include <QVBoxLayout>

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    TestWindow window;
    window.show();
    return app.exec();
} 