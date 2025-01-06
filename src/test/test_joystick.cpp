/**
 * @file test_joystick.cpp
 * @brief 虚拟摇杆测试程序
 */

#include <gtest/gtest.h>
#include <QApplication>
#include <QTest>
#include "ui/joystick_widget.h"

class TestJoystick : public ::testing::Test {
protected:
    void SetUp() override {
        int argc = 0;
        char** argv = nullptr;
        app = new QApplication(argc, argv);
        joystick = new JoystickWidget(nullptr);
    }

    void TearDown() override {
        delete joystick;
        delete app;
    }

    QApplication* app;
    JoystickWidget* joystick;
};

TEST_F(TestJoystick, TestInitialization) {
    ASSERT_TRUE(joystick != nullptr);
    EXPECT_TRUE(joystick->isVisible() == false);
}

TEST_F(TestJoystick, TestPosition) {
    // 测试初始位置
    EXPECT_DOUBLE_EQ(0.0, joystick->getX());
    EXPECT_DOUBLE_EQ(0.0, joystick->getY());

    // 测试设置位置
    joystick->setPosition(0.5, -0.5);
    EXPECT_DOUBLE_EQ(0.5, joystick->getX());
    EXPECT_DOUBLE_EQ(-0.5, joystick->getY());

    // 测试位置限制
    joystick->setPosition(1.5, -1.5);  // 超出范围
    EXPECT_DOUBLE_EQ(1.0, joystick->getX());  // 应该被限制在1.0
    EXPECT_DOUBLE_EQ(-1.0, joystick->getY()); // 应该被限制在-1.0
}

TEST_F(TestJoystick, TestMouseInteraction) {
    joystick->show();
    joystick->resize(100, 100);  // 设置一个固定大小以便测试

    // 模拟鼠标按下
    QTest::mousePress(joystick, Qt::LeftButton, Qt::NoModifier, QPoint(75, 75));
    EXPECT_TRUE(joystick->isPressed());

    // 模拟鼠标移动
    QTest::mouseMove(joystick, QPoint(80, 70));
    
    // 模拟鼠标释放
    QTest::mouseRelease(joystick, Qt::LeftButton, Qt::NoModifier, QPoint(80, 70));
    EXPECT_FALSE(joystick->isPressed());
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 