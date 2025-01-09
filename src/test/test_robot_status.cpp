/**
 * @file test_robot_status.cpp
 * @brief 机器人状态面板测试程序
 */

#include <gtest/gtest.h>
#include <QApplication>
#include <QTest>
#include "ui/robot_status_panel.h"

class TestRobotStatus : public ::testing::Test {
protected:
    void SetUp() override {
        int argc = 0;
        char** argv = nullptr;
        app = new QApplication(argc, argv);
        panel = new RobotStatusPanel(nullptr);
    }

    void TearDown() override {
        delete panel;
        delete app;
    }

    QApplication* app;
    RobotStatusPanel* panel;
};

TEST_F(TestRobotStatus, TestInitialization) {
    ASSERT_TRUE(panel != nullptr);
    EXPECT_TRUE(panel->isVisible() == false);
}

TEST_F(TestRobotStatus, TestBatteryLevel) {
    // 测试电池电量显示
    panel->updateBatteryLevel(75.5);
    EXPECT_DOUBLE_EQ(75.5, panel->getBatteryLevel());

    // 测试最大值
    panel->updateBatteryLevel(100.0);
    EXPECT_DOUBLE_EQ(100.0, panel->getBatteryLevel());

    // 测试最小值
    panel->updateBatteryLevel(0.0);
    EXPECT_DOUBLE_EQ(0.0, panel->getBatteryLevel());
}

TEST_F(TestRobotStatus, TestConnectionStatus) {
    // 测试连接状态
    panel->setConnected(true);
    EXPECT_TRUE(panel->isConnected());
    
    panel->setConnected(false);
    EXPECT_FALSE(panel->isConnected());
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 