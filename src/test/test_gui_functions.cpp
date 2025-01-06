/**
 * @file test_gui_functions.cpp
 * @brief GUI功能测试程序
 */

#include <gtest/gtest.h>
#include <QApplication>
#include <QTest>
#include "ui/main_window.h"
#include "ui/navigation_panel.h"
#include "ros/robot_controller.h"

class GUITest : public ::testing::Test {
protected:
    void SetUp() override {
        // 初始化ROS节点
        int argc = 0;
        char** argv = nullptr;
        ros::init(argc, argv, "gui_test");
        nh_ = std::make_shared<ros::NodeHandle>();
        
        // 创建机器人控制器
        robot_controller_ = std::make_shared<RobotController>(*nh_);
        
        // 创建主窗口
        main_window_ = new MainWindow();
        main_window_->show();
        QTest::qWait(500); // 等待窗口显示
    }

    void TearDown() override {
        delete main_window_;
        ros::shutdown();
    }

    std::shared_ptr<ros::NodeHandle> nh_;
    std::shared_ptr<RobotController> robot_controller_;
    MainWindow* main_window_;
};

// 测试基本GUI显示
TEST_F(GUITest, TestGUIDisplay) {
    ASSERT_TRUE(main_window_->isVisible());
    ASSERT_FALSE(main_window_->isMinimized());
}

// 测试导航面板功能
TEST_F(GUITest, TestNavigationPanel) {
    NavigationPanel* nav_panel = main_window_->findChild<NavigationPanel*>();
    ASSERT_TRUE(nav_panel != nullptr);
    
    // 测试坐标输入
    QLineEdit* x_input = nav_panel->findChild<QLineEdit*>();
    ASSERT_TRUE(x_input != nullptr);
    QTest::keyClicks(x_input, "1.0");
    EXPECT_EQ(x_input->text(), QString("1.0"));
    
    // 测试按钮状态
    QPushButton* set_goal_btn = nav_panel->findChild<QPushButton*>("set_goal_btn_");
    QPushButton* cancel_btn = nav_panel->findChild<QPushButton*>("cancel_btn_");
    ASSERT_TRUE(set_goal_btn != nullptr);
    ASSERT_TRUE(cancel_btn != nullptr);
    EXPECT_TRUE(set_goal_btn->isEnabled());
    EXPECT_FALSE(cancel_btn->isEnabled());
}

// 测试键盘控制
TEST_F(GUITest, TestKeyboardControl) {
    // 模拟按键事件
    QTest::keyPress(main_window_, Qt::Key_W);
    QTest::qWait(100);
    EXPECT_NE(robot_controller_->getCurrentLinearSpeed(), 0.0);
    QTest::keyRelease(main_window_, Qt::Key_W);
    QTest::qWait(100);
    EXPECT_EQ(robot_controller_->getCurrentLinearSpeed(), 0.0);
}

// 测试虚拟摇杆
TEST_F(GUITest, TestJoystick) {
    JoystickWidget* joystick = main_window_->findChild<JoystickWidget*>();
    ASSERT_TRUE(joystick != nullptr);
    
    // 模拟鼠标事件
    QTest::mousePress(joystick, Qt::LeftButton, Qt::NoModifier, QPoint(75, 75));
    QTest::mouseMove(joystick, QPoint(75, 25));  // 向前移动
    QTest::qWait(100);
    EXPECT_GT(robot_controller_->getCurrentLinearSpeed(), 0.0);
    QTest::mouseRelease(joystick, Qt::LeftButton, Qt::NoModifier, QPoint(75, 25));
    QTest::qWait(100);
    EXPECT_EQ(robot_controller_->getCurrentLinearSpeed(), 0.0);
}

// 测试状态显示
TEST_F(GUITest, TestStatusDisplay) {
    RobotStatusPanel* status_panel = main_window_->findChild<RobotStatusPanel*>();
    ASSERT_TRUE(status_panel != nullptr);
    
    // 测试电池状态显示
    status_panel->updateBatteryLevel(85.5);
    QLabel* battery_label = status_panel->findChild<QLabel*>("battery_label_");
    ASSERT_TRUE(battery_label != nullptr);
    EXPECT_TRUE(battery_label->text().contains("85.5"));
}

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 