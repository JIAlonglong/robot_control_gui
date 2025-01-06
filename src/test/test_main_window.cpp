#include <gtest/gtest.h>
#include <QApplication>
#include <QTest>
#include "ui/main_window.h"

class TestMainWindow : public ::testing::Test {
protected:
    void SetUp() override {
        int argc = 0;
        char** argv = nullptr;
        app = new QApplication(argc, argv);
        ros::NodeHandle nh;
        window = new MainWindow(nh);
    }

    void TearDown() override {
        delete window;
        delete app;
    }

    QApplication* app;
    MainWindow* window;
};

TEST_F(TestMainWindow, TestInitialization) {
    ASSERT_TRUE(window != nullptr);
    EXPECT_TRUE(window->isVisible() == false);
}

TEST_F(TestMainWindow, TestWindowDisplay) {
    window->show();
    EXPECT_TRUE(window->isVisible());
    EXPECT_TRUE(window->size().width() > 0);
    EXPECT_TRUE(window->size().height() > 0);
}

TEST_F(TestMainWindow, TestKeyboardControl) {
    window->show();
    
    // 测试W键（前进）
    QTest::keyPress(window, Qt::Key_W);
    QTest::qWait(100);  // 等待更新
    QTest::keyRelease(window, Qt::Key_W);
    
    // 测试S键（后退）
    QTest::keyPress(window, Qt::Key_S);
    QTest::qWait(100);
    QTest::keyRelease(window, Qt::Key_S);
    
    // 测试A键（左转）
    QTest::keyPress(window, Qt::Key_A);
    QTest::qWait(100);
    QTest::keyRelease(window, Qt::Key_A);
    
    // 测试D键（右转）
    QTest::keyPress(window, Qt::Key_D);
    QTest::qWait(100);
    QTest::keyRelease(window, Qt::Key_D);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_main_window");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 