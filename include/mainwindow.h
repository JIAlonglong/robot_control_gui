/**
 * @file mainwindow.h
 * @brief 主窗口类的声明
 * @author Your Name
 * @date 2024-03-xx
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <memory>

namespace Ui {
class MainWindow;
}

/**
 * @brief 主窗口类
 * 
 * 该类负责管理整个应用程序的主界面，包括：
 * - 机器人控制面板
 * - 状态监控
 * - SLAM建图界面
 * - 传感器数据显示
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param parent 父窗口指针
     */
    explicit MainWindow(QWidget *parent = nullptr);
    
    /**
     * @brief 析构函数
     */
    ~MainWindow();

private slots:
    /**
     * @brief 初始化UI组件
     */
    void initializeUI();
    
    /**
     * @brief 初始化ROS连接
     */
    void initializeROS();

private:
    std::unique_ptr<Ui::MainWindow> ui;
};

#endif // MAINWINDOW_H 