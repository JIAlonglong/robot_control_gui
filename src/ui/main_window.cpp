/**
 * @file main_window.cpp
 * @brief 主窗口类的实现
 */

#include "ui/main_window.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <cmath>
#include <ros/ros.h>
#include "ros/robot_controller.h"

MainWindow::MainWindow(ros::NodeHandle& nh, QWidget* parent)
    : QMainWindow(parent)
    , current_linear_speed_(0.0)
    , current_angular_speed_(0.0)
    , battery_level_(100)
    , is_auto_mode_(false)
{
    // 初始化ROS控制器
    robot_controller_ = std::make_unique<RobotController>(nh);

    // 创建中心部件
    QWidget* central = new QWidget(this);
    setCentralWidget(central);

    // 创建主布局
    QVBoxLayout* main_layout = new QVBoxLayout(central);
    main_layout->setContentsMargins(10, 10, 10, 10);
    main_layout->setSpacing(10);

    // 创建并添加组件
    joystick_ = new JoystickWidget(this);
    joystick_->setToolTip("Movement Control (WASD Keys)\nW: Forward\nS: Backward\nA: Left\nD: Right");
    joystick_->setFixedSize(150, 150);
    
    rotation_joystick_ = new JoystickWidget(this);
    rotation_joystick_->setToolTip("Rotation Control (Arrow Keys)\nLeft: Rotate Left\nRight: Rotate Right");
    rotation_joystick_->setFixedSize(150, 150);
    
    status_panel_ = new RobotStatusPanel(this);
    speed_dashboard_ = new SpeedDashboard(this);
    speed_dashboard_->setFixedHeight(100);
    map_view_ = new MapView(this);

    // 创建顶部状态栏布局
    QHBoxLayout* status_layout = new QHBoxLayout();
    status_layout->addWidget(status_panel_, 2);
    status_layout->addWidget(speed_dashboard_, 1);

    // 创建底部控制布局
    QHBoxLayout* control_layout = new QHBoxLayout();
    
    // 左侧移动控制组
    QVBoxLayout* left_control = new QVBoxLayout();
    QLabel* move_label = new QLabel("Movement Control (WASD)", this);
    move_label->setAlignment(Qt::AlignCenter);
    left_control->addWidget(move_label);
    left_control->addWidget(joystick_, 0, Qt::AlignCenter);
    
    // 右侧旋转控制组
    QVBoxLayout* right_control = new QVBoxLayout();
    QLabel* rotate_label = new QLabel("Rotation Control (←→)", this);
    rotate_label->setAlignment(Qt::AlignCenter);
    right_control->addWidget(rotate_label);
    right_control->addWidget(rotation_joystick_, 0, Qt::AlignCenter);

    // 添加到底部控制布局
    control_layout->addLayout(left_control, 1);
    control_layout->addStretch(3);  // 中间留空
    control_layout->addLayout(right_control, 1);

    // 添加到主布局
    main_layout->addLayout(status_layout);
    main_layout->addWidget(map_view_, 1);
    main_layout->addLayout(control_layout);

    // 设置窗口属性
    setWindowTitle("Robot Control GUI");
    resize(1200, 800);

    // 设置样式表
    setStyleSheet(R"(
        QLabel {
            font-size: 12px;
            font-weight: bold;
            color: #333333;
            margin-bottom: 5px;
        }
        QWidget {
            background-color: #f0f0f0;
        }
        RobotStatusPanel, SpeedDashboard {
            background-color: white;
            border: 1px solid #cccccc;
            border-radius: 5px;
        }
        MapView {
            border: 1px solid #cccccc;
            border-radius: 5px;
            background-color: white;
        }
    )");

    // 连接摇杆信号
    connect(joystick_, &JoystickWidget::positionChanged,
            this, &MainWindow::onJoystickMoved);
    connect(rotation_joystick_, &JoystickWidget::positionChanged,
            this, &MainWindow::onRotationJoystickMoved);

    // 创建并启动状态更新定时器
    status_timer_ = new QTimer(this);
    connect(status_timer_, &QTimer::timeout,
            this, &MainWindow::updateRobotStatus);
    status_timer_->start(100);  // 10Hz

    // 创建并启动键盘控制更新定时器
    keyboard_timer_ = new QTimer(this);
    connect(keyboard_timer_, &QTimer::timeout,
            this, &MainWindow::updateKeyboardControl);
    keyboard_timer_->start(50);  // 20Hz

    // 初始化显示
    updateRobotStatus();

    // 设置键盘焦点策略
    setFocusPolicy(Qt::StrongFocus);
}

void MainWindow::keyPressEvent(QKeyEvent* event)
{
    // 只处理我们关心的按键
    switch (event->key()) {
        case Qt::Key_W:
        case Qt::Key_S:
        case Qt::Key_A:
        case Qt::Key_D:
        case Qt::Key_Left:
        case Qt::Key_Right:
            pressed_keys_.insert(event->key());
            break;
    }
    QMainWindow::keyPressEvent(event);
}

void MainWindow::keyReleaseEvent(QKeyEvent* event)
{
    pressed_keys_.remove(event->key());
    QMainWindow::keyReleaseEvent(event);
}

void MainWindow::updateKeyboardControl()
{
    // 如果没有任何按键按下，且摇杆没有被按下，才发送零速度
    if (pressed_keys_.isEmpty() && !joystick_->property("is_pressed").toBool() && 
        !rotation_joystick_->property("is_pressed").toBool()) {
        current_linear_speed_ = 0.0;
        current_angular_speed_ = 0.0;
        speed_dashboard_->setLinearSpeed(0.0);
        speed_dashboard_->setAngularSpeed(0.0);
        status_panel_->updateVelocity(current_linear_speed_, current_angular_speed_);
        robot_controller_->publishVelocity(current_linear_speed_, current_angular_speed_);
        return;
    }

    // 处理键盘输入
    if (!pressed_keys_.isEmpty()) {
        // 处理移动控制
        double x = 0.0, y = 0.0;
        
        // 前后移动
        if (pressed_keys_.contains(Qt::Key_W) && !pressed_keys_.contains(Qt::Key_S)) {
            y = -1.0;  // 前进
        } else if (pressed_keys_.contains(Qt::Key_S) && !pressed_keys_.contains(Qt::Key_W)) {
            y = 1.0;   // 后退
        }
        
        // 左右移动
        if (pressed_keys_.contains(Qt::Key_A) && !pressed_keys_.contains(Qt::Key_D)) {
            x = -1.0;  // 左移
        } else if (pressed_keys_.contains(Qt::Key_D) && !pressed_keys_.contains(Qt::Key_A)) {
            x = 1.0;   // 右移
        }

        // 如果有移动输入，更新移动摇杆
        if (x != 0.0 || y != 0.0) {
            // 标准化向量以确保对角线移动速度不会过快
            double length = std::sqrt(x*x + y*y);
            if (length > 1.0) {
                x /= length;
                y /= length;
            }
            joystick_->setPosition(x, y);  // 更新摇杆位置
        }

        // 处理旋转控制
        double rot_x = 0.0;
        if (pressed_keys_.contains(Qt::Key_Left) && !pressed_keys_.contains(Qt::Key_Right)) {
            rot_x = 1.0;  // 左转
        } else if (pressed_keys_.contains(Qt::Key_Right) && !pressed_keys_.contains(Qt::Key_Left)) {
            rot_x = -1.0; // 右转
        }

        // 如果有旋转输入，更新旋转摇杆
        if (rot_x != 0.0) {
            rotation_joystick_->setPosition(rot_x, 0.0);  // 更新摇杆位置
        }
    }

    // 发布最新的速度命令
    robot_controller_->publishVelocity(current_linear_speed_, current_angular_speed_);
}

void MainWindow::onJoystickMoved(double x, double y)
{
    // 将移动摇杆位置转换为线速度命令
    const double MAX_LINEAR_SPEED = 2.0;   // m/s
    current_linear_speed_ = -y * MAX_LINEAR_SPEED;   // 向上为负，所以需要取反

    // 更新速度仪表盘和状态面板
    speed_dashboard_->setLinearSpeed(std::abs(current_linear_speed_));
    status_panel_->updateVelocity(current_linear_speed_, current_angular_speed_);

    // 立即发布速度命令
    robot_controller_->publishVelocity(current_linear_speed_, current_angular_speed_);
}

void MainWindow::onRotationJoystickMoved(double x, double y)
{
    // 将旋转摇杆位置转换为角速度命令
    const double MAX_ANGULAR_SPEED = 1.0;  // rad/s
    current_angular_speed_ = -x * MAX_ANGULAR_SPEED; // 向左为负，所以需要取反

    // 更新速度仪表盘和状态面板
    speed_dashboard_->setAngularSpeed(std::abs(current_angular_speed_));
    status_panel_->updateVelocity(current_linear_speed_, current_angular_speed_);

    // 立即发布速度命令
    robot_controller_->publishVelocity(current_linear_speed_, current_angular_speed_);
}

void MainWindow::updateRobotStatus()
{
    // 更新实际的机器人状态
    status_panel_->updateBatteryLevel(robot_controller_->getBatteryLevel());
    status_panel_->updateVelocity(
        robot_controller_->getCurrentLinearSpeed(),
        robot_controller_->getCurrentAngularSpeed()
    );

    // 更新地图显示
    if (auto odom = robot_controller_->getLatestOdom()) {
        map_view_->updateRobotPose(odom);
    }
    if (auto scan = robot_controller_->getLatestScan()) {
        map_view_->updateLaserScan(scan);
    }
    if (auto map = robot_controller_->getLatestMap()) {
        map_view_->updateMap(map);
    }
} 