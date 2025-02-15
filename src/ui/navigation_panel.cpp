/*
 * Copyright (c) 2025 JIAlonglong
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file navigation_panel.cpp
 * @brief 导航控制面板类的实现
 */

#include "ui/navigation_panel.h"
#include "ui/rviz_view.h"
#include "ui/planner_settings_dialog.h"
#include "ui/goal_setting_dialog.h"
#include "ros/robot_controller.h"
#include <QWidget>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QProgressBar>
#include <QKeyEvent>
#include <QTimer>
#include <QDebug>
#include <QString>
#include <QMetaObject>

class NavigationPanel::NavigationPanelPrivate {
public:
    NavigationPanelPrivate(NavigationPanel* q) : q_ptr(q) {}

    NavigationPanel* q_ptr;
    std::shared_ptr<RobotController> robot_controller;

    // Navigation tools
    QPushButton* set_initial_pose_button{nullptr};
    QPushButton* auto_localization_button{nullptr};
    QPushButton* set_goal_button{nullptr};
    QPushButton* cancel_goal_button{nullptr};
    QPushButton* planner_settings_button{nullptr};

    // Navigation control
    QPushButton* start_navigation_button{nullptr};
    QPushButton* pause_navigation_button{nullptr};
    QPushButton* stop_navigation_button{nullptr};
    QPushButton* emergency_stop_button{nullptr};

    // Status display
    QLabel* localization_status_label{nullptr};
    QLabel* navigation_status_label{nullptr};
    QProgressBar* localization_progress_bar{nullptr};
    QProgressBar* navigation_progress_bar{nullptr};
    QLabel* distance_label{nullptr};
    QLabel* estimated_time_label{nullptr};
    QLabel* linear_velocity_label{nullptr};
    QLabel* angular_velocity_label{nullptr};

    // Keyboard control
    std::map<int, bool> key_pressed_;
    double keyboard_linear_speed_{0.5};
    double keyboard_angular_speed_{1.0};

    // Localization
    QTimer* localization_timer{nullptr};
    bool is_localizing_{false};

    // RVizView
    RVizView* rviz_view_{nullptr};
};

NavigationPanel::NavigationPanel(std::shared_ptr<RobotController> robot_controller, QWidget* parent)
    : QWidget(parent)
    , d_ptr(new NavigationPanelPrivate(this))
{
    d_ptr->robot_controller = robot_controller;
    setupUI();
    setupJoystick();
    setupKeyboardControl();
    connectSignalsAndSlots();
    setFocusPolicy(Qt::StrongFocus);
}

NavigationPanel::~NavigationPanel()
{
    if (d_ptr->localization_timer) {
        d_ptr->localization_timer->stop();
        delete d_ptr->localization_timer;
    }
}

void NavigationPanel::setupUI()
{
    // 创建主布局
    auto* main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(0, 0, 0, 0);
    main_layout->setSpacing(10);

    // Navigation tools group
    auto* tools_group = new QGroupBox("导航工具", this);
    auto* tools_layout = new QHBoxLayout(tools_group);

    d_ptr->set_initial_pose_button = new QPushButton("设置初始位姿", this);
    d_ptr->auto_localization_button = new QPushButton("自动定位", this);
    d_ptr->set_goal_button = new QPushButton("设置目标点", this);
    d_ptr->cancel_goal_button = new QPushButton("取消目标点", this);
    d_ptr->planner_settings_button = new QPushButton("规划器设置", this);
    d_ptr->planner_settings_button->setToolTip("设置全局和局部路径规划器");

    tools_layout->addWidget(d_ptr->set_initial_pose_button);
    tools_layout->addWidget(d_ptr->auto_localization_button);
    tools_layout->addWidget(d_ptr->set_goal_button);
    tools_layout->addWidget(d_ptr->cancel_goal_button);
    tools_layout->addWidget(d_ptr->planner_settings_button);

    main_layout->addWidget(tools_group);

    // Navigation control group
    auto* control_group = new QGroupBox("导航控制", this);
    auto* control_layout = new QHBoxLayout(control_group);

    d_ptr->start_navigation_button = new QPushButton("开始导航", this);
    d_ptr->start_navigation_button->setObjectName("start_navigation_button");
    d_ptr->start_navigation_button->setEnabled(false);
    d_ptr->pause_navigation_button = new QPushButton("暂停导航", this);
    d_ptr->stop_navigation_button = new QPushButton("停止导航", this);
    d_ptr->emergency_stop_button = new QPushButton("紧急停止", this);
    d_ptr->emergency_stop_button->setStyleSheet("background-color: red; color: white;");

    control_layout->addWidget(d_ptr->start_navigation_button);
    control_layout->addWidget(d_ptr->pause_navigation_button);
    control_layout->addWidget(d_ptr->stop_navigation_button);
    control_layout->addWidget(d_ptr->emergency_stop_button);

    main_layout->addWidget(control_group);

    // Status display group
    auto* status_group = new QGroupBox("状态显示", this);
    auto* status_layout = new QVBoxLayout(status_group);

    d_ptr->localization_status_label = new QLabel("定位状态: 未定位", this);
    d_ptr->navigation_status_label = new QLabel("导航状态: 未开始", this);
    d_ptr->localization_progress_bar = new QProgressBar(this);
    d_ptr->navigation_progress_bar = new QProgressBar(this);
    d_ptr->distance_label = new QLabel("距离目标点: 0.00 米", this);
    d_ptr->estimated_time_label = new QLabel("预计到达时间: 0.0 秒", this);
    d_ptr->linear_velocity_label = new QLabel("线速度: 0.00 m/s", this);
    d_ptr->angular_velocity_label = new QLabel("角速度: 0.00 rad/s", this);

    status_layout->addWidget(d_ptr->localization_status_label);
    status_layout->addWidget(d_ptr->navigation_status_label);
    status_layout->addWidget(d_ptr->localization_progress_bar);
    status_layout->addWidget(d_ptr->navigation_progress_bar);
    status_layout->addWidget(d_ptr->distance_label);
    status_layout->addWidget(d_ptr->estimated_time_label);
    status_layout->addWidget(d_ptr->linear_velocity_label);
    status_layout->addWidget(d_ptr->angular_velocity_label);

    main_layout->addWidget(status_group);

    // 如果有RViz视图，添加到布局
    if (d_ptr->rviz_view_) {
        main_layout->addWidget(d_ptr->rviz_view_);
    }

    setLayout(main_layout);
}

void NavigationPanel::setupJoystick()
{
    // TODO: Implement joystick setup
}

void NavigationPanel::setupKeyboardControl()
{
    d_ptr->key_pressed_[Qt::Key_W] = false;
    d_ptr->key_pressed_[Qt::Key_S] = false;
    d_ptr->key_pressed_[Qt::Key_A] = false;
    d_ptr->key_pressed_[Qt::Key_D] = false;
    
    // 设置默认速度
    d_ptr->keyboard_linear_speed_ = 0.2;  // 默认线速度 0.2 m/s
    d_ptr->keyboard_angular_speed_ = 0.5; // 默认角速度 0.5 rad/s
}

void NavigationPanel::connectSignalsAndSlots()
{
    if (!d_ptr->robot_controller) {
        ROS_ERROR("Robot controller not initialized");
        return;
    }

    ROS_INFO("Connecting signals and slots in NavigationPanel");

    // 连接RobotController的信号
    connect(d_ptr->robot_controller.get(), &RobotController::localizationStateChanged,
            this, &NavigationPanel::onLocalizationStateChanged);
    connect(d_ptr->robot_controller.get(), &RobotController::localizationProgressChanged,
            this, &NavigationPanel::onLocalizationProgressChanged);
    connect(d_ptr->robot_controller.get(), &RobotController::localizationStatusChanged,
            this, &NavigationPanel::updateLocalizationStatus);
    connect(d_ptr->robot_controller.get(), &RobotController::navigationStateChanged,
            this, &NavigationPanel::onNavigationStateChanged);
    connect(d_ptr->robot_controller.get(), &RobotController::navigationProgressChanged,
            this, &NavigationPanel::onNavigationProgressChanged);
    connect(d_ptr->robot_controller.get(), &RobotController::navigationStatusChanged,
            this, &NavigationPanel::onNavigationStatusChanged);
    
    // 连接目标点设置信号
    connect(d_ptr->robot_controller.get(), &RobotController::goalSet,
            this, [this](const geometry_msgs::PoseStamped& goal) {
                ROS_INFO("Goal set signal received in NavigationPanel");
                if (d_ptr->start_navigation_button) {
                    d_ptr->start_navigation_button->setEnabled(true);
                    ROS_INFO("Start navigation button enabled in NavigationPanel");
                } else {
                    ROS_ERROR("Start navigation button is null in NavigationPanel!");
                }
                
                // 更新导航状态
                onNavigationStateChanged("就绪");
                ROS_INFO("Navigation state updated to '就绪'");
                
                // 更新状态标签
                QString status_msg = QString("已设置导航目标: (%1, %2)")
                    .arg(goal.pose.position.x, 0, 'f', 3)
                    .arg(goal.pose.position.y, 0, 'f', 3);
                onNavigationStatusChanged(status_msg);
                ROS_INFO_STREAM("Status updated: " << status_msg.toStdString());
            });

    // 连接按钮点击事件
    connect(d_ptr->set_initial_pose_button, &QPushButton::clicked,
            this, &NavigationPanel::onSetInitialPose);
    connect(d_ptr->set_goal_button, &QPushButton::clicked,
            this, &NavigationPanel::onSetGoal);

    // 连接按钮点击事件
    connect(d_ptr->auto_localization_button, &QPushButton::clicked, 
            this, &NavigationPanel::onAutoLocalization);
    connect(d_ptr->cancel_goal_button, &QPushButton::clicked, 
            this, &NavigationPanel::onCancelGoal);
    connect(d_ptr->start_navigation_button, &QPushButton::clicked, 
            this, &NavigationPanel::onStartNavigation);
    connect(d_ptr->pause_navigation_button, &QPushButton::clicked, 
            this, &NavigationPanel::onPauseNavigation);
    connect(d_ptr->stop_navigation_button, &QPushButton::clicked, 
            this, &NavigationPanel::onStopNavigation);
    connect(d_ptr->emergency_stop_button, &QPushButton::clicked, 
            this, &NavigationPanel::onEmergencyStop);
    connect(d_ptr->planner_settings_button, &QPushButton::clicked, 
            this, &NavigationPanel::onPlannerSettings);

    ROS_INFO("All signals and slots connected in NavigationPanel");
}

void NavigationPanel::onSetInitialPose()
{
    if (!d_ptr->robot_controller) {
        ROS_ERROR("Robot controller not initialized");
        return;
    }

    // 创建并显示初始位姿设置对话框
    InitialPoseDialog dialog(this);
    if (dialog.exec() == QDialog::Accepted) {
        // 获取用户设置的初始位姿
        geometry_msgs::PoseWithCovarianceStamped pose = dialog.getPose();
        
        // 设置初始位姿
        d_ptr->robot_controller->setInitialPose(pose);
        ROS_INFO("Set initial pose from dialog: x=%.3f, y=%.3f", 
                 pose.pose.pose.position.x, 
                 pose.pose.pose.position.y);
    }
}

void NavigationPanel::onAutoLocalization()
{
    if (!d_ptr->robot_controller) return;

    if (!d_ptr->is_localizing_) {
        // 开始自动定位
        d_ptr->robot_controller->startAutoLocalization();
        d_ptr->auto_localization_button->setText(tr("停止自动定位"));
        d_ptr->is_localizing_ = true;
        ROS_INFO("Started auto localization");
    } else {
        // 停止自动定位
        d_ptr->robot_controller->stopAutoLocalization();
        d_ptr->auto_localization_button->setText(tr("自动定位"));
        d_ptr->is_localizing_ = false;
        ROS_INFO("Stopped auto localization");
    }
}

void NavigationPanel::onSetGoal()
{
    if (!d_ptr->robot_controller) {
        ROS_ERROR("Robot controller not initialized");
        return;
    }

    // 获取当前机器人位置作为参考
    geometry_msgs::Pose current_pose = d_ptr->robot_controller->getCurrentPose();
    
    // 创建并显示目标点设置对话框
    GoalSettingDialog dialog(current_pose, this);
    if (dialog.exec() == QDialog::Accepted) {
        // 获取用户设置的目标点
        geometry_msgs::PoseStamped goal = dialog.getGoal();
        
        // 设置导航目标点
        d_ptr->robot_controller->setNavigationGoal(goal);
        ROS_INFO("Set navigation goal from dialog: x=%.3f, y=%.3f",
                 goal.pose.position.x,
                 goal.pose.position.y);
    }
}

void NavigationPanel::onCancelGoal()
{
    if (d_ptr->robot_controller) {
        d_ptr->robot_controller->stopNavigation();
    }
}

void NavigationPanel::onStartNavigation()
{
    if (d_ptr->robot_controller) {
        d_ptr->robot_controller->startNavigation();
    }
}

void NavigationPanel::onPauseNavigation()
{
    if (d_ptr->robot_controller) {
        d_ptr->robot_controller->pauseNavigation();
    }
}

void NavigationPanel::onStopNavigation()
{
    if (d_ptr->robot_controller) {
        d_ptr->robot_controller->stopNavigation();
    }
}

void NavigationPanel::onNavigationStateChanged(const QString& state)
{
    if (!d_ptr->navigation_status_label) return;
    
    d_ptr->navigation_status_label->setText("导航状态: " + state);
    
    // 根据状态更新按钮状态
    if (state == "就绪") {
        // 当状态为就绪时，启用开始导航按钮
        if (d_ptr->start_navigation_button) {
            d_ptr->start_navigation_button->setEnabled(true);
            ROS_INFO("Navigation start button enabled");
        }
        if (d_ptr->stop_navigation_button) {
            d_ptr->stop_navigation_button->setEnabled(false);
        }
        if (d_ptr->pause_navigation_button) {
            d_ptr->pause_navigation_button->setEnabled(false);
        }
        if (d_ptr->set_goal_button) {
            d_ptr->set_goal_button->setEnabled(true);
        }
    } else if (state == "进行中") {
        if (d_ptr->start_navigation_button) {
            d_ptr->start_navigation_button->setEnabled(false);
        }
        if (d_ptr->stop_navigation_button) {
            d_ptr->stop_navigation_button->setEnabled(true);
        }
        if (d_ptr->pause_navigation_button) {
            d_ptr->pause_navigation_button->setEnabled(true);
        }
        if (d_ptr->set_goal_button) {
            d_ptr->set_goal_button->setEnabled(false);
        }
    } else if (state == "已暂停") {
        if (d_ptr->start_navigation_button) {
            d_ptr->start_navigation_button->setEnabled(true);
        }
        if (d_ptr->stop_navigation_button) {
            d_ptr->stop_navigation_button->setEnabled(true);
        }
        if (d_ptr->pause_navigation_button) {
            d_ptr->pause_navigation_button->setEnabled(false);
        }
        if (d_ptr->set_goal_button) {
            d_ptr->set_goal_button->setEnabled(true);
        }
    } else if (state == "已完成" || state == "已停止" || state == "已取消" || state == "已中止") {
        if (d_ptr->start_navigation_button) {
            d_ptr->start_navigation_button->setEnabled(false);
        }
        if (d_ptr->stop_navigation_button) {
            d_ptr->stop_navigation_button->setEnabled(false);
        }
        if (d_ptr->pause_navigation_button) {
            d_ptr->pause_navigation_button->setEnabled(false);
        }
        if (d_ptr->set_goal_button) {
            d_ptr->set_goal_button->setEnabled(true);
        }
    }
    
    ROS_INFO_STREAM("Navigation state changed to: " << state.toStdString());
}

void NavigationPanel::onLocalizationStateChanged(const QString& state)
{
    if (state == "已完成" || state == "已取消") {
        // 重置自动定位按钮状态
        d_ptr->is_localizing_ = false;
        d_ptr->auto_localization_button->setText(tr("自动定位"));
        d_ptr->auto_localization_button->setEnabled(true);
        d_ptr->set_initial_pose_button->setEnabled(true);
        ROS_INFO("Reset auto localization button state");
    }
    
    if (d_ptr->localization_status_label) {
        d_ptr->localization_status_label->setText("定位状态: " + state);
    }
}

void NavigationPanel::onLocalizationProgressChanged(double progress)
{
    updateLocalizationProgress(progress);
}

void NavigationPanel::updateLocalizationProgress(double progress)
{
    if (d_ptr->localization_progress_bar) {
        d_ptr->localization_progress_bar->setValue(static_cast<int>(progress));
        if (progress >= 100.0) {
            updateLocalizationStatus("定位完成");
            d_ptr->set_initial_pose_button->setEnabled(true);
            d_ptr->auto_localization_button->setEnabled(false);
        }
    }
}

void NavigationPanel::onNavigationProgressChanged(double progress)
{
    if (d_ptr->navigation_progress_bar) {
        d_ptr->navigation_progress_bar->setValue(static_cast<int>(progress * 100));
    }
}

void NavigationPanel::onDistanceToGoalChanged(double distance)
{
    if (d_ptr->distance_label) {
        d_ptr->distance_label->setText(QString("距离目标点: %1 米").arg(distance, 0, 'f', 2));
    }
}

void NavigationPanel::onEstimatedTimeToGoalChanged(double time)
{
    if (d_ptr->estimated_time_label) {
        d_ptr->estimated_time_label->setText(QString("预计到达时间: %1 秒").arg(time, 0, 'f', 1));
    }
}

void NavigationPanel::updateVelocityDisplay(double linear, double angular)
{
    if (d_ptr->linear_velocity_label) {
        d_ptr->linear_velocity_label->setText(QString("线速度: %1 m/s").arg(linear, 0, 'f', 2));
    }
    if (d_ptr->angular_velocity_label) {
        d_ptr->angular_velocity_label->setText(QString("角速度: %1 rad/s").arg(angular, 0, 'f', 2));
    }
}

void NavigationPanel::onEmergencyStop()
{
    if (!d_ptr->robot_controller) return;
    
    // 停止所有运动
    d_ptr->robot_controller->publishVelocity(0.0, 0.0);
    
    // 重置所有按键状态
    d_ptr->key_pressed_[Qt::Key_W] = false;
    d_ptr->key_pressed_[Qt::Key_S] = false;
    d_ptr->key_pressed_[Qt::Key_A] = false;
    d_ptr->key_pressed_[Qt::Key_D] = false;
    
    // 更新显示
    d_ptr->linear_velocity_label->setText("线速度: 0.00 m/s");
    d_ptr->angular_velocity_label->setText("角速度: 0.00 rad/s");
}

void NavigationPanel::onJoystickMoved()
{
    // TODO: Implement joystick movement handling
}

void NavigationPanel::keyPressEvent(QKeyEvent* event)
{
    if (!event->isAutoRepeat()) {
        switch (event->key()) {
            case Qt::Key_W:
                d_ptr->key_pressed_[Qt::Key_W] = true;
                break;
            case Qt::Key_S:
                d_ptr->key_pressed_[Qt::Key_S] = true;
                break;
            case Qt::Key_A:
                d_ptr->key_pressed_[Qt::Key_A] = true;
                break;
            case Qt::Key_D:
                d_ptr->key_pressed_[Qt::Key_D] = true;
                break;
            case Qt::Key_Space:  // 空格键作为紧急停止
                onEmergencyStop();
                break;
            case Qt::Key_Up:     // 增加线速度
                d_ptr->keyboard_linear_speed_ = std::min(d_ptr->keyboard_linear_speed_ + 0.1, 1.0);
                break;
            case Qt::Key_Down:   // 减小线速度
                d_ptr->keyboard_linear_speed_ = std::max(d_ptr->keyboard_linear_speed_ - 0.1, 0.1);
                break;
            case Qt::Key_Left:   // 增加角速度
                d_ptr->keyboard_angular_speed_ = std::min(d_ptr->keyboard_angular_speed_ + 0.1, 2.0);
                break;
            case Qt::Key_Right:  // 减小角速度
                d_ptr->keyboard_angular_speed_ = std::max(d_ptr->keyboard_angular_speed_ - 0.1, 0.1);
                break;
        }
        updateKeyboardVelocity();
    }
    QWidget::keyPressEvent(event);
}

void NavigationPanel::keyReleaseEvent(QKeyEvent* event)
{
    if (!event->isAutoRepeat()) {
        switch (event->key()) {
            case Qt::Key_W:
                d_ptr->key_pressed_[Qt::Key_W] = false;
                break;
            case Qt::Key_S:
                d_ptr->key_pressed_[Qt::Key_S] = false;
                break;
            case Qt::Key_A:
                d_ptr->key_pressed_[Qt::Key_A] = false;
                break;
            case Qt::Key_D:
                d_ptr->key_pressed_[Qt::Key_D] = false;
                break;
        }
        updateKeyboardVelocity();
    }
    QWidget::keyReleaseEvent(event);
}

void NavigationPanel::updateKeyboardVelocity()
{
    if (!d_ptr->robot_controller) return;
    
    double linear = 0.0;
    double angular = 0.0;
    
    if (d_ptr->key_pressed_[Qt::Key_W]) {
        linear += d_ptr->keyboard_linear_speed_;
    }
    if (d_ptr->key_pressed_[Qt::Key_S]) {
        linear -= d_ptr->keyboard_linear_speed_;
    }
    if (d_ptr->key_pressed_[Qt::Key_A]) {
        angular += d_ptr->keyboard_angular_speed_;
    }
    if (d_ptr->key_pressed_[Qt::Key_D]) {
        angular -= d_ptr->keyboard_angular_speed_;
    }
    
    d_ptr->robot_controller->publishVelocity(linear, angular);
    
    // 更新速度显示
    d_ptr->linear_velocity_label->setText(QString("线速度: %1 m/s").arg(linear, 0, 'f', 2));
    d_ptr->angular_velocity_label->setText(QString("角速度: %1 rad/s").arg(angular, 0, 'f', 2));
}

void NavigationPanel::onNavigationModeChanged(int index)
{
    if (d_ptr->robot_controller) {
        d_ptr->robot_controller->setNavigationMode(index);
    }
}

void NavigationPanel::updateLocalizationStatus(const QString& status)
{
    if (d_ptr->localization_status_label) {
        d_ptr->localization_status_label->setText(status);
    }
}

void NavigationPanel::setRVizView(RVizView* view)
{
    if (!d_ptr) return;
    d_ptr->rviz_view_ = view;
}

void NavigationPanel::onPlannerSettings()
{
    if (!d_ptr->robot_controller) {
        return;
    }

    // 创建并显示规划器设置对话框
    PlannerSettingsDialog dialog(d_ptr->robot_controller, this);
    dialog.exec();
}

void NavigationPanel::onNavigationStatusChanged(const QString& status)
{
    if (d_ptr->navigation_status_label) {
        d_ptr->navigation_status_label->setText(status);
    }
} 