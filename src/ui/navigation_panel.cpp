/**
 * @file navigation_panel.cpp
 * @brief 导航控制面板类的实现
 */

#include "ui/navigation_panel.h"
#include "ui/rviz_view.h"
#include "ui/planner_settings_dialog.h"
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
    std::shared_ptr<RVizView> rviz_view;
};

NavigationPanel::NavigationPanel(std::shared_ptr<RobotController> robot_controller, QWidget* parent)
    : QWidget(parent)
    , d_ptr(new NavigationPanelPrivate(this))
{
    d_ptr->robot_controller = robot_controller;
    setupUi();
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

void NavigationPanel::setupUi()
{
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
        qDebug() << "Robot controller not initialized!";
        return;
    }
    
    // Connect navigation control buttons
    connect(d_ptr->start_navigation_button, &QPushButton::clicked, this, &NavigationPanel::onStartNavigation);
    connect(d_ptr->pause_navigation_button, &QPushButton::clicked, this, &NavigationPanel::onPauseNavigation);
    connect(d_ptr->stop_navigation_button, &QPushButton::clicked, this, &NavigationPanel::onStopNavigation);
    connect(d_ptr->emergency_stop_button, &QPushButton::clicked, this, &NavigationPanel::onEmergencyStop);

    // Connect robot controller signals
    connect(d_ptr->robot_controller.get(), &RobotController::localizationStateChanged,
            this, &NavigationPanel::onLocalizationStateChanged);
    connect(d_ptr->robot_controller.get(), &RobotController::localizationProgressChanged,
            this, &NavigationPanel::onLocalizationProgressChanged);
    connect(d_ptr->robot_controller.get(), &RobotController::navigationStateChanged,
            this, &NavigationPanel::onNavigationStateChanged);
    connect(d_ptr->robot_controller.get(), &RobotController::navigationProgressChanged,
            this, &NavigationPanel::onNavigationProgressChanged);
    connect(d_ptr->robot_controller.get(), &RobotController::distanceToGoalChanged,
            this, &NavigationPanel::onDistanceToGoalChanged);
    connect(d_ptr->robot_controller.get(), &RobotController::estimatedTimeToGoalChanged,
            this, &NavigationPanel::onEstimatedTimeToGoalChanged);
    connect(d_ptr->robot_controller.get(), &RobotController::velocityChanged,
            this, &NavigationPanel::updateVelocityDisplay);

    // Connect planner settings button
    connect(d_ptr->planner_settings_button, &QPushButton::clicked, 
            this, &NavigationPanel::onPlannerSettings);
}

void NavigationPanel::onSetInitialPose()
{
    if (!d_ptr->robot_controller || !d_ptr->rviz_view) {
        ROS_ERROR("Robot controller or RViz view not initialized");
        return;
    }

    // 启用初始位姿设置模式
    d_ptr->robot_controller->enableInitialPoseSetting(true);
    
    // 激活RViz中的初始位姿工具
    d_ptr->rviz_view->onInitialPoseToolActivated();
    
    // 更新UI状态
    updateLocalizationStatus("请在地图上点击设置初始位姿...");
    ROS_INFO("Initial pose setting mode activated");
}

void NavigationPanel::onAutoLocalization()
{
    if (!d_ptr->robot_controller) return;

    if (!d_ptr->is_localizing_) {
        // 开始自动定位
        d_ptr->is_localizing_ = true;
        d_ptr->auto_localization_button->setText("停止定位");
        updateLocalizationStatus("正在进行自动定位...");
        
        // 启动定位过程
        startAutoLocalization();
    } else {
        // 停止自动定位
        d_ptr->is_localizing_ = false;
        d_ptr->auto_localization_button->setText("自动定位");
        if (d_ptr->localization_timer) {
            d_ptr->localization_timer->stop();
        }
        d_ptr->robot_controller->publishVelocity(0.0, 0.0);
        updateLocalizationStatus("自动定位已停止");
    }
}

void NavigationPanel::startAutoLocalization()
{
    if (!d_ptr->localization_timer) {
        d_ptr->localization_timer = new QTimer(this);
        connect(d_ptr->localization_timer, &QTimer::timeout, this, [this]() {
            if (!d_ptr->is_localizing_) return;
            
            static int phase = 0;
            static int count = 0;
            const int PHASE_DURATION = 30; // 每个阶段持续3秒
            
            // 分阶段进行定位
            switch (phase) {
                case 0: // 原地旋转
                    d_ptr->robot_controller->publishVelocity(0.0, 0.5);
                    break;
                case 1: // 前进一小段
                    d_ptr->robot_controller->publishVelocity(0.1, 0.0);
                    break;
                case 2: // 反向旋转
                    d_ptr->robot_controller->publishVelocity(0.0, -0.5);
                    break;
                case 3: // 后退一小段
                    d_ptr->robot_controller->publishVelocity(-0.1, 0.0);
                    break;
            }
            
            count++;
            if (count >= PHASE_DURATION) {
                count = 0;
                phase = (phase + 1) % 4;
            }
        });
    }
    d_ptr->localization_timer->start(100);  // 100ms interval
}

void NavigationPanel::onSetGoal()
{
    if (!d_ptr->robot_controller || !d_ptr->rviz_view) {
        ROS_ERROR("Robot controller or RViz view not initialized");
        return;
    }

    // 启用目标点设置模式
    d_ptr->robot_controller->enableGoalSetting(true);
    
    // 激活RViz中的目标点工具
    d_ptr->rviz_view->onGoalToolActivated();
    
    ROS_INFO("Goal setting mode activated");
}

void NavigationPanel::onCancelGoal()
{
    if (d_ptr->robot_controller) {
        d_ptr->robot_controller->cancelNavigation();
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

void NavigationPanel::onNavigationStateChanged(RobotController::NavigationState state)
{
    switch (state) {
        case RobotController::NavigationState::IDLE:
            d_ptr->navigation_status_label->setText("导航状态: 空闲");
            d_ptr->start_navigation_button->setEnabled(true);
            d_ptr->pause_navigation_button->setEnabled(false);
            d_ptr->stop_navigation_button->setEnabled(false);
            d_ptr->set_goal_button->setEnabled(true);
            break;
            
        case RobotController::NavigationState::ACTIVE:
            d_ptr->navigation_status_label->setText("导航状态: 导航中");
            d_ptr->start_navigation_button->setEnabled(false);
            d_ptr->pause_navigation_button->setEnabled(true);
            d_ptr->stop_navigation_button->setEnabled(true);
            d_ptr->set_goal_button->setEnabled(false);
            break;
            
        case RobotController::NavigationState::PAUSED:
            d_ptr->navigation_status_label->setText("导航状态: 已暂停");
            d_ptr->start_navigation_button->setEnabled(true);
            d_ptr->pause_navigation_button->setEnabled(false);
            d_ptr->stop_navigation_button->setEnabled(true);
            d_ptr->set_goal_button->setEnabled(true);
            break;
            
        case RobotController::NavigationState::STOPPED:
            d_ptr->navigation_status_label->setText("导航状态: 已停止");
            d_ptr->start_navigation_button->setEnabled(true);
            d_ptr->pause_navigation_button->setEnabled(false);
            d_ptr->stop_navigation_button->setEnabled(false);
            d_ptr->set_goal_button->setEnabled(true);
            break;
            
        case RobotController::NavigationState::CANCELLED:
            d_ptr->navigation_status_label->setText("导航状态: 已取消");
            d_ptr->start_navigation_button->setEnabled(true);
            d_ptr->pause_navigation_button->setEnabled(false);
            d_ptr->stop_navigation_button->setEnabled(false);
            d_ptr->set_goal_button->setEnabled(true);
            break;
            
        case RobotController::NavigationState::SUCCEEDED:
            d_ptr->navigation_status_label->setText("导航状态: 已完成");
            d_ptr->start_navigation_button->setEnabled(false);
            d_ptr->pause_navigation_button->setEnabled(false);
            d_ptr->stop_navigation_button->setEnabled(false);
            d_ptr->set_goal_button->setEnabled(true);
            break;
            
        case RobotController::NavigationState::FAILED:
            d_ptr->navigation_status_label->setText("导航状态: 失败");
            d_ptr->start_navigation_button->setEnabled(false);
            d_ptr->pause_navigation_button->setEnabled(false);
            d_ptr->stop_navigation_button->setEnabled(false);
            d_ptr->set_goal_button->setEnabled(true);
            break;
    }
}

void NavigationPanel::onLocalizationStateChanged(bool is_localized)
{
    updateLocalizationState(is_localized);
    
    // 更新按钮状态
    d_ptr->set_initial_pose_button->setEnabled(true);
    d_ptr->auto_localization_button->setEnabled(!is_localized);
    
    if (is_localized) {
        d_ptr->set_goal_button->setEnabled(true);
        updateLocalizationStatus("定位成功");
    } else {
        d_ptr->set_goal_button->setEnabled(false);
        updateLocalizationStatus("未定位");
    }
}

void NavigationPanel::updateLocalizationState(bool is_localized)
{
    if (is_localized) {
        d_ptr->localization_status_label->setText("定位状态: 已定位");
        if (d_ptr->is_localizing_) {
            d_ptr->is_localizing_ = false;
            if (d_ptr->localization_timer) {
                d_ptr->localization_timer->stop();
                d_ptr->robot_controller->publishVelocity(0.0, 0.0);
            }
        }
    } else {
        d_ptr->localization_status_label->setText("定位状态: 未定位");
    }
    d_ptr->auto_localization_button->setEnabled(!is_localized);
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

void NavigationPanel::setRVizView(const std::shared_ptr<RVizView>& rviz_view)
{
    if (!rviz_view) return;
    
    // 保存RVizView指针
    d_ptr->rviz_view = rviz_view;
    
    // 连接初始位姿按钮点击信号
    connect(d_ptr->set_initial_pose_button, &QPushButton::clicked, this, &NavigationPanel::onSetInitialPose);
    
    // 连接目标点按钮点击信号
    connect(d_ptr->set_goal_button, &QPushButton::clicked, this, &NavigationPanel::onSetGoal);
    
    // 连接位姿选择信号
    connect(rviz_view.get(), &RVizView::initialPoseSelected, 
            [this](const geometry_msgs::PoseWithCovarianceStamped& pose) {
                if (d_ptr->robot_controller) {
                    d_ptr->robot_controller->setInitialPose(pose);
                    d_ptr->robot_controller->enableInitialPoseSetting(false);
                    updateLocalizationStatus("初始位姿已设置");
                }
            });
    
    connect(rviz_view.get(), &RVizView::goalSelected, 
            [this](const geometry_msgs::PoseStamped& goal) {
                if (d_ptr->robot_controller) {
                    d_ptr->robot_controller->setNavigationGoal(goal);
                    d_ptr->robot_controller->enableGoalSetting(false);
                }
            });
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