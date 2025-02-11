/**
 * @file navigation_panel.cpp
 * @brief 导航控制面板类的实现
 */

#include "ui/navigation_panel.h"
#include "ros/robot_controller.h"
#include "ui/goal_setting_dialog.h"
#include "ui/planner_settings_dialog.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QGroupBox>
#include <QPushButton>
#include <QComboBox>
#include <QProgressBar>
#include <QKeyEvent>
#include <QTimer>
#include <QDebug>
#include <QString>
#include <QMetaObject>
#include <QMessageBox>
#include <QMutexLocker>
#include <QIcon>

NavigationPanel::NavigationPanel(const std::shared_ptr<RobotController>& controller, QWidget* parent)
    : QWidget(parent)
    , d_(std::make_unique<NavigationPanelPrivate>(this))
{
    d_->robot_controller = controller;
    setupUi();
    connectSignalsAndSlots();
    updateButtons();
}

NavigationPanel::~NavigationPanel() = default;

void NavigationPanel::setupUi()
{
    d_->main_layout_ = new QVBoxLayout(this);
    createControlGroup();
    createStatusGroup();
    createPlannerGroup();
    d_->main_layout_->addStretch();
}

void NavigationPanel::createControlGroup()
{
    d_->control_group_ = new QGroupBox(tr("导航控制"), this);
    auto* layout = new QGridLayout(d_->control_group_);

    // 初始化位姿设置
    d_->set_initial_pose_button = new QPushButton(tr("设置初始位姿"), this);
    d_->auto_localization_button = new QPushButton(tr("启用自动定位"), this);
    d_->auto_localization_button->setCheckable(true);
    d_->auto_localization_button->setToolTip(tr("在导航过程中自动纠正机器人定位"));
    
    // 导航目标设置
    d_->set_goal_button = new QPushButton(tr("设置目标点"), this);
    d_->cancel_goal_button = new QPushButton(tr("取消目标点"), this);
    
    // 导航控制
    d_->start_navigation_button = new QPushButton(tr("开始导航"), this);
    d_->stop_navigation_button = new QPushButton(tr("停止导航"), this);
    d_->pause_navigation_button = new QPushButton(tr("暂停导航"), this);
    d_->emergency_stop_button = new QPushButton(tr("紧急停止"), this);
    d_->emergency_stop_button->setStyleSheet("background-color: red; color: white;");

    // 添加到布局
    layout->addWidget(d_->set_initial_pose_button, 0, 0);
    layout->addWidget(d_->auto_localization_button, 0, 1);
    layout->addWidget(d_->set_goal_button, 1, 0);
    layout->addWidget(d_->cancel_goal_button, 1, 1);
    layout->addWidget(d_->start_navigation_button, 2, 0);
    layout->addWidget(d_->stop_navigation_button, 2, 1);
    layout->addWidget(d_->pause_navigation_button, 3, 0);
    layout->addWidget(d_->emergency_stop_button, 3, 1);

    d_->main_layout_->addWidget(d_->control_group_);
}

void NavigationPanel::createStatusGroup()
{
    d_->status_group_ = new QGroupBox(tr("导航状态"), this);
    auto* layout = new QGridLayout(d_->status_group_);

    // 状态显示
    d_->navigation_status_label = new QLabel(tr("就绪"), this);
    d_->distance_label = new QLabel(tr("距离目标点: 0.00 m"), this);
    d_->estimated_time_label = new QLabel(tr("预计时间: 0.00 s"), this);
    d_->linear_velocity_label = new QLabel(tr("线速度: 0.00 m/s"), this);
    d_->angular_velocity_label = new QLabel(tr("角速度: 0.00 rad/s"), this);
    d_->goal_status_label_ = new QLabel(tr("目标点状态: 未设置"), this);

    // 进度条
    d_->navigation_progress_bar = new QProgressBar(this);
    d_->navigation_progress_bar->setRange(0, 100);
    d_->navigation_progress_bar->setValue(0);

    // 添加到布局
    layout->addWidget(d_->navigation_status_label, 0, 0, 1, 2);
    layout->addWidget(d_->navigation_progress_bar, 1, 0, 1, 2);
    layout->addWidget(d_->distance_label, 2, 0);
    layout->addWidget(d_->estimated_time_label, 2, 1);
    layout->addWidget(d_->linear_velocity_label, 3, 0);
    layout->addWidget(d_->angular_velocity_label, 3, 1);
    layout->addWidget(d_->goal_status_label_, 4, 0, 1, 2);

    d_->main_layout_->addWidget(d_->status_group_);
}

void NavigationPanel::createPlannerGroup()
{
    d_->planner_group_ = new QGroupBox(tr("规划器设置"), this);
    auto* layout = new QHBoxLayout(d_->planner_group_);

    d_->planner_combo_ = new QComboBox(this);
    d_->planner_combo_->addItem(tr("DWA规划器"), "dwa");
    d_->planner_combo_->addItem(tr("TEB规划器"), "teb");
    d_->planner_combo_->addItem(tr("EBand规划器"), "eband");

    d_->planner_settings_button = new QPushButton(tr("高级设置"), this);

    layout->addWidget(new QLabel(tr("选择规划器:")));
    layout->addWidget(d_->planner_combo_);
    layout->addWidget(d_->planner_settings_button);
    layout->addStretch();

    d_->main_layout_->addWidget(d_->planner_group_);
}

void NavigationPanel::connectSignalsAndSlots()
{
    // 初始位姿设置
    connect(d_->set_initial_pose_button, &QPushButton::clicked, this, &NavigationPanel::onSetInitialPose);
    connect(d_->auto_localization_button, &QPushButton::clicked, this, &NavigationPanel::onAutoLocalizationButtonClicked);
    
    // 导航目标设置
    connect(d_->set_goal_button, &QPushButton::clicked, this, &NavigationPanel::onSetGoal);
    connect(d_->cancel_goal_button, &QPushButton::clicked, this, &NavigationPanel::onCancelGoal);
    
    // 导航控制
    connect(d_->start_navigation_button, &QPushButton::clicked, this, &NavigationPanel::onStartClicked);
    connect(d_->stop_navigation_button, &QPushButton::clicked, this, &NavigationPanel::onStopClicked);
    connect(d_->pause_navigation_button, &QPushButton::clicked, this, &NavigationPanel::onPauseClicked);
    connect(d_->emergency_stop_button, &QPushButton::clicked, this, &NavigationPanel::onEmergencyStop);
    
    // 规划器设置
    connect(d_->planner_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &NavigationPanel::onNavigationModeChanged);
    connect(d_->planner_settings_button, &QPushButton::clicked, this, &NavigationPanel::onPlannerSettings);

    // 添加自动定位相关的信号连接
    connect(this, &NavigationPanel::autoLocalizationStarted,
            [this]() {
        updateNavigationStatus(tr("正在进行自动定位纠正..."));
    });

    connect(this, &NavigationPanel::autoLocalizationFinished,
            [this]() {
        updateNavigationStatus(tr("自动定位纠正完成，继续导航"));
    });

    connect(d_->auto_localization_button_, &QPushButton::toggled,
            this, &NavigationPanel::onAutoLocalizationToggled);
}

void NavigationPanel::updateButtons()
{
    bool has_goal = d_->has_goal_;
    bool is_navigating = d_->is_navigating_;
    
    d_->set_initial_pose_button->setEnabled(!is_navigating);
    d_->auto_localization_button->setEnabled(!is_navigating);
    d_->set_goal_button->setEnabled(!is_navigating);
    d_->cancel_goal_button->setEnabled(has_goal);
    d_->start_navigation_button->setEnabled(has_goal && !is_navigating);
    d_->stop_navigation_button->setEnabled(is_navigating);
    d_->pause_navigation_button->setEnabled(is_navigating);
    d_->emergency_stop_button->setEnabled(true);  // 紧急停止始终可用
    d_->planner_combo_->setEnabled(!is_navigating);
    d_->planner_settings_button->setEnabled(!is_navigating);
}

void NavigationPanel::setRobotController(const std::shared_ptr<RobotController>& controller)
{
    QMutexLocker locker(&mutex_);
    if (!controller) {
        ROS_WARN("尝试设置空的RobotController指针");
        return;
    }
    d_->robot_controller = controller;
    ROS_INFO("RobotController设置成功");
}

void NavigationPanel::onSetInitialPose()
{
    if (!d_->robot_controller) return;
    
    d_->robot_controller->enableInitialPoseSetting(true);
    if (d_->rviz_view) {
        d_->rviz_view->activateInitialPoseTool();
    }
    d_->navigation_status_label->setText(tr("请在地图上设置初始位姿..."));
    updateButtons();
}

void NavigationPanel::onAutoLocalizationButtonClicked()
{
    if (!d_->robot_controller) {
        return;
    }

    // 发出自动定位开始信号
    Q_EMIT autoLocalizationStarted();

    // 暂停当前导航
    d_->robot_controller->pauseNavigation();

    // 启动自动定位
    d_->robot_controller->startAutoLocalization();

    // 连接定位完成信号
    connect(d_->robot_controller.get(), &RobotController::localizationStateChanged,
            this, [this](const QString& state) {
        if (state == "已完成") {
            // 断开信号连接
            disconnect(d_->robot_controller.get(), &RobotController::localizationStateChanged,
                      this, nullptr);
            
            // 恢复导航
            d_->robot_controller->resumeNavigation();

            // 发出自动定位完成信号
            Q_EMIT autoLocalizationFinished();
        }
    });
}

void NavigationPanel::onSetGoal()
{
    if (!d_->robot_controller) return;
    
    if (d_->rviz_view) {
        d_->rviz_view->activateGoalTool();
        d_->navigation_status_label->setText(tr("请在地图上设置导航目标点..."));
    } else {
        // 如果没有RViz视图，使用对话框设置目标点
        GoalSettingDialog dialog(d_->robot_controller->getCurrentPose(), this);
        if (dialog.exec() == QDialog::Accepted) {
            d_->current_goal_ = dialog.getGoal();
            d_->has_goal_ = true;
            updateGoalStatus(true);
        }
    }
    updateButtons();
}

void NavigationPanel::onCancelGoal()
{
    if (!d_->robot_controller) return;
    
    d_->robot_controller->stopNavigation();
    d_->has_goal_ = false;
    d_->current_goal_ = geometry_msgs::PoseStamped();
    updateGoalStatus(false);
    d_->navigation_status_label->setText(tr("导航已取消"));
    updateButtons();
}

void NavigationPanel::onStartClicked()
{
    if (!d_->robot_controller) return;
    
    if (d_->has_goal_) {
        d_->is_navigating_ = true;
        d_->robot_controller->setNavigationGoal(d_->current_goal_);
        d_->robot_controller->startNavigation();
        d_->navigation_status_label->setText(tr("正在导航..."));
        updateButtons();
    }
}

void NavigationPanel::onStopClicked()
{
    if (!d_->robot_controller) return;
    
    d_->robot_controller->stopNavigation();
    d_->is_navigating_ = false;
    d_->navigation_status_label->setText(tr("导航已停止"));
    updateButtons();
}

void NavigationPanel::onPauseClicked()
{
    if (!d_->robot_controller) return;
    
    if (d_->is_navigating_) {
        d_->robot_controller->pauseNavigation();
        d_->navigation_status_label->setText(tr("导航已暂停"));
        } else {
        d_->robot_controller->resumeNavigation();
        d_->navigation_status_label->setText(tr("导航已恢复"));
    }
    d_->is_navigating_ = !d_->is_navigating_;
    updateButtons();
}

void NavigationPanel::onEmergencyStop()
{
    if (!d_->robot_controller) return;
    
    d_->robot_controller->emergencyStop();
    d_->is_navigating_ = false;
    d_->navigation_status_label->setText(tr("紧急停止！"));
    updateButtons();
}

void NavigationPanel::onNavigationModeChanged(int index)
{
    if (d_->robot_controller) {
        QString planner = d_->planner_combo_->itemData(index).toString();
        d_->robot_controller->setPlanner(planner);
        d_->navigation_status_label->setText(tr("已切换到%1").arg(d_->planner_combo_->currentText()));
    }
}

void NavigationPanel::onPlannerSettings()
{
    if (!d_->robot_controller) return;
    
    PlannerSettingsDialog dialog(d_->robot_controller, this);
    dialog.exec();
}

void NavigationPanel::updateVelocityDisplay(double linear, double angular)
{
    d_->linear_velocity_label->setText(tr("线速度: %1 m/s").arg(linear, 0, 'f', 2));
    d_->angular_velocity_label->setText(tr("角速度: %1 rad/s").arg(angular, 0, 'f', 2));
}

void NavigationPanel::onNavigationStateChanged(const QString& state)
{
    d_->navigation_status_label->setText(state);
}

void NavigationPanel::onNavigationProgressChanged(double progress)
{
    d_->navigation_progress_bar->setValue(static_cast<int>(progress * 100));
}

void NavigationPanel::onDistanceToGoalChanged(double distance)
{
    d_->distance_label->setText(tr("距离目标点: %1 m").arg(distance, 0, 'f', 2));
}

void NavigationPanel::onEstimatedTimeToGoalChanged(double time)
{
    d_->estimated_time_label->setText(tr("预计时间: %1 s").arg(time, 0, 'f', 2));
}

void NavigationPanel::updateGoalStatus(bool has_goal)
{
    d_->goal_status_label_->setText(has_goal ? tr("目标点状态: 已设置") : tr("目标点状态: 未设置"));
}

void NavigationPanel::onGoalReached()
{
    d_->is_navigating_ = false;
    d_->has_goal_ = false;
    updateGoalStatus(false);
    updateButtons();
    QMessageBox::information(this, tr("导航完成"), tr("已到达目标点"));
}

void NavigationPanel::setRVizView(const std::shared_ptr<RVizView>& view)
{
    QMutexLocker locker(&d_->mutex_);
    d_->rviz_view = view;
}

void NavigationPanel::onGoalPoseSelected(const geometry_msgs::PoseStamped& pose)
{
    QMutexLocker locker(&d_->mutex_);
    if (d_->robot_controller) {
        d_->robot_controller->setNavigationGoal(pose);
    }
}

void NavigationPanel::onInitialPoseSelected(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
    QMutexLocker locker(&d_->mutex_);
    if (d_->robot_controller) {
        d_->robot_controller->setInitialPose(pose);
    }
}

void NavigationPanel::onLocalizationProgressChanged(double progress)
{
    updateLocalizationProgress(progress);
}

void NavigationPanel::onLocalizationStateChanged(const QString& state)
{
    updateLocalizationStatus(state);
}

void NavigationPanel::updateLocalizationStatus(const QString& status)
{
    d_->navigation_status_label->setText(status);
}

void NavigationPanel::handleKeyEvent(QKeyEvent* event, bool pressed)
{
    if (pressed) {
        keyPressEvent(event);
    } else {
        keyReleaseEvent(event);
    }
}

void NavigationPanel::keyPressEvent(QKeyEvent* event)
{
    if (!d_->robot_controller) return;

    d_->key_pressed_[event->key()] = true;
    onKeyboardVelocity();
    event->accept();
}

void NavigationPanel::keyReleaseEvent(QKeyEvent* event)
{
    if (!d_->robot_controller) return;

    d_->key_pressed_[event->key()] = false;
    onKeyboardVelocity();
    event->accept();
}

void NavigationPanel::onKeyboardVelocity()
{
    if (!d_->robot_controller) return;

    double linear = 0.0;
    double angular = 0.0;

    if (d_->key_pressed_[Qt::Key_W]) linear += 1.0;
    if (d_->key_pressed_[Qt::Key_S]) linear -= 1.0;
    if (d_->key_pressed_[Qt::Key_A]) angular += 1.0;
    if (d_->key_pressed_[Qt::Key_D]) angular -= 1.0;

    d_->robot_controller->setLinearVelocity(linear * d_->keyboard_linear_speed_);
    d_->robot_controller->setAngularVelocity(angular * d_->keyboard_angular_speed_);
}

void NavigationPanel::onJoystickMoved()
{
    if (!d_->robot_controller) return;

    double linear = -d_->joystick_linear_scale_;
    double angular = -d_->joystick_angular_scale_;

    d_->robot_controller->setLinearVelocity(linear);
    d_->robot_controller->setAngularVelocity(angular);
}

void NavigationPanel::updateLocalizationProgress(double progress)
{
    d_->navigation_progress_bar->setValue(static_cast<int>(progress * 100));
}

void NavigationPanel::updateNavigationStatus(const QString& status)
{
    d_->navigation_status_label->setText(status);
}

void NavigationPanel::onAutoLocalizationToggled(bool enabled)
{
    d_->auto_localization_enabled_ = enabled;
    d_->auto_localization_button_->setText(enabled ? tr("关闭自动定位") : tr("启用自动定位"));
    
    if (d_->robot_controller) {
        d_->robot_controller->enableAutoLocalization(enabled);
        d_->navigation_status_label->setText(enabled ? tr("自动定位已启用") : tr("自动定位已关闭"));
    }
} 