/**
 * @file navigation_panel.cpp
 * @brief 导航控制面板类的实现
 */

#include "ui/navigation_panel.h"
#include "ros/robot_controller.h"
#include "ui/rviz_view.h"

#include <QObject>
#include <QWidget>
#include <QFileDialog>
#include <QMessageBox>
#include <QTimer>
#include <QGroupBox>
#include <QKeyEvent>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QLineEdit>
#include <QCheckBox>
#include <QProgressBar>
#include <QDial>
#include <QTabWidget>
#include <QIcon>

NavigationPanel::NavigationPanel(const std::shared_ptr<RobotController>& robot_controller, QWidget* parent)
    : QWidget(parent), d_ptr(std::make_unique<NavigationPanelPrivate>())
{
    d_ptr->robot_controller = robot_controller;
    setupUi();
    setupJoystick();
    setupKeyboardControl();
    setFocusPolicy(Qt::StrongFocus);  // 使面板可以接收键盘事件

    // 连接机器人控制器的信号
    if (robot_controller) {
        connect(robot_controller.get(), &RobotController::localizationStateChanged,
                this, &NavigationPanel::onLocalizationStateChanged);
        connect(robot_controller.get(), &RobotController::localizationStatusChanged,
                this, &NavigationPanel::updateLocalizationStatus);
        connect(robot_controller.get(), &RobotController::localizationProgressChanged,
                this, &NavigationPanel::onLocalizationProgressChanged);
        connect(robot_controller.get(), &RobotController::velocityChanged,
                this, &NavigationPanel::updateVelocityDisplay);
        connect(robot_controller.get(), &RobotController::navigationStateChanged,
                this, &NavigationPanel::onNavigationStateChanged);
        connect(robot_controller.get(), &RobotController::navigationProgressChanged,
                this, &NavigationPanel::onNavigationProgressChanged);
        connect(robot_controller.get(), &RobotController::navigationModeChanged,
                this, &NavigationPanel::onNavigationModeChanged);
        connect(robot_controller.get(), &RobotController::distanceToGoalChanged,
                this, &NavigationPanel::onDistanceToGoalChanged);
        connect(robot_controller.get(), &RobotController::estimatedTimeToGoalChanged,
                this, &NavigationPanel::onEstimatedTimeToGoalChanged);
    }
}

NavigationPanel::~NavigationPanel() = default;

void NavigationPanel::setupUi()
{
    d_ptr->main_layout = new QVBoxLayout(this);
    d_ptr->main_layout->setContentsMargins(10, 10, 10, 10);
    d_ptr->main_layout->setSpacing(15);

    // 导航工具组
    auto* nav_tools_group = new QGroupBox(tr("导航工具"), this);
    nav_tools_group->setStyleSheet(
        "QGroupBox {"
        "    border: 1px solid #cccccc;"
        "    border-radius: 6px;"
        "    margin-top: 1em;"
        "    padding: 10px;"
        "    background-color: #ffffff;"
        "}"
        "QGroupBox::title {"
        "    subcontrol-origin: margin;"
        "    left: 10px;"
        "    padding: 0 3px 0 3px;"
        "    color: #333333;"
        "}"
    );
    
    auto* nav_tools_layout = new QHBoxLayout(nav_tools_group);
    nav_tools_layout->setSpacing(10);

    // 创建导航工具按钮
    d_ptr->set_goal_button = createStyledButton(tr("设置目标点"), ":/icons/target.png");
    d_ptr->set_pose_button = createStyledButton(tr("设置初始位姿"), ":/icons/pose.png");
    d_ptr->auto_localization_button = createStyledButton(tr("自动定位"), ":/icons/locate.png");
    
    nav_tools_layout->addWidget(d_ptr->set_goal_button);
    nav_tools_layout->addWidget(d_ptr->set_pose_button);
    nav_tools_layout->addWidget(d_ptr->auto_localization_button);
    
    d_ptr->main_layout->addWidget(nav_tools_group);

    // 目标点设置组
    auto* goal_group = new QGroupBox(tr("目标点设置"), this);
    goal_group->setStyleSheet(nav_tools_group->styleSheet());
    auto* goal_layout = new QGridLayout(goal_group);
    goal_layout->setSpacing(8);

    // X坐标
    goal_layout->addWidget(new QLabel(tr("X (米):"), this), 0, 0);
    auto* x_input = new QLineEdit(this);
    x_input->setStyleSheet(
        "QLineEdit {"
        "    border: 1px solid #cccccc;"
        "    border-radius: 4px;"
        "    padding: 5px;"
        "    background-color: #ffffff;"
        "}"
        "QLineEdit:focus {"
        "    border-color: #007AFF;"
        "}"
    );
    goal_layout->addWidget(x_input, 0, 1);

    // Y坐标
    goal_layout->addWidget(new QLabel(tr("Y (米):"), this), 1, 0);
    auto* y_input = new QLineEdit(this);
    y_input->setStyleSheet(x_input->styleSheet());
    goal_layout->addWidget(y_input, 1, 1);

    // 角度
    goal_layout->addWidget(new QLabel(tr("角度 (度):"), this), 2, 0);
    auto* angle_input = new QLineEdit(this);
    angle_input->setStyleSheet(x_input->styleSheet());
    goal_layout->addWidget(angle_input, 2, 1);

    d_ptr->main_layout->addWidget(goal_group);

    // 路径规划设置组
    auto* path_group = new QGroupBox(tr("路径规划设置"), this);
    path_group->setStyleSheet(nav_tools_group->styleSheet());
    auto* path_layout = new QGridLayout(path_group);
    path_layout->setSpacing(8);

    // 规划器选择
    path_layout->addWidget(new QLabel(tr("规划器:"), this), 0, 0);
    d_ptr->navigation_mode = new QComboBox(this);
    d_ptr->navigation_mode->addItem("Dijkstra");
    d_ptr->navigation_mode->addItem("A*");
    d_ptr->navigation_mode->addItem("RRT");
    d_ptr->navigation_mode->addItem("RRT*");
    d_ptr->navigation_mode->setStyleSheet(
        "QComboBox {"
        "    border: 1px solid #cccccc;"
        "    border-radius: 4px;"
        "    padding: 5px;"
        "    background-color: #ffffff;"
        "}"
        "QComboBox::drop-down {"
        "    border: none;"
        "}"
        "QComboBox::down-arrow {"
        "    image: url(:/icons/down_arrow.png);"
        "    width: 12px;"
        "    height: 12px;"
        "}"
    );
    path_layout->addWidget(d_ptr->navigation_mode, 0, 1);

    // 启发式函数选择
    path_layout->addWidget(new QLabel(tr("启发式函数:"), this), 1, 0);
    auto* heuristic = new QComboBox(this);
    heuristic->addItem(tr("欧几里得距离"));
    heuristic->addItem(tr("曼哈顿距离"));
    heuristic->setStyleSheet(d_ptr->navigation_mode->styleSheet());
    path_layout->addWidget(heuristic, 1, 1);

    // 规划参数
    path_layout->addWidget(new QLabel(tr("规划时间间隔(秒):"), this), 2, 0);
    auto* plan_time = new QLineEdit("5.0", this);
    plan_time->setStyleSheet(x_input->styleSheet());
    path_layout->addWidget(plan_time, 2, 1);

    path_layout->addWidget(new QLabel(tr("路径插值距离(米):"), this), 3, 0);
    auto* path_resolution = new QLineEdit("0.01", this);
    path_resolution->setStyleSheet(x_input->styleSheet());
    path_layout->addWidget(path_resolution, 3, 1);

    // 规划选项
    auto* allow_unknown = new QCheckBox(tr("允许未知区域"), this);
    path_layout->addWidget(allow_unknown, 4, 0);

    auto* optimize_path = new QCheckBox(tr("优化规划路径"), this);
    optimize_path->setChecked(true);
    path_layout->addWidget(optimize_path, 4, 1);

    d_ptr->main_layout->addWidget(path_group);

    // 导航控制组
    auto* control_group = new QGroupBox(tr("导航控制"), this);
    control_group->setStyleSheet(nav_tools_group->styleSheet());
    auto* control_layout = new QVBoxLayout(control_group);

    // 导航状态
    auto* status_layout = new QHBoxLayout();
    status_layout->addWidget(new QLabel(tr("当前状态:")));
    d_ptr->navigation_status_label = new QLabel(tr("等待目标点"));
    status_layout->addWidget(d_ptr->navigation_status_label);
    control_layout->addLayout(status_layout);

    // 导航进度
    d_ptr->navigation_progress = new QProgressBar(this);
    d_ptr->navigation_progress->setStyleSheet(
        "QProgressBar {"
        "    border: 1px solid #cccccc;"
        "    border-radius: 4px;"
        "    text-align: center;"
        "    background-color: #f5f5f5;"
        "}"
        "QProgressBar::chunk {"
        "    background-color: #007AFF;"
        "    border-radius: 3px;"
        "}"
    );
    control_layout->addWidget(d_ptr->navigation_progress);

    // 导航信息
    auto* info_layout = new QGridLayout();
    info_layout->addWidget(new QLabel(tr("距离目标点:")), 0, 0);
    d_ptr->distance_label = new QLabel("-- 米");
    info_layout->addWidget(d_ptr->distance_label, 0, 1);

    info_layout->addWidget(new QLabel(tr("预计时间:")), 1, 0);
    d_ptr->time_label = new QLabel("-- 秒");
    info_layout->addWidget(d_ptr->time_label, 1, 1);

    control_layout->addLayout(info_layout);

    // 导航控制按钮
    auto* nav_buttons_layout = new QHBoxLayout();
    d_ptr->start_nav_button = createStyledButton(tr("开始导航"), ":/icons/start.png");
    d_ptr->pause_nav_button = createStyledButton(tr("暂停导航"), ":/icons/pause.png");
    d_ptr->stop_nav_button = createStyledButton(tr("停止导航"), ":/icons/stop.png");

    nav_buttons_layout->addWidget(d_ptr->start_nav_button);
    nav_buttons_layout->addWidget(d_ptr->pause_nav_button);
    nav_buttons_layout->addWidget(d_ptr->stop_nav_button);

    control_layout->addLayout(nav_buttons_layout);
    d_ptr->main_layout->addWidget(control_group);

    // 键盘控制提示
    auto* keyboard_group = new QGroupBox(tr("键盘控制"), this);
    keyboard_group->setStyleSheet(nav_tools_group->styleSheet());
    auto* keyboard_layout = new QVBoxLayout(keyboard_group);

    auto* keyboard_label = new QLabel(
        tr("↑/W: 前进  ↓/S: 后退\n"
           "←/A: 左转  →/D: 右转\n"
           "空格: 紧急停止"), this);
    keyboard_label->setAlignment(Qt::AlignCenter);
    keyboard_layout->addWidget(keyboard_label);

    d_ptr->main_layout->addWidget(keyboard_group);

    // 添加伸缩器
    d_ptr->main_layout->addStretch();

    // 连接信号和槽
    connectSignalsAndSlots();
}

QPushButton* NavigationPanel::createStyledButton(const QString& text, const QString& icon_path)
{
    auto* button = new QPushButton(text, this);
    button->setStyleSheet(
        "QPushButton {"
        "    border: 1px solid #cccccc;"
        "    border-radius: 4px;"
        "    padding: 8px 16px;"
        "    background-color: #ffffff;"
        "    color: #333333;"
        "}"
        "QPushButton:hover {"
        "    background-color: #f5f5f5;"
        "}"
        "QPushButton:pressed {"
        "    background-color: #e5e5e5;"
        "}"
        "QPushButton:disabled {"
        "    background-color: #f5f5f5;"
        "    color: #999999;"
        "}"
    );
    
    if (!icon_path.isEmpty()) {
        button->setIcon(QIcon(icon_path));
    }
    
    return button;
}

void NavigationPanel::setRVizView(const std::shared_ptr<RVizView>& rviz_view)
{
    d_ptr->rviz_view = rviz_view;
    if (d_ptr->rviz_view && d_ptr->rviz_placeholder) {
        // 将RViz视图添加到占位符的布局中
        auto* layout = new QVBoxLayout(d_ptr->rviz_placeholder);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->addWidget(d_ptr->rviz_view.get());
        
        // 设置固定帧和目标帧
        d_ptr->rviz_view->setFixedFrame("map");
        d_ptr->rviz_view->setTargetFrame("base_footprint");
        
        // 启用必要的显示
        d_ptr->rviz_view->setDisplayEnabled("Grid", true);
        d_ptr->rviz_view->setDisplayEnabled("Map", true);
        d_ptr->rviz_view->setDisplayEnabled("LaserScan", true);
        d_ptr->rviz_view->setDisplayEnabled("RobotModel", true);
        d_ptr->rviz_view->setDisplayEnabled("Path", true);
    }
}

void NavigationPanel::onSetInitialPose()
{
    if (d_ptr->rviz_view) {
        d_ptr->rviz_view->onInitialPoseToolActivated();
    }
}

void NavigationPanel::onAutoLocalization()
{
    if (!d_ptr->is_localizing) {
        startAutoLocalization();
    } else {
        d_ptr->is_localizing = false;
        if (d_ptr->localization_timer) {
            d_ptr->localization_timer->stop();
        }
        updateLocalizationStatus(tr("自动定位已取消"));
        d_ptr->auto_localization_button->setText(tr("自动定位"));
        d_ptr->localization_progress->setVisible(false);
    }
}

void NavigationPanel::startAutoLocalization()
{
    updateLocalizationStatus(tr("开始自动定位..."));
    d_ptr->localization_progress->setValue(0);
    d_ptr->localization_progress->setVisible(true);
    d_ptr->auto_localization_button->setText(tr("取消定位"));
    d_ptr->is_localizing = true;

    if (!d_ptr->localization_timer) {
        d_ptr->localization_timer = new QTimer(this);
        d_ptr->localization_timer->setInterval(100);
    }

    auto updateProgress = [this](int step) {
        double progress = step * 25.0;
        d_ptr->localization_progress->setValue(static_cast<int>(progress));
        emit localizationProgressChanged(progress / 100.0);
    };

    std::function<void()> nextStep = [this, updateProgress, &nextStep]() {
        static int current_step = 0;

        switch (current_step) {
            case 0:
                updateLocalizationStatus(tr("步骤1/4: 原地旋转扫描环境..."));
                updateProgress(1);
                QTimer::singleShot(3000, this, [&nextStep]() { nextStep(); });
                break;

            case 1:
                updateLocalizationStatus(tr("步骤2/4: 前进采集数据..."));
                updateProgress(2);
                QTimer::singleShot(3000, this, [&nextStep]() { nextStep(); });
                break;

            case 2:
                updateLocalizationStatus(tr("步骤3/4: 旋转扫描..."));
                updateProgress(3);
                QTimer::singleShot(3000, this, [&nextStep]() { nextStep(); });
                break;

            case 3:
                updateLocalizationStatus(tr("步骤4/4: 优化定位结果..."));
                updateProgress(4);
                QTimer::singleShot(3000, this, [this]() {
                    updateLocalizationStatus(tr("自动定位完成"));
                    d_ptr->localization_progress->setValue(100);
                    d_ptr->auto_localization_button->setText(tr("自动定位"));
                    d_ptr->is_localizing = false;
                    emit localizationStateChanged(true);
                });
                break;
        }

        current_step = (current_step + 1) % 4;
    };

    nextStep();
}

void NavigationPanel::onSetGoal()
{
    if (d_ptr->rviz_view) {
        d_ptr->rviz_view->onGoalToolActivated();
    }
}

void NavigationPanel::onCancelGoal()
{
    if (d_ptr->robot_controller) {
        d_ptr->robot_controller->cancelNavigation();
    }
}

void NavigationPanel::onStartMapping()
{
    if (d_ptr->robot_controller) {
        d_ptr->robot_controller->startMapping();
    }
}

void NavigationPanel::onStopMapping()
{
    if (d_ptr->robot_controller) {
        d_ptr->robot_controller->stopMapping();
    }
}

void NavigationPanel::onSaveMap()
{
    QString filename = QFileDialog::getSaveFileName(
        this,
        tr("保存地图"),
        QString(),
        tr("地图文件 (*.yaml);;所有文件 (*)")
    );

    if (!filename.isEmpty()) {
        if (!filename.endsWith(".yaml", Qt::CaseInsensitive)) {
            filename += ".yaml";
        }
        d_ptr->robot_controller->saveMap(filename);
    }
}

void NavigationPanel::onLoadMap()
{
    QString filename = QFileDialog::getOpenFileName(
        this,
        tr("加载地图"),
        QString(),
        tr("地图文件 (*.yaml);;所有文件 (*)")
    );

    if (!filename.isEmpty()) {
        d_ptr->robot_controller->loadMap(filename);
    }
}

void NavigationPanel::onNavigationModeChanged(int index)
{
    if (d_ptr->robot_controller) {
        d_ptr->robot_controller->setNavigationMode(index);
        emit navigationModeChanged(index);
    }
}

void NavigationPanel::updateLocalizationState(bool is_localized)
{
    if (is_localized) {
        d_ptr->is_localizing = false;
        updateButtonStates();
    }
    emit localizationStateChanged(is_localized);
}

void NavigationPanel::updateLocalizationStatus(const QString& status)
{
    if (d_ptr->localization_status) {
        d_ptr->localization_status->setText(status);
    }
    emit localizationStatusChanged(status);
}

void NavigationPanel::updateLocalizationProgress(double progress)
{
    if (d_ptr->localization_progress) {
        d_ptr->localization_progress->setValue(static_cast<int>(progress * 100));
    }
    emit localizationProgressChanged(progress);
}

void NavigationPanel::updateButtonStates()
{
    if (d_ptr->auto_localization_button) {
        d_ptr->auto_localization_button->setEnabled(!d_ptr->is_localizing);
    }
}

void NavigationPanel::setupJoystick()
{
    // 设置转盘的范围和初始值
    if (d_ptr->linear_velocity_dial) {
        d_ptr->linear_velocity_dial->setRange(-100, 100);
        d_ptr->linear_velocity_dial->setValue(0);
        d_ptr->linear_velocity_dial->setNotchesVisible(true);
    }
    
    if (d_ptr->angular_velocity_dial) {
        d_ptr->angular_velocity_dial->setRange(-100, 100);
        d_ptr->angular_velocity_dial->setValue(0);
        d_ptr->angular_velocity_dial->setNotchesVisible(true);
    }
}

void NavigationPanel::setupKeyboardControl()
{
    // 重置键盘状态
    d_ptr->key_up_pressed = false;
    d_ptr->key_down_pressed = false;
    d_ptr->key_left_pressed = false;
    d_ptr->key_right_pressed = false;
}

void NavigationPanel::keyPressEvent(QKeyEvent* event)
{
    if (!d_ptr->robot_controller) return;

    bool handled = true;
    switch (event->key()) {
        case Qt::Key_Up:
        case Qt::Key_W:
            d_ptr->key_up_pressed = true;
            d_ptr->current_linear_velocity = 0.2;  // 向前
            break;
        case Qt::Key_Down:
        case Qt::Key_S:
            d_ptr->key_down_pressed = true;
            d_ptr->current_linear_velocity = -0.2;  // 向后
            break;
        case Qt::Key_Left:
        case Qt::Key_A:
            d_ptr->key_left_pressed = true;
            d_ptr->current_angular_velocity = 0.5;  // 左转
            break;
        case Qt::Key_Right:
        case Qt::Key_D:
            d_ptr->key_right_pressed = true;
            d_ptr->current_angular_velocity = -0.5;  // 右转
            break;
        case Qt::Key_Space:
            onEmergencyStop();
            break;
        default:
            handled = false;
            break;
    }

    if (handled) {
        d_ptr->robot_controller->publishVelocity(
            d_ptr->current_linear_velocity,
            d_ptr->current_angular_velocity
        );
        event->accept();
    } else {
        QWidget::keyPressEvent(event);
    }
}

void NavigationPanel::keyReleaseEvent(QKeyEvent* event)
{
    if (!d_ptr->robot_controller) return;

    bool handled = true;
    switch (event->key()) {
        case Qt::Key_Up:
        case Qt::Key_W:
            d_ptr->key_up_pressed = false;
            if (!d_ptr->key_down_pressed) d_ptr->current_linear_velocity = 0.0;
            break;
        case Qt::Key_Down:
        case Qt::Key_S:
            d_ptr->key_down_pressed = false;
            if (!d_ptr->key_up_pressed) d_ptr->current_linear_velocity = 0.0;
            break;
        case Qt::Key_Left:
        case Qt::Key_A:
            d_ptr->key_left_pressed = false;
            if (!d_ptr->key_right_pressed) d_ptr->current_angular_velocity = 0.0;
            break;
        case Qt::Key_Right:
        case Qt::Key_D:
            d_ptr->key_right_pressed = false;
            if (!d_ptr->key_left_pressed) d_ptr->current_angular_velocity = 0.0;
            break;
        default:
            handled = false;
            break;
    }

    if (handled) {
        d_ptr->robot_controller->publishVelocity(
            d_ptr->current_linear_velocity,
            d_ptr->current_angular_velocity
        );
        event->accept();
    } else {
        QWidget::keyReleaseEvent(event);
    }
}

void NavigationPanel::onLinearVelocityChanged(int value)
{
    if (!d_ptr->robot_controller) return;
    
    // 将-100到100的范围映射到实际速度范围
    double velocity = (value / 100.0) * d_ptr->robot_controller->getMaxLinearVelocity();
    d_ptr->current_linear_velocity = velocity;
    d_ptr->robot_controller->setLinearVelocity(velocity);
}

void NavigationPanel::onAngularVelocityChanged(int value)
{
    if (!d_ptr->robot_controller) return;
    
    // 将-100到100的范围映射到实际速度范围
    double velocity = (value / 100.0) * d_ptr->robot_controller->getMaxAngularVelocity();
    d_ptr->current_angular_velocity = velocity;
    d_ptr->robot_controller->setAngularVelocity(velocity);
}

void NavigationPanel::onEmergencyStop()
{
    if (!d_ptr->robot_controller) return;
    
    // 停止所有运动
    d_ptr->current_linear_velocity = 0.0;
    d_ptr->current_angular_velocity = 0.0;
    d_ptr->robot_controller->stop();
    
    // 重置控制器状态
    d_ptr->linear_velocity_dial->setValue(0);
    d_ptr->angular_velocity_dial->setValue(0);
    
    // 重置键盘状态
    d_ptr->key_up_pressed = false;
    d_ptr->key_down_pressed = false;
    d_ptr->key_left_pressed = false;
    d_ptr->key_right_pressed = false;
}

void NavigationPanel::updateVelocityDisplay(double linear, double angular)
{
    if (d_ptr->linear_velocity_label) {
        QString linear_text = QString("线速度: %1 m/s").arg(linear, 0, 'f', 2);
        d_ptr->linear_velocity_label->setText(linear_text);
    }
    if (d_ptr->angular_velocity_label) {
        QString angular_text = QString("角速度: %1 rad/s").arg(angular, 0, 'f', 2);
        d_ptr->angular_velocity_label->setText(angular_text);
    }
}

void NavigationPanel::onJoystickMoved()
{
    if (!d_ptr->robot_controller) return;
    
    // 获取当前转盘的值
    int linear_value = d_ptr->linear_velocity_dial->value();
    int angular_value = d_ptr->angular_velocity_dial->value();
    
    // 将值映射到实际速度
    double linear_velocity = (linear_value / 100.0) * d_ptr->robot_controller->getMaxLinearVelocity();
    double angular_velocity = (angular_value / 100.0) * d_ptr->robot_controller->getMaxAngularVelocity();
    
    // 更新当前速度
    d_ptr->current_linear_velocity = linear_velocity;
    d_ptr->current_angular_velocity = angular_velocity;
    
    // 发布速度命令
    d_ptr->robot_controller->publishVelocity(linear_velocity, angular_velocity);
}

void NavigationPanel::onStartNavigation()
{
    if (d_ptr->robot_controller) {
        d_ptr->robot_controller->startNavigation();
        d_ptr->navigation_status_label->setText(tr("正在导航..."));
        d_ptr->start_nav_button->setEnabled(false);
        d_ptr->pause_nav_button->setEnabled(true);
        d_ptr->stop_nav_button->setEnabled(true);
    }
}

void NavigationPanel::onPauseNavigation()
{
    if (d_ptr->robot_controller) {
        d_ptr->robot_controller->pauseNavigation();
        d_ptr->navigation_status_label->setText(tr("导航已暂停"));
        d_ptr->start_nav_button->setEnabled(true);
        d_ptr->pause_nav_button->setEnabled(false);
    }
}

void NavigationPanel::onStopNavigation()
{
    if (d_ptr->robot_controller) {
        d_ptr->robot_controller->stopNavigation();
        d_ptr->navigation_status_label->setText(tr("导航已停止"));
        d_ptr->navigation_progress->setValue(0);
        d_ptr->distance_label->setText("-- 米");
        d_ptr->time_label->setText("-- 秒");
        d_ptr->start_nav_button->setEnabled(true);
        d_ptr->pause_nav_button->setEnabled(false);
        d_ptr->stop_nav_button->setEnabled(false);
    }
}

void NavigationPanel::onLocalizationStateChanged(bool is_localized)
{
    updateLocalizationState(is_localized);
}

void NavigationPanel::onLocalizationProgressChanged(double progress)
{
    updateLocalizationProgress(progress);
}

void NavigationPanel::onNavigationStateChanged(bool is_navigating)
{
    d_ptr->set_goal_button->setEnabled(!is_navigating);
    d_ptr->cancel_goal_button->setEnabled(is_navigating);
    if (is_navigating) {
        d_ptr->navigation_status->setText(tr("导航中..."));
    } else {
        d_ptr->navigation_status->setText(tr("就绪"));
    }
}

void NavigationPanel::onNavigationProgressChanged(double progress)
{
    // TODO: 更新导航进度显示
}

void NavigationPanel::onDistanceToGoalChanged(double distance)
{
    // TODO: 更新到目标点的距离显示
}

void NavigationPanel::onEstimatedTimeToGoalChanged(double time)
{
    // TODO: 更新预计到达时间显示
}

void NavigationPanel::connectSignalsAndSlots()
{
    // 连接导航工具按钮的信号
    connect(d_ptr->set_goal_button, &QPushButton::clicked, this, &NavigationPanel::onSetGoal);
    connect(d_ptr->set_pose_button, &QPushButton::clicked, this, &NavigationPanel::onSetInitialPose);
    connect(d_ptr->auto_localization_button, &QPushButton::clicked, this, &NavigationPanel::onAutoLocalization);

    // 连接导航控制按钮的信号
    connect(d_ptr->start_nav_button, &QPushButton::clicked, this, &NavigationPanel::onStartNavigation);
    connect(d_ptr->pause_nav_button, &QPushButton::clicked, this, &NavigationPanel::onPauseNavigation);
    connect(d_ptr->stop_nav_button, &QPushButton::clicked, this, &NavigationPanel::onStopNavigation);

    // 连接导航模式选择的信号
    connect(d_ptr->navigation_mode, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &NavigationPanel::onNavigationModeChanged);

    // 连接速度控制的信号
    if (d_ptr->linear_velocity_dial) {
        connect(d_ptr->linear_velocity_dial, &QDial::valueChanged,
                this, &NavigationPanel::onLinearVelocityChanged);
    }
    if (d_ptr->angular_velocity_dial) {
        connect(d_ptr->angular_velocity_dial, &QDial::valueChanged,
                this, &NavigationPanel::onAngularVelocityChanged);
    }
} 