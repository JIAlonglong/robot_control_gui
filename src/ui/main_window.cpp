/**
 * @file main_window.cpp
 * @brief 主窗口类的实现
 */

#include "ui/main_window.h"
#include "ui/rviz_view.h"
#include "ui/navigation_panel.h"
#include "ui/joystick_widget.h"
#include "ui/speed_dashboard.h"
#include "ros/robot_controller.h"

#include <QMainWindow>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QGroupBox>
#include <QMessageBox>
#include <QKeyEvent>
#include <QToolBar>
#include <QDockWidget>
#include <QStatusBar>
#include <QDebug>
#include <QTimer>
#include <QCheckBox>
#include <QAction>
#include <QSizePolicy>
#include <QSplitter>
#include <QMenuBar>
#include <QToolButton>
#include <QSpacerItem>
#include <QIcon>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), d_(std::make_unique<MainWindowPrivate>())
{
    // 初始化ROS节点
    if (!ros::isInitialized()) {
        throw std::runtime_error("ROS节点未初始化");
    }

    // 创建机器人控制器
    d_->robot_controller_ = std::make_shared<RobotController>();
    if (!d_->robot_controller_) {
        throw std::runtime_error("无法创建机器人控制器");
    }

    // 设置窗口标题和大小
    setWindowTitle(tr("TurtleBot3 控制面板"));
    setMinimumSize(1200, 800);

    // 设置UI
    setupUi();
    setupSubscribers();
    setupRosConnections();
}

MainWindow::~MainWindow()
{
    if (d_->robot_controller_) {
        d_->robot_controller_->stop();
    }
}

void MainWindow::setupUi()
{
    // 设置窗口属性
    setWindowTitle(tr("机器人控制界面"));
    setMinimumSize(1024, 768);
    
    // 创建中央部件
    d_->central_widget_ = new QWidget(this);
    setCentralWidget(d_->central_widget_);
    
    // 创建主布局
    auto* main_layout = new QHBoxLayout(d_->central_widget_);
    main_layout->setContentsMargins(0, 0, 0, 0);
    main_layout->setSpacing(0);

    // 创建RViz视图
    d_->rviz_view_ = std::make_shared<RVizView>(d_->central_widget_);
    
    // 创建堆叠部件
    d_->stacked_widget_ = new QStackedWidget(d_->central_widget_);
    
    // 创建控制面板
    d_->robot_controller_ = std::make_shared<RobotController>();
    d_->control_panel_ = std::make_shared<ControlPanel>(d_->robot_controller_, d_->stacked_widget_);
    d_->stacked_widget_->addWidget(d_->control_panel_.get());
    
    // 创建导航面板
    d_->navigation_panel_ = std::make_shared<NavigationPanel>(d_->robot_controller_, d_->stacked_widget_);
    d_->stacked_widget_->addWidget(d_->navigation_panel_.get());
    
    // 设置RVizView
    d_->navigation_panel_->setRVizView(d_->rviz_view_);
    
    // 创建设置面板
    d_->settings_panel_ = std::make_shared<SettingsPanel>(d_->robot_controller_, d_->stacked_widget_);
    d_->stacked_widget_->addWidget(d_->settings_panel_.get());
    
    // 设置布局
    main_layout->addWidget(d_->rviz_view_.get(), 2);  // RViz视图占2/3
    main_layout->addWidget(d_->stacked_widget_, 1);   // 控制面板占1/3
    
    // 创建工具栏
    createToolBar();
    
    // 创建显示选项面板
    createDisplayOptionsPanel();
    
    // 设置初始页面
    d_->stacked_widget_->setCurrentWidget(d_->control_panel_.get());
}

void MainWindow::createToolBar()
{
    d_->tool_bar_ = addToolBar(tr("工具栏"));
    d_->tool_bar_->setMovable(false);
    d_->tool_bar_->setFloatable(false);
    
    // 设置工具栏样式
    d_->tool_bar_->setStyleSheet(
        "QToolBar {"
        "    background: #f8f9fa;"
        "    border-bottom: 1px solid #dee2e6;"
        "    spacing: 5px;"
        "}"
        "QToolButton {"
        "    background: transparent;"
        "    border: none;"
        "    padding: 8px;"
        "    border-radius: 4px;"
        "}"
        "QToolButton:hover {"
        "    background: #e9ecef;"
        "}"
        "QToolButton:pressed {"
        "    background: #dee2e6;"
        "}"
    );

    // 创建控制面板按钮
    auto* control_action = new QAction(QIcon::fromTheme("video-display"), tr("控制面板"), this);
    control_action->setCheckable(true);
    control_action->setChecked(true);
    d_->tool_bar_->addAction(control_action);

    // 创建导航面板按钮
    auto* navigation_action = new QAction(QIcon::fromTheme("map"), tr("导航面板"), this);
    navigation_action->setCheckable(true);
    d_->tool_bar_->addAction(navigation_action);

    // 创建设置按钮
    auto* settings_action = new QAction(QIcon::fromTheme("preferences-system"), tr("设置"), this);
    settings_action->setCheckable(true);
    d_->tool_bar_->addAction(settings_action);

    // 添加伸缩器
    auto* spacer = new QWidget();
    spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    d_->tool_bar_->addWidget(spacer);

    // 添加状态标签
    auto* status_label = new QLabel(tr("就绪"), this);
    d_->tool_bar_->addWidget(status_label);

    // 连接信号
    connect(control_action, &QAction::triggered, this, [this, control_action, navigation_action, settings_action]() {
        if (control_action->isChecked()) {
            navigation_action->setChecked(false);
            settings_action->setChecked(false);
            d_->stacked_widget_->setCurrentWidget(d_->control_panel_.get());
        } else {
            control_action->setChecked(true);
        }
    });

    connect(navigation_action, &QAction::triggered, this, [this, control_action, navigation_action, settings_action]() {
        if (navigation_action->isChecked()) {
            control_action->setChecked(false);
            settings_action->setChecked(false);
            d_->stacked_widget_->setCurrentWidget(d_->navigation_panel_.get());
        } else {
            navigation_action->setChecked(true);
        }
    });

    connect(settings_action, &QAction::triggered, this, [this, control_action, navigation_action, settings_action]() {
        if (settings_action->isChecked()) {
            control_action->setChecked(false);
            navigation_action->setChecked(false);
            d_->stacked_widget_->setCurrentWidget(d_->settings_panel_.get());
        } else {
            settings_action->setChecked(true);
        }
    });
}

void MainWindow::createDisplayOptionsPanel()
{
    // 创建显示选项面板
    auto* dock = new QDockWidget(tr("显示选项"), this);
    dock->setFeatures(QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable);
    dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

    auto* content = new QWidget(dock);
    auto* layout = new QVBoxLayout(content);
    layout->setContentsMargins(10, 10, 10, 10);
    layout->setSpacing(10);

    // 创建显示选项复选框
    auto createCheckBox = [](const QString& text, QWidget* parent) {
        auto* box = new QCheckBox(text, parent);
        box->setStyleSheet(
            "QCheckBox {"
            "    spacing: 5px;"
            "}"
            "QCheckBox::indicator {"
            "    width: 18px;"
            "    height: 18px;"
            "    border: 1px solid #cccccc;"
            "    border-radius: 3px;"
            "}"
            "QCheckBox::indicator:checked {"
            "    background-color: #007AFF;"
            "    border-color: #007AFF;"
            "    image: url(:/icons/check.png);"
            "}"
            "QCheckBox::indicator:unchecked:hover {"
            "    border-color: #007AFF;"
            "}"
        );
        return box;
    };

    auto* grid_box = createCheckBox(tr("显示网格"), content);
    auto* map_box = createCheckBox(tr("显示地图"), content);
    auto* robot_box = createCheckBox(tr("显示机器人"), content);
    auto* laser_box = createCheckBox(tr("显示激光"), content);
    auto* path_box = createCheckBox(tr("显示路径"), content);
    auto* goal_box = createCheckBox(tr("显示目标"), content);

    // 设置默认状态
    grid_box->setChecked(true);
    map_box->setChecked(true);
    robot_box->setChecked(true);
    laser_box->setChecked(true);
    path_box->setChecked(true);
    goal_box->setChecked(true);

    // 添加到布局
    layout->addWidget(grid_box);
    layout->addWidget(map_box);
    layout->addWidget(robot_box);
    layout->addWidget(laser_box);
    layout->addWidget(path_box);
    layout->addWidget(goal_box);
    layout->addStretch();

    // 设置对象名称
    grid_box->setObjectName("grid_box");
    map_box->setObjectName("map_box");
    robot_box->setObjectName("robot_box");
    laser_box->setObjectName("laser_box");
    path_box->setObjectName("path_box");
    goal_box->setObjectName("goal_box");

    // 连接信号
    connect(grid_box, &QCheckBox::toggled, this, &MainWindow::onDisplayOptionsChanged);
    connect(map_box, &QCheckBox::toggled, this, &MainWindow::onDisplayOptionsChanged);
    connect(robot_box, &QCheckBox::toggled, this, &MainWindow::onDisplayOptionsChanged);
    connect(laser_box, &QCheckBox::toggled, this, &MainWindow::onDisplayOptionsChanged);
    connect(path_box, &QCheckBox::toggled, this, &MainWindow::onDisplayOptionsChanged);
    connect(goal_box, &QCheckBox::toggled, this, &MainWindow::onDisplayOptionsChanged);

    dock->setWidget(content);
    addDockWidget(Qt::RightDockWidgetArea, dock);
    d_->display_options_dock_ = dock;
}

void MainWindow::connectSignals()
{
    if (d_->rviz_view_) {
        connect(d_->rviz_view_.get(), &RVizView::goalSelected,
                this, &MainWindow::onGoalSelected);
        connect(d_->rviz_view_.get(), &RVizView::initialPoseSelected,
                this, &MainWindow::onInitialPoseSelected);
    }

    if (d_->navigation_panel_) {
        // 连接导航面板的信号
    }
}

void MainWindow::onLinearJoystickMoved(double x, double y)
{
    if (!d_->robot_controller_) return;

    double linear_vel = -y * d_->max_linear_vel_;
    d_->robot_controller_->setLinearVelocity(linear_vel);
    d_->current_linear_vel_ = linear_vel;

    if (d_->speed_dashboard_) {
        d_->speed_dashboard_->setLinearSpeed(linear_vel);
    }
}

void MainWindow::onAngularJoystickMoved(double x, double y)
{
    if (!d_->robot_controller_) return;

    double angular_vel = -x * d_->max_angular_vel_;
    d_->robot_controller_->setAngularVelocity(angular_vel);
    d_->current_angular_vel_ = angular_vel;

    if (d_->speed_dashboard_) {
        d_->speed_dashboard_->setAngularSpeed(angular_vel);
    }
}

void MainWindow::onGoalSelected(const geometry_msgs::PoseStamped& goal)
{
    if (d_->robot_controller_) {
        d_->robot_controller_->setNavigationGoal(goal);
    }
}

void MainWindow::onInitialPoseSelected(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
    if (d_->robot_controller_) {
        d_->robot_controller_->setInitialPose(pose);
    }
}

void MainWindow::createFloatingControlPanel()
{
    QDockWidget* dock = new QDockWidget(tr("速度控制"), this);
    dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

    QWidget* control_widget = new QWidget(dock);
    QVBoxLayout* layout = new QVBoxLayout(control_widget);

    d_->speed_dashboard_ = std::make_shared<SpeedDashboard>(control_widget);
    layout->addWidget(d_->speed_dashboard_.get());

    dock->setWidget(control_widget);
    addDockWidget(Qt::RightDockWidgetArea, dock);
}

void MainWindow::setupSubscribers()
{
    d_->goal_pub_ = d_->nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    d_->path_sub_ = d_->nh_.subscribe("/move_base/NavfnROS/plan", 1, &MainWindow::pathCallback, this);
    d_->map_sub_ = d_->nh_.subscribe("/map", 1, &MainWindow::mapCallback, this);
    d_->scan_sub_ = d_->nh_.subscribe("/scan", 1, &MainWindow::scanCallback, this);
    d_->odom_sub_ = d_->nh_.subscribe("/odom", 1, &MainWindow::odomCallback, this);
}

void MainWindow::handleMapUpdate(const nav_msgs::OccupancyGridConstPtr& msg)
{
    if (d_->rviz_view_) {
        // 确保地图显示已启用
        d_->rviz_view_->setDisplayEnabled("Map", true);
        // 更新地图数据
        d_->rviz_view_->updateMap(msg);
    }
}

void MainWindow::handleOdomUpdate(const nav_msgs::OdometryConstPtr& msg)
{
    if (d_->rviz_view_) {
        d_->rviz_view_->updateRobotPose(msg);
    }
}

void MainWindow::handleScanUpdate(const sensor_msgs::LaserScanConstPtr& msg)
{
    if (d_->rviz_view_) {
        d_->rviz_view_->updateLaserScan(msg);
    }
}

void MainWindow::keyPressEvent(QKeyEvent* event)
{
    if (!event->isAutoRepeat()) {
        d_->key_states_[event->key()] = true;
        updateKeyboardControl();
    }
    QMainWindow::keyPressEvent(event);
}

void MainWindow::keyReleaseEvent(QKeyEvent* event)
{
    if (!event->isAutoRepeat()) {
        d_->key_states_[event->key()] = false;
        updateKeyboardControl();

        if (event->key() == Qt::Key_Space && d_->robot_controller_) {
            d_->robot_controller_->stop();
            d_->current_linear_vel_ = 0.0;
            d_->current_angular_vel_ = 0.0;
            if (d_->speed_dashboard_) {
                d_->speed_dashboard_->setLinearSpeed(0.0);
                d_->speed_dashboard_->setAngularSpeed(0.0);
            }
        }
    }
    QMainWindow::keyReleaseEvent(event);
}

void MainWindow::updateKeyboardControl()
{
    if (!d_->robot_controller_) return;

    // 获取按键状态
    bool up = d_->key_states_.value(Qt::Key_W) || d_->key_states_.value(Qt::Key_Up);
    bool down = d_->key_states_.value(Qt::Key_S) || d_->key_states_.value(Qt::Key_Down);
    bool left = d_->key_states_.value(Qt::Key_A) || d_->key_states_.value(Qt::Key_Left);
    bool right = d_->key_states_.value(Qt::Key_D) || d_->key_states_.value(Qt::Key_Right);

    // 计算线速度和角速度
    double linear_vel = 0.0;
    double angular_vel = 0.0;

    if (up && !down) {
        linear_vel = d_->max_linear_vel_;
    } else if (down && !up) {
        linear_vel = -d_->max_linear_vel_;
    }

    if (left && !right) {
        angular_vel = d_->max_angular_vel_;
    } else if (right && !left) {
        angular_vel = -d_->max_angular_vel_;
    }

    // 如果速度发生变化，则更新
    if (linear_vel != d_->current_linear_vel_ || angular_vel != d_->current_angular_vel_) {
        d_->current_linear_vel_ = linear_vel;
        d_->current_angular_vel_ = angular_vel;

        // 发送速度命令
        d_->robot_controller_->setLinearVelocity(linear_vel);
        d_->robot_controller_->setAngularVelocity(angular_vel);

        // 更新速度显示
        if (d_->speed_dashboard_) {
            d_->speed_dashboard_->setLinearSpeed(linear_vel);
            d_->speed_dashboard_->setAngularSpeed(angular_vel);
        }
    }
}

void MainWindow::updateRobotState()
{
    if (!d_->robot_controller_) return;
    // TODO: 更新机器人状态显示
}

void MainWindow::updateRobotVelocity()
{
    if (!d_->robot_controller_ || !d_->speed_dashboard_) return;

    // 获取当前速度
    d_->current_linear_vel_ = d_->robot_controller_->getCurrentLinearVelocity();
    d_->current_angular_vel_ = d_->robot_controller_->getCurrentAngularVelocity();

    // 更新速度显示
    d_->speed_dashboard_->setLinearSpeed(d_->current_linear_vel_);
    d_->speed_dashboard_->setAngularSpeed(d_->current_angular_vel_);
}

void MainWindow::setupRosConnections()
{
    d_->goal_pub_ = d_->nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    d_->path_sub_ = d_->nh_.subscribe("/move_base/NavfnROS/plan", 1, &MainWindow::pathCallback, this);
    d_->map_sub_ = d_->nh_.subscribe("/map", 1, &MainWindow::mapCallback, this);
    d_->scan_sub_ = d_->nh_.subscribe("/scan", 1, &MainWindow::scanCallback, this);
    d_->odom_sub_ = d_->nh_.subscribe("/odom", 1, &MainWindow::odomCallback, this);
}

void MainWindow::onDisplayOptionsChanged()
{
    QWidget* content = d_->display_options_dock_->widget();
    if (!content) return;

    QCheckBox* box = content->findChild<QCheckBox*>("grid_box");
    if (box) {
        if (d_->rviz_view_) {
            d_->rviz_view_->setDisplayEnabled("Grid", box->isChecked());
        }
    }

    box = content->findChild<QCheckBox*>("map_box");
    if (box) {
        if (d_->rviz_view_) {
            d_->rviz_view_->setDisplayEnabled("Map", box->isChecked());
        }
    }

    box = content->findChild<QCheckBox*>("robot_box");
    if (box) {
        if (d_->rviz_view_) {
            d_->rviz_view_->setDisplayEnabled("Robot Model", box->isChecked());
        }
    }

    box = content->findChild<QCheckBox*>("laser_box");
    if (box) {
        if (d_->rviz_view_) {
            d_->rviz_view_->setDisplayEnabled("LaserScan", box->isChecked());
        }
    }

    box = content->findChild<QCheckBox*>("path_box");
    if (box) {
        if (d_->rviz_view_) {
            d_->rviz_view_->setDisplayEnabled("Path", box->isChecked());
        }
    }

    box = content->findChild<QCheckBox*>("goal_box");
    if (box) {
        if (d_->rviz_view_) {
            d_->rviz_view_->setDisplayEnabled("Goal", box->isChecked());
        }
    }
}

void MainWindow::pathCallback(const nav_msgs::PathConstPtr& msg)
{
    if (d_->rviz_view_) {
        d_->rviz_view_->updatePath(msg->poses);
    }
}

void MainWindow::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    if (!msg) {
        ROS_WARN("Received null map message");
        return;
    }

    ROS_INFO_ONCE("Received first map update");
    handleMapUpdate(msg);
}

void MainWindow::scanCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    handleScanUpdate(msg);
}

void MainWindow::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    handleOdomUpdate(msg);
    
    // 更新速度显示
    if (d_->speed_dashboard_) {
        double linear_vel = msg->twist.twist.linear.x;
        double angular_vel = msg->twist.twist.angular.z;
        d_->speed_dashboard_->setLinearSpeed(linear_vel);
        d_->speed_dashboard_->setAngularSpeed(angular_vel);
    }
}

void MainWindow::onRobotStatusChanged(const QString& status)
{
    if (statusBar()) {
        statusBar()->showMessage(status);
    }
} 