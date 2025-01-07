/**
 * @file main_window.cpp
 * @brief 主窗口类的实现
 */

#include "ui/main_window.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QGraphicsDropShadowEffect>
#include <QGraphicsEffect>
#include <QLabel>
#include <QGroupBox>
#include <QRadioButton>
#include <QPushButton>
#include <QStackedWidget>
#include <QToolBar>
#include <QAction>
#include <QStatusBar>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , central_widget_(nullptr)
    , stacked_widget_(nullptr)
    , tool_bar_(nullptr)
    , rviz_view_(nullptr)
    , status_panel_(nullptr)
    , speed_dashboard_(nullptr)
    , navigation_panel_(nullptr)
    , linear_joystick_(nullptr)
    , angular_joystick_(nullptr)
    , display_options_dock_(nullptr)
    , camera_view_(nullptr)
    , control_page_(nullptr)
    , navigation_page_(nullptr)
    , sensor_page_(nullptr)
    , mapping_page_(nullptr)
    , update_timer_(new QTimer(this))
    , keyboard_timer_(new QTimer(this))
    , robot_controller_(std::make_shared<RobotController>())
{
    // 初始化ROS节点（如果尚未初始化）
    if (!ros::isInitialized()) {
        int argc = 0;
        char** argv = nullptr;
        ros::init(argc, argv, "robot_control_gui", ros::init_options::NoSigintHandler);
    }
    
    // 设置UI
    setupUi();
    
    // 设置ROS连接
    setupRosConnections();
    
    // 设置信号连接
    setupConnections();

    // 设置键盘控制定时器
    keyboard_timer_->setInterval(100);  // 10Hz
    keyboard_timer_->start();

    // 设置状态更新定时器
    update_timer_->setInterval(50);  // 20Hz
    update_timer_->start();

    // 设置焦点策略，使窗口可以接收键盘事件
    setFocusPolicy(Qt::StrongFocus);
}

MainWindow::~MainWindow()
{
}

void MainWindow::setupUi()
{
    // 设置窗口标题和大小
    setWindowTitle(tr("TurtleBot3 控制面板"));
    resize(1200, 800);

    // 创建中央部件
    central_widget_ = new QWidget(this);
    setCentralWidget(central_widget_);

    // 创建主布局
    QVBoxLayout* main_layout = new QVBoxLayout(central_widget_);
    main_layout->setContentsMargins(0, 0, 0, 0);  // 移除边距
    
    // 创建工具栏
    createToolBar();
    
    // 创建堆叠式窗口部件
    stacked_widget_ = new QStackedWidget(this);
    main_layout->addWidget(stacked_widget_);
    
    // 创建各个页面
    createPages();
    
    // 创建浮动控制面板
    createFloatingControlPanel();
    
    // 创建状态栏
    statusBar()->showMessage(tr("就绪"));
}

void MainWindow::createToolBar()
{
    tool_bar_ = addToolBar(tr("工具栏"));
    tool_bar_->setMovable(false);
    tool_bar_->setIconSize(QSize(32, 32));

    // 添加工具栏按钮
    QAction* control_action = tool_bar_->addAction(tr("控制"));
    QAction* navigation_action = tool_bar_->addAction(tr("导航"));

    // 连接信号
    connect(control_action, &QAction::triggered, [this]() { switchToPage(0); });
    connect(navigation_action, &QAction::triggered, [this]() { switchToPage(1); });
}

void MainWindow::createPages()
{
    // 创建主页面
    QWidget* main_page = new QWidget;
    QHBoxLayout* main_layout = new QHBoxLayout(main_page);
    main_layout->setContentsMargins(0, 0, 0, 0);
    
    // 左侧状态面板
    QWidget* status_widget = new QWidget;
    QVBoxLayout* status_layout = new QVBoxLayout(status_widget);
    status_layout->setContentsMargins(5, 5, 5, 5);
    
    status_panel_ = new RobotStatusPanel(status_widget);
    status_layout->addWidget(status_panel_);
    status_layout->addStretch();
    
    status_widget->setMaximumWidth(250);
    main_layout->addWidget(status_widget);
    
    // 中央显示区域
    QWidget* center_widget = new QWidget;
    QVBoxLayout* center_layout = new QVBoxLayout(center_widget);
    center_layout->setContentsMargins(0, 0, 0, 0);
    
    // 摄像头视图（调整大小）
    camera_view_ = new CameraView(center_widget);
    camera_view_->setMinimumHeight(300);
    camera_view_->setMaximumHeight(400);
    center_layout->addWidget(camera_view_);
    
    main_layout->addWidget(center_widget);
    
    // 创建导航页面
    QWidget* nav_page = new QWidget;
    QHBoxLayout* nav_layout = new QHBoxLayout(nav_page);
    nav_layout->setContentsMargins(0, 0, 0, 0);
    
    // 左侧地图显示
    rviz_view_ = new RVizView(nav_page);
    
    // 右侧导航控制面板
    QWidget* nav_control_panel = new QWidget;
    QVBoxLayout* nav_control_layout = new QVBoxLayout(nav_control_panel);
    nav_control_layout->setContentsMargins(5, 5, 5, 5);
    
    navigation_panel_ = new NavigationPanel(robot_controller_, nav_control_panel);
    nav_control_layout->addWidget(navigation_panel_);
    
    // 添加建图控制
    QGroupBox* mapping_group = new QGroupBox(tr("建图控制"));
    QVBoxLayout* mapping_layout = new QVBoxLayout(mapping_group);
    
    QPushButton* start_mapping_btn = new QPushButton(tr("开始建图"));
    QPushButton* stop_mapping_btn = new QPushButton(tr("停止建图"));
    QPushButton* save_map_btn = new QPushButton(tr("保存地图"));
    
    mapping_layout->addWidget(start_mapping_btn);
    mapping_layout->addWidget(stop_mapping_btn);
    mapping_layout->addWidget(save_map_btn);
    
    nav_control_layout->addWidget(mapping_group);
    nav_control_layout->addStretch();
    
    nav_control_panel->setMaximumWidth(300);
    
    // 添加到导航页面布局
    nav_layout->addWidget(rviz_view_);
    nav_layout->addWidget(nav_control_panel);
    
    // 添加页面到堆叠式窗口部件
    stacked_widget_->addWidget(main_page);
    stacked_widget_->addWidget(nav_page);
    
    // 默认显示主页面
    stacked_widget_->setCurrentIndex(0);
}

void MainWindow::createFloatingControlPanel()
{
    // 创建浮动控制面板
    QDockWidget* control_dock = new QDockWidget(tr("控制面板"), this);
    control_dock->setAllowedAreas(Qt::AllDockWidgetAreas);
    control_dock->setFeatures(QDockWidget::DockWidgetFloatable | 
                             QDockWidget::DockWidgetMovable);
    
    QWidget* control_widget = new QWidget(control_dock);
    QVBoxLayout* control_layout = new QVBoxLayout(control_widget);
    control_layout->setContentsMargins(5, 5, 5, 5);
    
    // 添加速度仪表盘（紧凑版）
    QHBoxLayout* dashboard_layout = new QHBoxLayout;
    speed_dashboard_ = new SpeedDashboard(control_widget);
    speed_dashboard_->setMaximumHeight(100);  // 限制仪表盘高度
    dashboard_layout->addWidget(speed_dashboard_);
    control_layout->addLayout(dashboard_layout);
    
    // 添加摇杆控制
    QGroupBox* joystick_group = new QGroupBox(tr("摇杆控制"));
    QVBoxLayout* joystick_layout = new QVBoxLayout(joystick_group);
    joystick_layout->setSpacing(5);
    
    QWidget* joysticks_container = new QWidget;
    QHBoxLayout* joysticks_layout = new QHBoxLayout(joysticks_container);
    joysticks_layout->setSpacing(10);
    
    linear_joystick_ = new JoystickWidget;
    angular_joystick_ = new JoystickWidget;
    linear_joystick_->setFixedSize(100, 100);
    angular_joystick_->setFixedSize(100, 100);
    
    joysticks_layout->addWidget(linear_joystick_);
    joysticks_layout->addWidget(angular_joystick_);
    
    joystick_layout->addWidget(joysticks_container);
    control_layout->addWidget(joystick_group);
    
    // 添加键盘控制提示
    QGroupBox* keyboard_group = new QGroupBox(tr("键盘控制"));
    QVBoxLayout* keyboard_layout = new QVBoxLayout(keyboard_group);
    QLabel* keyboard_hint = new QLabel(tr(
        "↑: 前进  ↓: 后退\n"
        "←: 左转  →: 右转\n"
        "空格: 紧急停止"
    ));
    keyboard_hint->setAlignment(Qt::AlignCenter);
    keyboard_layout->addWidget(keyboard_hint);
    control_layout->addWidget(keyboard_group);
    
    control_dock->setWidget(control_widget);
    addDockWidget(Qt::RightDockWidgetArea, control_dock);
    control_dock->setMinimumWidth(250);
    control_dock->setMaximumWidth(300);
}

void MainWindow::switchToPage(int index)
{
    stacked_widget_->setCurrentIndex(index);
    
    // 根据页面更新状态
    switch (index) {
        case 0:
            statusBar()->showMessage(tr("控制模式"));
            break;
        case 1:
            statusBar()->showMessage(tr("导航模式"));
            break;
    }
}

void MainWindow::setupConnections()
{
    // 连接定时器
    connect(update_timer_, &QTimer::timeout, this, [this]() {
        // 处理ROS消息
        if (ros::ok()) {
            ros::spinOnce();
            updateRobotState();
        }
    });
    update_timer_->start(50);  // 20Hz更新频率

    // 连接键盘定时器
    connect(keyboard_timer_, &QTimer::timeout, this, &MainWindow::updateKeyboardControl);
    keyboard_timer_->start(100);  // 10Hz更新频率

    // 连接摇杆信号
    connect(linear_joystick_, &JoystickWidget::positionChanged,
            this, [this](double x, double y) {
                double linear_vel = -y * robot_controller_->getMaxLinearVelocity();
                robot_controller_->setLinearVelocity(linear_vel);
                speed_dashboard_->setLinearSpeed(linear_vel);
            });
    
    connect(angular_joystick_, &JoystickWidget::positionChanged,
            this, [this](double x, double y) {
                double angular_vel = -x * robot_controller_->getMaxAngularVelocity();
                robot_controller_->setAngularVelocity(angular_vel);
                speed_dashboard_->setAngularSpeed(angular_vel);
            });

    // 连接地图视图信号
    connect(rviz_view_, &RVizView::goalSelected,
            this, &MainWindow::onGoalSelected);

    // 连接ROS控制器的数据更新信号
    connect(robot_controller_.get(), &RobotController::mapUpdated,
            this, &MainWindow::handleMapUpdate);
    connect(robot_controller_.get(), &RobotController::odomUpdated,
            this, &MainWindow::handleOdomUpdate);
    connect(robot_controller_.get(), &RobotController::scanUpdated,
            this, &MainWindow::handleScanUpdate);
    connect(robot_controller_.get(), &RobotController::statusChanged,
            this, &MainWindow::onRobotStatusChanged);

    // 连接导航面板的信号
    connect(navigation_panel_, &NavigationPanel::navigationGoalSet,
            [this](double x, double y, double theta) {
                robot_controller_->setNavigationGoal(x, y, theta);
            });

    // 连接按钮信号
    QPushButton* start_mapping_btn = findChild<QPushButton*>("start_mapping_btn");
    connect(start_mapping_btn, &QPushButton::clicked,
            [this]() { robot_controller_->startMapping(); });

    QPushButton* stop_mapping_btn = findChild<QPushButton*>("stop_mapping_btn");
    connect(stop_mapping_btn, &QPushButton::clicked,
            [this]() { robot_controller_->stopMapping(); });

    QPushButton* save_map_btn = findChild<QPushButton*>("save_map_btn");
    connect(save_map_btn, &QPushButton::clicked,
            [this]() { robot_controller_->saveMap("map"); });

    QPushButton* load_map_btn = findChild<QPushButton*>("load_map_btn");
    connect(load_map_btn, &QPushButton::clicked,
            [this]() { robot_controller_->loadMap("map.yaml"); });

    QPushButton* set_pose_btn = findChild<QPushButton*>("set_pose_btn");
    connect(set_pose_btn, &QPushButton::clicked,
            [this]() { /* 实现设置初始位置的逻辑 */ });

    QPushButton* clear_costmap_btn = findChild<QPushButton*>("clear_costmap_btn");
    connect(clear_costmap_btn, &QPushButton::clicked,
            [this]() { robot_controller_->updateCostmap(); });

    // 连接导航模式选择
    QRadioButton* normal_mode = findChild<QRadioButton*>("normal_mode");
    connect(normal_mode, &QRadioButton::toggled,
            [this](bool checked) { if (checked) robot_controller_->setNavigationMode(0); });

    QRadioButton* fast_mode = findChild<QRadioButton*>("fast_mode");
    connect(fast_mode, &QRadioButton::toggled,
            [this](bool checked) { if (checked) robot_controller_->setNavigationMode(1); });

    QRadioButton* precise_mode = findChild<QRadioButton*>("precise_mode");
    connect(precise_mode, &QRadioButton::toggled,
            [this](bool checked) { if (checked) robot_controller_->setNavigationMode(2); });
}

void MainWindow::handleMapUpdate(const std::shared_ptr<nav_msgs::OccupancyGrid>& map)
{
    rviz_view_->updateMap(map);
}

void MainWindow::handleOdomUpdate(const std::shared_ptr<nav_msgs::Odometry>& odom)
{
    rviz_view_->updateRobotPose(odom);
}

void MainWindow::handleScanUpdate(const std::shared_ptr<sensor_msgs::LaserScan>& scan)
{
    rviz_view_->updateLaserScan(scan);
}

void MainWindow::onJoystickMoved(double x, double y)
{
    // TODO: 实现机器人速度控制
}

void MainWindow::keyPressEvent(QKeyEvent* event)
{
    if (event->isAutoRepeat()) {
        return;  // 忽略按键重复
    }

    switch (event->key()) {
        case Qt::Key_Up:
            current_linear_vel_ = max_linear_vel_;
            break;
        case Qt::Key_Down:
            current_linear_vel_ = -max_linear_vel_;
            break;
        case Qt::Key_Left:
            current_angular_vel_ = max_angular_vel_;
            break;
        case Qt::Key_Right:
            current_angular_vel_ = -max_angular_vel_;
            break;
        case Qt::Key_Space:
            // 紧急停止
            current_linear_vel_ = 0.0;
            current_angular_vel_ = 0.0;
            robot_controller_->setLinearVelocity(0.0);
            robot_controller_->setAngularVelocity(0.0);
            break;
    }
    // 更新按键状态
    key_states_[event->key()] = true;
    updateRobotVelocity();
}

void MainWindow::keyReleaseEvent(QKeyEvent* event)
{
    if (event->isAutoRepeat()) {
        return;  // 忽略按键重复
    }

    // 更新按键状态
    key_states_[event->key()] = false;

    // 检查其他方向键的状态
    if (event->key() == Qt::Key_Up || event->key() == Qt::Key_Down) {
        if (!key_states_[Qt::Key_Up] && !key_states_[Qt::Key_Down]) {
            current_linear_vel_ = 0.0;
        }
    }
    if (event->key() == Qt::Key_Left || event->key() == Qt::Key_Right) {
        if (!key_states_[Qt::Key_Left] && !key_states_[Qt::Key_Right]) {
            current_angular_vel_ = 0.0;
        }
    }
    updateRobotVelocity();
}

void MainWindow::updateRobotVelocity()
{
    // 发送速度命令到机器人
    robot_controller_->setLinearVelocity(current_linear_vel_);
    robot_controller_->setAngularVelocity(current_angular_vel_);
    
    // 更新速度仪表盘显示
    if (speed_dashboard_) {
        speed_dashboard_->setLinearSpeed(current_linear_vel_);
        speed_dashboard_->setAngularSpeed(current_angular_vel_);
    }
}

void MainWindow::updateKeyboardControl()
{
    // 定期更新速度命令，确保持续运动
    if (key_states_[Qt::Key_Up]) {
        current_linear_vel_ = max_linear_vel_;
    } else if (key_states_[Qt::Key_Down]) {
        current_linear_vel_ = -max_linear_vel_;
    }

    if (key_states_[Qt::Key_Left]) {
        current_angular_vel_ = max_angular_vel_;
    } else if (key_states_[Qt::Key_Right]) {
        current_angular_vel_ = -max_angular_vel_;
    }

    updateRobotVelocity();
}

void MainWindow::setupRosConnections()
{
    // 设置发布器和订阅器
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    path_sub_ = nh_.subscribe("/move_base/NavfnROS/plan", 1, &MainWindow::pathCallback, this);
    map_sub_ = nh_.subscribe("/map", 1, &MainWindow::mapCallback, this);
    scan_sub_ = nh_.subscribe("/scan", 1, &MainWindow::scanCallback, this);
    odom_sub_ = nh_.subscribe("/odom", 1, &MainWindow::odomCallback, this);
}

void MainWindow::createDisplayOptionsPanel()
{
    // 创建显示选项面板
    QDockWidget* dock = new QDockWidget(tr("显示选项"), this);
    dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    
    QWidget* content = new QWidget(dock);
    QVBoxLayout* layout = new QVBoxLayout(content);
    
    // 创建复选框
    auto createCheckBox = [](const QString& text, bool checked) {
        QCheckBox* box = new QCheckBox(text);
        box->setChecked(checked);
        return box;
    };
    
    QCheckBox* show_grid = createCheckBox("显示网格", true);
    QCheckBox* show_map = createCheckBox("显示地图", true);
    QCheckBox* show_robot = createCheckBox("显示机器人", true);
    QCheckBox* show_laser = createCheckBox("显示激光扫描", true);
    QCheckBox* show_path = createCheckBox("显示路径", true);
    QCheckBox* show_goal = createCheckBox("显示目标点", true);
    
    // 添加到布局
    layout->addWidget(show_grid);
    layout->addWidget(show_map);
    layout->addWidget(show_robot);
    layout->addWidget(show_laser);
    layout->addWidget(show_path);
    layout->addWidget(show_goal);
    layout->addStretch();
    
    // 连接信号
    connect(show_grid, &QCheckBox::toggled, this, &MainWindow::onDisplayOptionsChanged);
    connect(show_map, &QCheckBox::toggled, this, &MainWindow::onDisplayOptionsChanged);
    connect(show_robot, &QCheckBox::toggled, this, &MainWindow::onDisplayOptionsChanged);
    connect(show_laser, &QCheckBox::toggled, this, &MainWindow::onDisplayOptionsChanged);
    connect(show_path, &QCheckBox::toggled, this, &MainWindow::onDisplayOptionsChanged);
    connect(show_goal, &QCheckBox::toggled, this, &MainWindow::onDisplayOptionsChanged);
    
    dock->setWidget(content);
    addDockWidget(Qt::RightDockWidgetArea, dock);
    display_options_dock_ = dock;
}

void MainWindow::onDisplayOptionsChanged()
{
    QWidget* content = display_options_dock_->widget();
    const QList<QCheckBox*> boxes = content->findChildren<QCheckBox*>();
    
    for (const QCheckBox* box : boxes) {
        if (box->text() == "显示网格") {
            rviz_view_->setDisplayEnabled("Grid", box->isChecked());
        }
        else if (box->text() == "显示地图") {
            rviz_view_->setDisplayEnabled("Map", box->isChecked());
        }
        else if (box->text() == "显示机器人") {
            rviz_view_->setDisplayEnabled("Robot Model", box->isChecked());
        }
        else if (box->text() == "显示激光扫描") {
            rviz_view_->setDisplayEnabled("LaserScan", box->isChecked());
        }
        else if (box->text() == "显示路径") {
            rviz_view_->setDisplayEnabled("Path", box->isChecked());
        }
        else if (box->text() == "显示目标点") {
            rviz_view_->setDisplayEnabled("Goal", box->isChecked());
        }
    }
}

void MainWindow::onGoalSelected(const geometry_msgs::PoseStamped& goal)
{
    // 发布导航目标点
    goal_pub_.publish(goal);
}

// ROS回调函数
void MainWindow::pathCallback(const nav_msgs::Path::ConstPtr& path)
{
    if (!path) return;
    rviz_view_->updatePath(path->poses);
}

void MainWindow::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    if (!map) return;
    rviz_view_->updateMap(std::make_shared<nav_msgs::OccupancyGrid>(*map));
}

void MainWindow::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    if (!scan) return;
    rviz_view_->updateLaserScan(std::make_shared<sensor_msgs::LaserScan>(*scan));
}

void MainWindow::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
    if (!odom) return;
    rviz_view_->updateRobotPose(std::make_shared<nav_msgs::Odometry>(*odom));
}

void MainWindow::updateRobotState()
{
    if (!ros::ok()) return;

    // 更新机器人状态面板
    if (status_panel_) {
        status_panel_->updateBatteryLevel(robot_controller_->getBatteryLevel());
        status_panel_->updateWifiStrength(robot_controller_->getWifiStrength());
        status_panel_->updateStatus(robot_controller_->getStatus());
    }

    // 更新速度仪表盘
    if (speed_dashboard_) {
        speed_dashboard_->setLinearSpeed(robot_controller_->getCurrentLinearVelocity());
        speed_dashboard_->setAngularSpeed(robot_controller_->getCurrentAngularVelocity());
    }

    // 尝试获取最新的里程计数据
    auto odom = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", ros::Duration(0.1));
    if (odom) {
        auto std_odom = std::make_shared<nav_msgs::Odometry>(*odom);
        handleOdomUpdate(std_odom);
    }

    // 尝试获取最新的激光数据
    auto scan = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", ros::Duration(0.1));
    if (scan) {
        auto std_scan = std::make_shared<sensor_msgs::LaserScan>(*scan);
        handleScanUpdate(std_scan);
    }
}

void MainWindow::onRobotStatusChanged(const QString& status)
{
    // 在状态栏显示状态信息
    statusBar()->showMessage(status);
    
    // 如果是导航服务器未连接的消息，禁用相关功能
    if (status.contains("导航服务器未连接")) {
        if (navigation_panel_) {
            navigation_panel_->setEnabled(false);
        }
    }
} 