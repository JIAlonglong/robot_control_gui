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

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , central_widget_(nullptr)
    , rviz_view_(nullptr)
    , status_panel_(nullptr)
    , speed_dashboard_(nullptr)
    , navigation_panel_(nullptr)
    , linear_joystick_(nullptr)
    , angular_joystick_(nullptr)
    , display_options_dock_(nullptr)
    , update_timer_(new QTimer(this))
    , keyboard_timer_(new QTimer(this))
    , robot_controller_(std::make_shared<RobotController>())
{
    // 初始化ROS节点
    int argc = 0;
    char** argv = nullptr;
    ros::init(argc, argv, "robot_control_gui", ros::init_options::NoSigintHandler);
    
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
    QHBoxLayout* main_layout = new QHBoxLayout(central_widget_);
    
    // 创建RViz视图
    rviz_view_ = new RVizView(this);
    rviz_view_->setMinimumSize(800, 600);
    
    // 创建控制面板
    QWidget* control_panel = new QWidget(this);
    QVBoxLayout* control_layout = new QVBoxLayout(control_panel);
    control_panel->setMaximumWidth(350);
    
    // 创建状态面板
    status_panel_ = new RobotStatusPanel(control_panel);
    control_layout->addWidget(status_panel_);

    // 创建速度仪表盘
    speed_dashboard_ = new SpeedDashboard(control_panel);
    control_layout->addWidget(speed_dashboard_);
    
    // 创建摇杆控制组
    QGroupBox* joystick_group = new QGroupBox(tr("摇杆控制"), control_panel);
    QVBoxLayout* joystick_layout = new QVBoxLayout(joystick_group);
    
    // 添加提示标签
    QLabel* joystick_hint = new QLabel(tr("左摇杆: 线速度控制 (上下移动)\n右摇杆: 角速度控制 (左右移动)"));
    joystick_hint->setStyleSheet("color: #666666; font-size: 12px;");
    joystick_hint->setAlignment(Qt::AlignCenter);
    joystick_layout->addWidget(joystick_hint);
    
    // 创建摇杆容器
    QWidget* joysticks_container = new QWidget;
    QHBoxLayout* joysticks_layout = new QHBoxLayout(joysticks_container);
    joysticks_layout->setSpacing(20);
    
    // 创建左摇杆（线速度控制）
    QWidget* left_joystick_widget = new QWidget;
    QVBoxLayout* left_layout = new QVBoxLayout(left_joystick_widget);
    left_layout->setContentsMargins(0, 0, 0, 0);
    
    QLabel* linear_label = new QLabel(tr("线速度控制"));
    linear_label->setAlignment(Qt::AlignCenter);
    linear_label->setStyleSheet("font-weight: bold;");
    
    linear_joystick_ = new JoystickWidget(joystick_group);
    linear_joystick_->setFixedSize(120, 120);
    
    left_layout->addWidget(linear_label);
    left_layout->addWidget(linear_joystick_);
    
    // 创建右摇杆（角速度控制）
    QWidget* right_joystick_widget = new QWidget;
    QVBoxLayout* right_layout = new QVBoxLayout(right_joystick_widget);
    right_layout->setContentsMargins(0, 0, 0, 0);
    
    QLabel* angular_label = new QLabel(tr("角速度控制"));
    angular_label->setAlignment(Qt::AlignCenter);
    angular_label->setStyleSheet("font-weight: bold;");
    
    angular_joystick_ = new JoystickWidget(joystick_group);
    angular_joystick_->setFixedSize(120, 120);
    
    right_layout->addWidget(angular_label);
    right_layout->addWidget(angular_joystick_);
    
    // 将摇杆添加到容器
    joysticks_layout->addStretch();
    joysticks_layout->addWidget(left_joystick_widget);
    joysticks_layout->addWidget(right_joystick_widget);
    joysticks_layout->addStretch();
    
    joystick_layout->addWidget(joysticks_container);
    
    // 添加键盘控制提示
    QGroupBox* keyboard_group = new QGroupBox(tr("键盘控制"), control_panel);
    QVBoxLayout* keyboard_layout = new QVBoxLayout(keyboard_group);
    QLabel* keyboard_hint = new QLabel(tr(
        "↑: 前进\n"
        "↓: 后退\n"
        "←: 左转\n"
        "→: 右转\n"
        "空格: 紧急停止"
    ));
    keyboard_hint->setAlignment(Qt::AlignCenter);
    keyboard_layout->addWidget(keyboard_hint);
    
    // 创建导航面板
    navigation_panel_ = new NavigationPanel(robot_controller_, control_panel);
    
    // 添加到控制面板
    control_layout->addWidget(joystick_group);
    control_layout->addWidget(keyboard_group);
    control_layout->addWidget(navigation_panel_);
    control_layout->addStretch();
    
    // 添加到主布局
    main_layout->addWidget(rviz_view_, 1);
    main_layout->addWidget(control_panel);
    
    // 创建显示选项面板
    createDisplayOptionsPanel();
    
    // 设置状态栏
    statusBar()->showMessage(tr("就绪"));
}

void MainWindow::setupConnections()
{
    // 连接定时器
    update_timer_ = new QTimer(this);
    connect(update_timer_, &QTimer::timeout, this, &MainWindow::updateRobotState);
    update_timer_->start(50);  // 20Hz更新频率

    // 连接键盘定时器
    keyboard_timer_ = new QTimer(this);
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
    // 处理键盘按下事件
    switch (event->key()) {
        case Qt::Key_Up:
            linear_joystick_->setPosition(0.0, -1.0);  // 前进
            break;
        case Qt::Key_Down:
            linear_joystick_->setPosition(0.0, 1.0);   // 后退
            break;
        case Qt::Key_Left:
            angular_joystick_->setPosition(-1.0, 0.0); // 左转
            break;
        case Qt::Key_Right:
            angular_joystick_->setPosition(1.0, 0.0);  // 右转
            break;
        case Qt::Key_Space:
            // 急停：将两个摇杆都归零
            linear_joystick_->setPosition(0.0, 0.0);
            angular_joystick_->setPosition(0.0, 0.0);
            robot_controller_->publishVelocity(0.0, 0.0);
            break;
        default:
            QMainWindow::keyPressEvent(event);
            break;
    }
}

void MainWindow::keyReleaseEvent(QKeyEvent* event)
{
    // 处理键盘释放事件
    switch (event->key()) {
        case Qt::Key_Up:
        case Qt::Key_Down:
            linear_joystick_->setPosition(0.0, 0.0);  // 停止线速度
            break;
        case Qt::Key_Left:
        case Qt::Key_Right:
            angular_joystick_->setPosition(0.0, 0.0); // 停止角速度
            break;
        default:
            QMainWindow::keyReleaseEvent(event);
            break;
    }
}

void MainWindow::updateKeyboardControl()
{
    // 键盘控制已经通过摇杆位置的改变来实现
    // 这个函数可以用来处理一些持续性的键盘控制逻辑
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