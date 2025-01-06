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
    , keyboard_timer_(new QTimer(this))
{
    // 创建ROS控制器
    robot_controller_ = std::make_shared<RobotController>();

    // 设置UI
    setupUi();
    
    // 设置信号连接
    setupConnections();

    // 设置键盘控制定时器
    keyboard_timer_->setInterval(100);  // 10Hz
    connect(keyboard_timer_, &QTimer::timeout, this, &MainWindow::updateKeyboardControl);
    keyboard_timer_->start();

    // 创建并发布一次地图数据
    auto map = std::make_shared<nav_msgs::OccupancyGrid>();
    map->header.frame_id = "map";
    map->header.stamp = ros::Time::now();
    map->info.resolution = 0.05;  // 5cm分辨率
    map->info.width = 200;        // 10米宽
    map->info.height = 200;       // 10米高
    map->info.origin.position.x = -5.0;
    map->info.origin.position.y = -5.0;
    
    // 初始化地图数据
    map->data.resize(map->info.width * map->info.height, 0);
    
    // 创建一些墙壁和障碍物
    for (unsigned int i = 0; i < map->info.width; ++i) {
        // 上下边界
        map->data[i] = 100;
        map->data[i + (map->info.height-1) * map->info.width] = 100;
        // 左右边界
        map->data[i * map->info.width] = 100;
        map->data[i * map->info.width + map->info.width-1] = 100;
    }
    
    // 添加一些固定的障碍物（不再是随机的）
    for (int i = 0; i < 5; ++i) {
        int x = 50 + i * 30;  // 固定间隔的障碍物
        int y = 100;
        for (int dx = -2; dx <= 2; ++dx) {
            for (int dy = -2; dy <= 2; ++dy) {
                int idx = (y + dy) * map->info.width + (x + dx);
                if (0 <= idx && idx < static_cast<int>(map->data.size())) {
                    map->data[idx] = 100;
                }
            }
        }
    }

    // 发布地图数据（只发布一次）
    handleMapUpdate(map);

    // 创建位姿和激光数据更新定时器
    QTimer* pose_timer = new QTimer(this);
    connect(pose_timer, &QTimer::timeout, this, [this]() {
        static double robot_x = 0.0;
        static double robot_y = 0.0;
        static double robot_theta = 0.0;
        
        // 创建机器人位置数据
        auto odom = std::make_shared<nav_msgs::Odometry>();
        odom->header.frame_id = "odom";
        odom->header.stamp = ros::Time::now();
        odom->child_frame_id = "base_link";
        
        // 更新机器人位置（圆周运动）
        robot_x = 2.0 * cos(robot_theta);
        robot_y = 2.0 * sin(robot_theta);
        robot_theta += 0.02;  // 每次更新旋转一点
        
        odom->pose.pose.position.x = robot_x;
        odom->pose.pose.position.y = robot_y;
        odom->pose.pose.orientation.w = cos(robot_theta / 2.0);
        odom->pose.pose.orientation.z = sin(robot_theta / 2.0);

        // 创建激光扫描数据
        auto scan = std::make_shared<sensor_msgs::LaserScan>();
        scan->header.frame_id = "base_laser";
        scan->header.stamp = ros::Time::now();
        scan->angle_min = -M_PI/2;
        scan->angle_max = M_PI/2;
        scan->angle_increment = M_PI/180;  // 1度分辨率
        scan->range_min = 0.1;
        scan->range_max = 10.0;
        
        // 生成180个激光点
        scan->ranges.resize(180);
        for (int i = 0; i < 180; ++i) {
            double angle = scan->angle_min + i * scan->angle_increment;
            double range = 2.0 + 0.5 * sin(angle * 2.0);  // 生成一些动态的激光数据
            scan->ranges[i] = range;
        }

        // 发布数据
        handleOdomUpdate(odom);
        handleScanUpdate(scan);
    });
    
    // 启动位姿更新定时器
    pose_timer->start(50);  // 20Hz更新位姿和激光数据
}

MainWindow::~MainWindow()
{
}

void MainWindow::setupUi()
{
    // 设置窗口标题和大小
    setWindowTitle("TurtleBot3 控制面板");
    resize(1200, 800);

    // 创建中央部件和主布局
    QWidget* central_widget = new QWidget(this);
    setCentralWidget(central_widget);
    QHBoxLayout* main_layout = new QHBoxLayout(central_widget);

    // 创建左侧控制面板
    QWidget* control_panel = new QWidget;
    QVBoxLayout* control_layout = new QVBoxLayout(control_panel);
    control_panel->setFixedWidth(300);

    // 添加机器人状态面板
    status_panel_ = new RobotStatusPanel;
    control_layout->addWidget(status_panel_);

    // 添加速度仪表盘
    speed_dashboard_ = new SpeedDashboard;
    control_layout->addWidget(speed_dashboard_);

    // 添加导航面板
    navigation_panel_ = new NavigationPanel(robot_controller_);
    control_layout->addWidget(navigation_panel_);

    // 添加摇杆控制器
    joystick_ = new JoystickWidget;
    joystick_->setFixedSize(200, 200);
    QHBoxLayout* joystick_layout = new QHBoxLayout;
    joystick_layout->addWidget(joystick_);
    joystick_layout->setAlignment(Qt::AlignCenter);
    control_layout->addLayout(joystick_layout);

    // 添加控制按钮
    QHBoxLayout* button_layout = new QHBoxLayout;
    QPushButton* start_mapping_btn = new QPushButton("开始建图");
    start_mapping_btn->setObjectName("start_mapping_btn");
    QPushButton* stop_mapping_btn = new QPushButton("停止建图");
    stop_mapping_btn->setObjectName("stop_mapping_btn");
    QPushButton* save_map_btn = new QPushButton("保存地图");
    save_map_btn->setObjectName("save_map_btn");
    button_layout->addWidget(start_mapping_btn);
    button_layout->addWidget(stop_mapping_btn);
    button_layout->addWidget(save_map_btn);
    control_layout->addLayout(button_layout);

    QHBoxLayout* button_layout2 = new QHBoxLayout;
    QPushButton* load_map_btn = new QPushButton("加载地图");
    load_map_btn->setObjectName("load_map_btn");
    QPushButton* set_pose_btn = new QPushButton("设置初始位置");
    set_pose_btn->setObjectName("set_pose_btn");
    QPushButton* clear_costmap_btn = new QPushButton("清除代价地图");
    clear_costmap_btn->setObjectName("clear_costmap_btn");
    button_layout2->addWidget(load_map_btn);
    button_layout2->addWidget(set_pose_btn);
    button_layout2->addWidget(clear_costmap_btn);
    control_layout->addLayout(button_layout2);

    // 添加导航模式选择
    QGroupBox* nav_mode_group = new QGroupBox("导航模式");
    QVBoxLayout* nav_mode_layout = new QVBoxLayout(nav_mode_group);
    QRadioButton* normal_mode = new QRadioButton("普通模式");
    normal_mode->setObjectName("normal_mode");
    QRadioButton* fast_mode = new QRadioButton("快速模式");
    fast_mode->setObjectName("fast_mode");
    QRadioButton* precise_mode = new QRadioButton("精确模式");
    precise_mode->setObjectName("precise_mode");
    normal_mode->setChecked(true);
    nav_mode_layout->addWidget(normal_mode);
    nav_mode_layout->addWidget(fast_mode);
    nav_mode_layout->addWidget(precise_mode);
    control_layout->addWidget(nav_mode_group);

    // 添加状态信息标签
    QLabel* status_label = new QLabel("状态: 就绪");
    control_layout->addWidget(status_label);

    // 添加伸缩器
    control_layout->addStretch();

    // 创建地图显示区域
    map_view_ = new MapView;
    
    // 将控制面板和地图添加到主布局
    main_layout->addWidget(control_panel);
    main_layout->addWidget(map_view_, 1);

    // 设置样式
    setStyleSheet(R"(
        QMainWindow {
            background-color: #f0f0f0;
        }
        QPushButton {
            background-color: #4CAF50;
            color: white;
            border: none;
            padding: 5px;
            min-height: 30px;
            border-radius: 4px;
        }
        QPushButton:hover {
            background-color: #45a049;
        }
        QPushButton:pressed {
            background-color: #3d8b40;
        }
        QGroupBox {
            border: 1px solid #cccccc;
            border-radius: 4px;
            margin-top: 1ex;
            padding: 5px;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 3px 0 3px;
        }
        QLabel {
            color: #333333;
        }
    )");

    // 创建键盘控制定时器
    keyboard_timer_ = new QTimer(this);
    keyboard_timer_->setInterval(50);  // 20Hz更新频率
}

void MainWindow::setupConnections()
{
    // 连接摇杆信号
    connect(joystick_, &JoystickWidget::positionChanged,
            this, &MainWindow::onJoystickMoved);

    // 连接键盘定时器
    connect(keyboard_timer_, &QTimer::timeout,
            this, &MainWindow::updateKeyboardControl);

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
    map_view_->updateMap(map);
}

void MainWindow::handleOdomUpdate(const std::shared_ptr<nav_msgs::Odometry>& odom)
{
    map_view_->updateRobotPose(odom);
}

void MainWindow::handleScanUpdate(const std::shared_ptr<sensor_msgs::LaserScan>& scan)
{
    map_view_->updateLaserScan(scan);
}

void MainWindow::onJoystickMoved(double x, double y)
{
    // TODO: 实现机器人速度控制
}

void MainWindow::keyPressEvent(QKeyEvent* event)
{
    // TODO: 处理键盘按下事件
    QMainWindow::keyPressEvent(event);
}

void MainWindow::keyReleaseEvent(QKeyEvent* event)
{
    // TODO: 处理键盘释放事件
    QMainWindow::keyReleaseEvent(event);
}

void MainWindow::updateKeyboardControl()
{
    // TODO: 实现键盘控制逻辑
} 