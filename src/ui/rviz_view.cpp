#include "ui/rviz_view.h"

#include <QVBoxLayout>
#include <QPainter>
#include <QPaintEvent>
#include <QTimer>
#include <QVariant>
#include <QThread>

#include <rviz/display.h>
#include <rviz/tool.h>
#include <rviz/tool_manager.h>
#include <rviz/display_group.h>
#include <rviz/properties/property_tree_model.h>

RVizView::RVizView(QWidget* parent)
    : QWidget(parent)
{
    // 创建布局
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    // 创建状态标签
    status_label_ = new QLabel(this);
    status_label_->setAlignment(Qt::AlignCenter);
    status_label_->setStyleSheet("QLabel { color: red; }");
    status_label_->hide();
    layout->addWidget(status_label_);

    // 创建渲染面板
    render_panel_ = new rviz::RenderPanel();
    layout->addWidget(render_panel_);

    // 创建可视化管理器
    manager_ = new rviz::VisualizationManager(render_panel_);
    render_panel_->initialize(manager_->getSceneManager(), manager_);

    // 设置固定坐标系
    manager_->setFixedFrame(QString("map"));

    // 初始化工具管理器
    manager_->initialize();

    // 设置工具
    auto* tool_manager = manager_->getToolManager();
    auto* move_camera_tool = tool_manager->addTool("rviz/MoveCamera");
    goal_tool_ = tool_manager->addTool("rviz/SetGoal");
    initial_pose_tool_ = tool_manager->addTool("rviz/SetInitialPose");

    // 设置默认工具
    if (move_camera_tool) {
        tool_manager->setCurrentTool(move_camera_tool);
        ROS_INFO("Set default tool to MoveCamera");
    }

    // 连接工具信号
    if (goal_tool_) {
        connect(goal_tool_, SIGNAL(positionSelected(const Ogre::Vector3&, const Ogre::Quaternion&)),
                this, SLOT(onGoalPositionSelected(const Ogre::Vector3&, const Ogre::Quaternion&)));
        ROS_INFO("Goal tool connected");
    } else {
        ROS_ERROR("Failed to create goal tool");
    }

    if (initial_pose_tool_) {
        connect(initial_pose_tool_, SIGNAL(positionSelected(const Ogre::Vector3&, const Ogre::Quaternion&)),
                this, SLOT(onInitialPoseSelected(const Ogre::Vector3&, const Ogre::Quaternion&)));
        ROS_INFO("Initial pose tool connected");
    } else {
        ROS_ERROR("Failed to create initial pose tool");
    }

    // 设置显示组件
    setupDisplays();

    // 启动渲染
    manager_->startUpdate();
}

RVizView::~RVizView()
{
    cleanup();
}

void RVizView::cleanup()
{
    if (manager_) {
        manager_->stopUpdate();
        delete manager_;
        manager_ = nullptr;
    }

    if (render_panel_) {
        delete render_panel_;
        render_panel_ = nullptr;
    }
}

void RVizView::setupDisplays()
{
    // 创建网格显示
    grid_display_ = manager_->createDisplay("rviz/Grid", "Grid", true);
    if (!grid_display_) {
        ROS_ERROR("Failed to create Grid display");
        return;
    }
    grid_display_->subProp("Cell Size")->setValue(1.0);
    grid_display_->subProp("Color")->setValue(QColor(125, 125, 125));

    // 创建机器人模型显示
    robot_model_display_ = manager_->createDisplay("rviz/RobotModel", "Robot Model", true);
    if (!robot_model_display_) {
        ROS_ERROR("Failed to create RobotModel display");
        return;
    }

    // 创建地图显示
    map_display_ = manager_->createDisplay("rviz/Map", "Map", true);
    if (!map_display_) {
        ROS_ERROR("Failed to create Map display");
        return;
    }
    // 设置地图显示属性
    map_display_->subProp("Topic")->setValue("/map");
    map_display_->subProp("Color Scheme")->setValue("costmap");
    map_display_->subProp("Draw Behind")->setValue(true);
    map_display_->subProp("Alpha")->setValue(0.7);
    map_display_->subProp("Update Topic")->setValue("/map_updates");

    // 创建激光扫描显示
    laser_scan_display_ = manager_->createDisplay("rviz/LaserScan", "LaserScan", true);
    if (!laser_scan_display_) {
        ROS_ERROR("Failed to create LaserScan display");
        return;
    }
    laser_scan_display_->subProp("Topic")->setValue("/scan");
    laser_scan_display_->subProp("Size (m)")->setValue(0.05);
    laser_scan_display_->subProp("Style")->setValue("Points");

    // 创建全局路径显示
    global_path_display_ = manager_->createDisplay("rviz/Path", "Global Path", true);
    if (!global_path_display_) {
        ROS_ERROR("Failed to create Global Path display");
        return;
    }
    global_path_display_->subProp("Topic")->setValue("/move_base/GlobalPlanner/plan");
    global_path_display_->subProp("Color")->setValue(QColor(0, 255, 0));  // 绿色
    global_path_display_->subProp("Alpha")->setValue(1.0);
    global_path_display_->subProp("Line Width")->setValue(0.1);

    // 创建局部路径显示
    local_path_display_ = manager_->createDisplay("rviz/Path", "Local Path", true);
    if (!local_path_display_) {
        ROS_ERROR("Failed to create Local Path display");
        return;
    }
    local_path_display_->subProp("Topic")->setValue("/move_base/DWAPlannerROS/local_plan");
    local_path_display_->subProp("Color")->setValue(QColor(255, 0, 0));  // 红色
    local_path_display_->subProp("Alpha")->setValue(1.0);
    local_path_display_->subProp("Line Width")->setValue(0.1);

    // 创建目标点显示
    goal_display_ = manager_->createDisplay("rviz/Pose", "Navigation Goal", true);
    if (!goal_display_) {
        ROS_ERROR("Failed to create Goal display");
        return;
    }
    goal_display_->subProp("Topic")->setValue("/move_base_simple/goal");
    goal_display_->subProp("Color")->setValue(QColor(255, 0, 0));  // 红色
    goal_display_->subProp("Alpha")->setValue(1.0);
    goal_display_->subProp("Shaft Length")->setValue(1.0);  // 增加箭头长度
    goal_display_->subProp("Head Length")->setValue(0.5);   // 增加箭头头部长度
    goal_display_->subProp("Head Radius")->setValue(0.3);   // 增加箭头头部半径
    goal_display_->subProp("Shaft Radius")->setValue(0.1);  // 增加箭头轴半径

    // 创建当前目标点显示
    auto* current_goal_display = manager_->createDisplay("rviz/Pose", "Current Goal", true);
    if (current_goal_display) {
        current_goal_display->subProp("Topic")->setValue("/move_base/current_goal");
        current_goal_display->subProp("Color")->setValue(QColor(0, 255, 0));  // 绿色
        current_goal_display->subProp("Alpha")->setValue(1.0);
        current_goal_display->subProp("Shaft Length")->setValue(1.0);
        current_goal_display->subProp("Head Length")->setValue(0.5);
        current_goal_display->subProp("Head Radius")->setValue(0.3);
        current_goal_display->subProp("Shaft Radius")->setValue(0.1);
    }

    // 创建机器人位姿显示
    auto* robot_pose_display = manager_->createDisplay("rviz/PoseWithCovariance", "Robot Pose", true);
    if (robot_pose_display) {
        robot_pose_display->subProp("Topic")->setValue("/amcl_pose");
        robot_pose_display->subProp("Color")->setValue(QColor(255, 255, 0));  // 黄色
        robot_pose_display->subProp("Alpha")->setValue(1.0);
        robot_pose_display->subProp("Shaft Length")->setValue(1.0);
        robot_pose_display->subProp("Head Length")->setValue(0.3);
        robot_pose_display->subProp("Covariance")->setValue(true);
    }

    // 创建全局代价地图显示
    auto* global_costmap_display = manager_->createDisplay("rviz/Map", "Global Costmap", true);
    if (global_costmap_display) {
        global_costmap_display->subProp("Topic")->setValue("/move_base/global_costmap/costmap");
        global_costmap_display->subProp("Color Scheme")->setValue("raw");
        global_costmap_display->subProp("Draw Behind")->setValue(false);
        global_costmap_display->subProp("Alpha")->setValue(0.4);
    }

    // 创建局部代价地图显示
    auto* local_costmap_display = manager_->createDisplay("rviz/Map", "Local Costmap", true);
    if (local_costmap_display) {
        local_costmap_display->subProp("Topic")->setValue("/move_base/local_costmap/costmap");
        local_costmap_display->subProp("Color Scheme")->setValue("raw");
        local_costmap_display->subProp("Draw Behind")->setValue(false);
        local_costmap_display->subProp("Alpha")->setValue(0.4);
    }
}

void RVizView::setRobotModel(const QString& model)
{
    if (robot_model_display_) {
        robot_model_display_->subProp("Robot Description")->setValue(model);
    }
}

void RVizView::setFixedFrame(const QString& frame)
{
    if (manager_) {
        manager_->setFixedFrame(frame);
    }
}

void RVizView::setTargetFrame(const QString& frame)
{
    if (manager_) {
        // 设置目标帧
        if (robot_model_display_) {
            robot_model_display_->subProp("Target Frame")->setValue(frame);
        }
    }
}

void RVizView::setDisplayEnabled(const QString& display_name, bool enabled)
{
    rviz::Display* display = nullptr;
    
    if (display_name == "Grid") {
        display = grid_display_;
    } else if (display_name == "RobotModel") {
        display = robot_model_display_;
    } else if (display_name == "Map") {
        display = map_display_;
    } else if (display_name == "LaserScan") {
        display = laser_scan_display_;
    } else if (display_name == "GlobalPath") {
        display = global_path_display_;
    } else if (display_name == "LocalPath") {
        display = local_path_display_;
    } else if (display_name == "Goal") {
        display = goal_display_;
    }
    
    if (display) {
        display->setEnabled(enabled);
    }
}

void RVizView::updateMap(const nav_msgs::OccupancyGridConstPtr& map)
{
    QMutexLocker locker(&mutex_);
    if (map_display_) {
        map_display_->setEnabled(true);
    }
}

void RVizView::updateRobotPose(const nav_msgs::OdometryConstPtr& odom)
{
    QMutexLocker locker(&mutex_);
    if (robot_model_display_) {
        robot_model_display_->setEnabled(true);
    }
}

void RVizView::updateLaserScan(const sensor_msgs::LaserScanConstPtr& scan)
{
    QMutexLocker locker(&mutex_);
    if (laser_scan_display_) {
        laser_scan_display_->setEnabled(true);
    }
}

void RVizView::updatePath(const std::vector<geometry_msgs::PoseStamped>& poses)
{
    QMutexLocker locker(&mutex_);
    if (global_path_display_) {
        global_path_display_->setEnabled(!poses.empty());
    }
    if (local_path_display_) {
        local_path_display_->setEnabled(!poses.empty());
    }
}

void RVizView::onGoalToolActivated()
{
    if (!manager_) {
        ROS_ERROR("Visualization manager not initialized");
        return;
    }

    auto* tool_manager = manager_->getToolManager();
    if (!tool_manager) {
        ROS_ERROR("Tool manager is null");
        return;
    }

    // 查找或创建目标点工具
    rviz::Tool* goal_tool = nullptr;
    for (int i = 0; i < tool_manager->numTools(); i++) {
        if (tool_manager->getTool(i)->getClassId() == "rviz/SetGoal") {
            goal_tool = tool_manager->getTool(i);
            break;
        }
    }
    
    if (!goal_tool) {
        goal_tool = tool_manager->addTool("rviz/SetGoal");
    }

    if (goal_tool) {
        tool_manager->setCurrentTool(goal_tool);
        ROS_INFO("Goal tool activated");
    } else {
        ROS_ERROR("Failed to activate goal tool");
    }
}

void RVizView::onInitialPoseToolActivated()
{
    if (!manager_ || !initial_pose_tool_) {
        ROS_ERROR("Tool manager or initial pose tool not initialized");
        return;
    }

    auto* tool_manager = manager_->getToolManager();
    if (!tool_manager) {
        ROS_ERROR("Tool manager is null");
        return;
    }

    // 切换到初始位姿工具
    tool_manager->setCurrentTool(initial_pose_tool_);
    ROS_INFO("Initial pose tool activated");
}

void RVizView::onGoalPositionSelected(const Ogre::Vector3& position, const Ogre::Quaternion& orientation)
{
    ROS_INFO("Goal position selected: x=%.2f, y=%.2f", position.x, position.y);
    
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = manager_->getFixedFrame().toStdString();
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = position.x;
    goal.pose.position.y = position.y;
    goal.pose.position.z = position.z;
    goal.pose.orientation.x = orientation.x;
    goal.pose.orientation.y = orientation.y;
    goal.pose.orientation.z = orientation.z;
    goal.pose.orientation.w = orientation.w;

    if (goal_display_) {
        goal_display_->setEnabled(true);
    }

    emit goalSelected(goal);
    ROS_INFO("Goal selected and emitted");

    // 切换回移动相机工具
    if (manager_ && manager_->getToolManager()) {
        auto* tool_manager = manager_->getToolManager();
        // 查找已存在的移动相机工具
        rviz::Tool* move_camera_tool = nullptr;
        for (int i = 0; i < tool_manager->numTools(); i++) {
            if (tool_manager->getTool(i)->getClassId() == "rviz/MoveCamera") {
                move_camera_tool = tool_manager->getTool(i);
                break;
            }
        }
        // 如果没找到，才创建新的
        if (!move_camera_tool) {
            move_camera_tool = tool_manager->addTool("rviz/MoveCamera");
        }
        if (move_camera_tool) {
            tool_manager->setCurrentTool(move_camera_tool);
            ROS_INFO("Switched back to MoveCamera tool after goal selection");
        }
    }
}

void RVizView::onInitialPoseSelected(const Ogre::Vector3& position, const Ogre::Quaternion& orientation)
{
    ROS_INFO("Initial pose selected: x=%.2f, y=%.2f", position.x, position.y);
    
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.pose.pose.position.x = position.x;
    pose.pose.pose.position.y = position.y;
    pose.pose.pose.position.z = 0.0;
    pose.pose.pose.orientation.x = orientation.x;
    pose.pose.pose.orientation.y = orientation.y;
    pose.pose.pose.orientation.z = orientation.z;
    pose.pose.pose.orientation.w = orientation.w;

    // 设置协方差
    for (int i = 0; i < 36; ++i) {
        pose.pose.covariance[i] = 0.0;
    }
    pose.pose.covariance[0] = 0.25;   // x
    pose.pose.covariance[7] = 0.25;   // y
    pose.pose.covariance[35] = 0.068; // yaw

    // 发送初始位姿
    emit initialPoseSelected(pose);
    ROS_INFO("Initial pose emitted");

    // 切换回移动相机工具
    if (manager_ && manager_->getToolManager()) {
        auto* tool_manager = manager_->getToolManager();
        // 查找已存在的移动相机工具
        rviz::Tool* move_camera_tool = nullptr;
        for (int i = 0; i < tool_manager->numTools(); i++) {
            if (tool_manager->getTool(i)->getClassId() == "rviz/MoveCamera") {
                move_camera_tool = tool_manager->getTool(i);
                break;
            }
        }
        // 如果没找到，才创建新的
        if (!move_camera_tool) {
            move_camera_tool = tool_manager->addTool("rviz/MoveCamera");
        }
        if (move_camera_tool) {
            tool_manager->setCurrentTool(move_camera_tool);
            ROS_INFO("Switched back to MoveCamera tool");
        }
    }
}

void RVizView::paintEvent(QPaintEvent* event)
{
    QWidget::paintEvent(event);
    if (status_label_ && status_label_->isVisible()) {
        QPainter painter(this);
        painter.fillRect(rect(), QColor(0, 0, 0, 128));
    }
} 