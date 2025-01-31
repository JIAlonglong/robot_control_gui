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
    rviz::ToolManager* tool_manager = manager_->getToolManager();
    if (tool_manager) {
        // 添加移动相机工具
        rviz::Tool* move_camera_tool = tool_manager->addTool("rviz/MoveCamera");
        if (move_camera_tool) {
            tool_manager->setCurrentTool(move_camera_tool);
            ROS_INFO("Move camera tool created and set as current");
        }

        // 添加目标点工具
        goal_tool_ = tool_manager->addTool("rviz/SetGoal");
        if (goal_tool_) {
            ROS_INFO("Goal tool created successfully");
            // 不直接发布到move_base_simple/goal,而是通过信号传递给RobotController
            if (goal_tool_->getPropertyContainer()) {
                goal_tool_->getPropertyContainer()->subProp("Topic")->setValue("");
                ROS_INFO("Goal tool topic cleared");
            }
            
            // 连接信号
            connect(goal_tool_, SIGNAL(activated()), this, SLOT(onGoalToolActivated()));
            connect(goal_tool_, SIGNAL(deactivated()), this, SLOT(onGoalToolDeactivated()));
            connect(goal_tool_, SIGNAL(positionSelected(const Ogre::Vector3&, const Ogre::Quaternion&)),
                    this, SLOT(onGoalPositionSelected(const Ogre::Vector3&, const Ogre::Quaternion&)));
            ROS_INFO("Goal tool signals connected");
            
            // 设置为当前工具
            tool_manager->setCurrentTool(goal_tool_);
            ROS_INFO("Goal tool set as current tool");
        }

        // 添加初始位姿工具
        initial_pose_tool_ = tool_manager->addTool("rviz/SetInitialPose");
        if (initial_pose_tool_) {
            ROS_INFO("Initial pose tool created successfully");
            if (initial_pose_tool_->getPropertyContainer()) {
                initial_pose_tool_->getPropertyContainer()->subProp("Topic")->setValue("/initialpose");
                ROS_INFO("Initial pose tool topic set to /initialpose");
            }
            
            // 连接信号
            connect(initial_pose_tool_, SIGNAL(activated()), this, SLOT(onInitialPoseToolActivated()));
            connect(initial_pose_tool_, SIGNAL(positionSelected(const Ogre::Vector3&, const Ogre::Quaternion&)),
                    this, SLOT(onInitialPoseSelected(const Ogre::Vector3&, const Ogre::Quaternion&)));
            ROS_INFO("Initial pose tool signals connected");
        }
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
    if (!manager_) return;

    // 创建显示组
    display_group_ = manager_->getRootDisplayGroup();

    // 添加网格显示
    grid_display_ = manager_->createDisplay("rviz/Grid", "Grid", true);
    if (grid_display_) {
        grid_display_->subProp("Cell Size")->setValue(1.0f);
        grid_display_->subProp("Color")->setValue(QColor(125, 125, 125));
    }

    // 添加机器人模型显示
    robot_model_display_ = manager_->createDisplay("rviz/RobotModel", "RobotModel", true);
    if (robot_model_display_) {
        robot_model_display_->subProp("Robot Description")->setValue("robot_description");
    }

    // 添加地图显示
    map_display_ = manager_->createDisplay("rviz/Map", "Map", true);
    if (map_display_) {
        map_display_->subProp("Topic")->setValue("/map");
        map_display_->subProp("Color Scheme")->setValue("map");
    }

    // 添加激光扫描显示
    laser_scan_display_ = manager_->createDisplay("rviz/LaserScan", "LaserScan", true);
    if (laser_scan_display_) {
        laser_scan_display_->subProp("Topic")->setValue("/scan");
        laser_scan_display_->subProp("Size (m)")->setValue(0.05);
        laser_scan_display_->subProp("Color")->setValue(QColor(255, 0, 0));
    }

    // 添加全局路径显示
    global_path_display_ = manager_->createDisplay("rviz/Path", "GlobalPath", true);
    if (global_path_display_) {
        global_path_display_->subProp("Topic")->setValue("/move_base/NavfnROS/plan");
        global_path_display_->subProp("Color")->setValue(QColor(0, 255, 0));
        global_path_display_->subProp("Line Width")->setValue(0.1);
    }

    // 添加局部路径显示
    local_path_display_ = manager_->createDisplay("rviz/Path", "LocalPath", true);
    if (local_path_display_) {
        local_path_display_->subProp("Topic")->setValue("/move_base/DWAPlannerROS/local_plan");
        local_path_display_->subProp("Color")->setValue(QColor(0, 0, 255));
        local_path_display_->subProp("Line Width")->setValue(0.1);
    }

    // 添加目标点显示
    goal_display_ = manager_->createDisplay("rviz/Pose", "Goal", true);
    if (goal_display_) {
        goal_display_->subProp("Topic")->setValue("/move_base_simple/goal");
        goal_display_->subProp("Color")->setValue(QColor(255, 0, 0));
        goal_display_->subProp("Shaft Length")->setValue(1.0);
        goal_display_->subProp("Head Length")->setValue(0.3);
    }

    ROS_INFO("All displays set up successfully");
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

void RVizView::paintEvent(QPaintEvent* event)
{
    QWidget::paintEvent(event);
    if (status_label_ && status_label_->isVisible()) {
        QPainter painter(this);
        painter.fillRect(rect(), QColor(0, 0, 0, 128));
    }
}

void RVizView::onGoalToolActivated()
{
    ROS_INFO("Goal tool activated in RVizView");
    emit goalToolStatusChanged(true);
}

void RVizView::onGoalToolDeactivated()
{
    ROS_INFO("Goal tool deactivated in RVizView");
    emit goalToolStatusChanged(false);
}

void RVizView::onGoalPositionSelected(const Ogre::Vector3& position, const Ogre::Quaternion& orientation)
{
    ROS_INFO("Goal position selected in RVizView: x=%f, y=%f, z=%f", position.x, position.y, position.z);
    
    if (!robot_controller_) {
        ROS_ERROR("Robot controller not initialized");
        return;
    }
    
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = position.x;
    goal.pose.position.y = position.y;
    goal.pose.position.z = 0.0;  // 将z坐标设置为0
    goal.pose.orientation.x = orientation.x;
    goal.pose.orientation.y = orientation.y;
    goal.pose.orientation.z = orientation.z;
    goal.pose.orientation.w = orientation.w;

    // 发送目标点选择信号
    ROS_INFO("RVizView: Emitting goalSelected signal...");
    robot_controller_->setNavigationGoal(goal);  // 直接调用 RobotController 的方法
    ROS_INFO("RVizView: Navigation goal set");
    
    // 切换回移动相机工具
    if (manager_ && manager_->getToolManager()) {
        rviz::Tool* move_camera = manager_->getToolManager()->getTool(0);
        if (move_camera) {
            manager_->getToolManager()->setCurrentTool(move_camera);
            ROS_INFO("RVizView: Switched back to MoveCamera tool");
        }
    }
}

void RVizView::onInitialPoseToolActivated()
{
    ROS_INFO("Initial pose tool activated in RVizView");
    if (!manager_) {
        ROS_ERROR("Visualization manager not initialized");
        return;
    }

    // 获取工具管理器
    rviz::ToolManager* tool_manager = manager_->getToolManager();
    if (!tool_manager) {
        ROS_ERROR("Tool manager not available");
        return;
    }

    // 查找并激活初始位姿工具
    for (int i = 0; i < tool_manager->numTools(); i++) {
        rviz::Tool* tool = tool_manager->getTool(i);
        if (tool && tool->getClassId() == "rviz/SetInitialPose") {
            tool_manager->setCurrentTool(tool);
            ROS_INFO("Initial pose tool found and activated");
            break;
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
        rviz::Tool* move_camera = manager_->getToolManager()->getTool(0);
        if (move_camera) {
            manager_->getToolManager()->setCurrentTool(move_camera);
            ROS_INFO("RVizView: Switched back to MoveCamera tool");
        }
    }
}

void RVizView::activateGoalTool()
{
    if (manager_ && manager_->getToolManager() && goal_tool_) {
        manager_->getToolManager()->setCurrentTool(goal_tool_);
        ROS_INFO("Goal tool activated");
    } else {
        ROS_ERROR("Failed to activate goal tool: manager=%p, tool_manager=%p, goal_tool=%p",
                  manager_, 
                  manager_ ? manager_->getToolManager() : nullptr,
                  goal_tool_);
    }
}

void RVizView::activateInitialPoseTool()
{
    if (manager_ && manager_->getToolManager() && initial_pose_tool_) {
        manager_->getToolManager()->setCurrentTool(initial_pose_tool_);
        ROS_INFO("Initial pose tool activated");
    } else {
        ROS_ERROR("Failed to activate initial pose tool");
    }
} 