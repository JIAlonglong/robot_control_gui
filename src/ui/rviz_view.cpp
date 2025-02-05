#include "ui/rviz_view.h"

#include <QVBoxLayout>
#include <QPainter>
#include <QPaintEvent>
#include <QTimer>
#include <QVariant>
#include <QThread>
#include <QMutexLocker>

#include <rviz/display.h>
#include <rviz/tool.h>
#include <rviz/tool_manager.h>
#include <rviz/display_group.h>
#include <rviz/properties/property_tree_model.h>
#include <rviz/properties/property.h>
#include <rviz/properties/tf_frame_property.h>

RVizView::RVizView(QWidget* parent)
    : QWidget(parent)
    , render_panel_(nullptr)
    , manager_(nullptr)
    , grid_display_(nullptr)
    , robot_model_display_(nullptr)
    , map_display_(nullptr)
    , laser_scan_display_(nullptr)
    , robot_controller_(nullptr)
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

    // 等待ROS初始化完成
    ros::Duration(0.5).sleep();

    initialize();
}

RVizView::~RVizView()
{
    cleanup();
}

void RVizView::initialize()
{
    try {
        if (!render_panel_) {
            throw std::runtime_error("Render panel is null");
        }

        // 创建可视化管理器
        manager_ = new rviz::VisualizationManager(render_panel_);
        if (!manager_) {
            throw std::runtime_error("Failed to create visualization manager");
        }

        // 初始化渲染面板
        render_panel_->initialize(manager_->getSceneManager(), manager_);

        // 设置固定坐标系
        manager_->setFixedFrame("map");

        // 初始化工具管理器
        manager_->initialize();

        // 等待TF树建立
        ros::Duration(0.1).sleep();

        // 设置工具
        setupTools();

        // 设置显示组件
        setupDisplays();

        // 启动渲染
        manager_->startUpdate();

    } catch (const std::exception& e) {
        ROS_ERROR("Exception in RVizView initialization: %s", e.what());
        cleanup();
        throw;
    }
}

void RVizView::setupDisplays()
{
    if (!manager_) {
        ROS_ERROR("Visualization manager not initialized");
        return;
    }

    try {
        // 创建网格显示
        grid_display_ = manager_->createDisplay("rviz/Grid", "Grid", true);
        if (!grid_display_) {
            ROS_ERROR("Failed to create grid display");
            return;
        }
        grid_display_->setEnabled(true);
        grid_display_->subProp("Cell Size")->setValue(1.0);
        grid_display_->subProp("Color")->setValue(QColor(125, 125, 125));

        // 创建机器人模型显示
        robot_model_display_ = manager_->createDisplay("rviz/RobotModel", "Robot Model", true);
        if (!robot_model_display_) {
            ROS_ERROR("Failed to create robot model display");
            return;
        }
        robot_model_display_->setEnabled(true);

        // 创建地图显示
        map_display_ = manager_->createDisplay("rviz/Map", "Map", true);
        if (!map_display_) {
            ROS_ERROR("Failed to create map display");
            return;
        }
        map_display_->setEnabled(true);
        map_display_->subProp("Topic")->setValue("/map");

        // 创建激光扫描显示
        laser_scan_display_ = manager_->createDisplay("rviz/LaserScan", "LaserScan", true);
        if (!laser_scan_display_) {
            ROS_ERROR("Failed to create laser scan display");
            return;
        }
        laser_scan_display_->setEnabled(true);
        laser_scan_display_->subProp("Topic")->setValue("/scan");
        laser_scan_display_->subProp("Size (m)")->setValue(0.05);
        laser_scan_display_->subProp("Color")->setValue(QColor(255, 0, 0));

        // 等待所有显示组件初始化完成
        ros::Duration(0.1).sleep();

        ROS_INFO("All displays set up successfully");

    } catch (const std::exception& e) {
        ROS_ERROR("Exception in setupDisplays: %s", e.what());
        cleanup();
    }
}

void RVizView::cleanup()
{
    if (manager_) {
        manager_->stopUpdate();
        
        // 禁用所有显示组件
        if (grid_display_) {
            grid_display_->setEnabled(false);
            grid_display_ = nullptr;
        }
        if (robot_model_display_) {
            robot_model_display_->setEnabled(false);
            robot_model_display_ = nullptr;
        }
        if (map_display_) {
            map_display_->setEnabled(false);
            map_display_ = nullptr;
        }
        if (laser_scan_display_) {
            laser_scan_display_->setEnabled(false);
            laser_scan_display_ = nullptr;
        }

        // 删除管理器会自动清理所有显示组件
        delete manager_;
        manager_ = nullptr;
    }

    if (render_panel_) {
        delete render_panel_;
        render_panel_ = nullptr;
    }
}

void RVizView::setRobotModel(const QString& model)
{
    QMutexLocker locker(&mutex_);
    
    if (robot_model_display_) {
        try {
            robot_model_display_->subProp("Robot Description")->setValue(model);
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to set robot model: %s", e.what());
        }
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

void RVizView::setupTools()
{
    if (!manager_) {
        ROS_ERROR("Visualization manager not initialized");
        return;
    }

    try {
        // 创建移动相机工具
        move_camera_tool_ = manager_->getToolManager()->addTool("rviz/MoveCamera");
        if (!move_camera_tool_) {
            ROS_ERROR("Failed to create move camera tool");
            return;
        }
        manager_->getToolManager()->setCurrentTool(move_camera_tool_);
        ROS_INFO("Move camera tool created and set as current");

        // 创建目标点工具
        goal_tool_ = manager_->getToolManager()->addTool("rviz/SetGoal");
        if (!goal_tool_) {
            ROS_ERROR("Failed to create goal tool");
            return;
        }
        ROS_INFO("Goal tool created successfully");

        // 清除目标点工具的话题
        goal_tool_->getPropertyContainer()->subProp("Topic")->setValue("");
        ROS_INFO("Goal tool topic cleared");

        // 创建初始位姿工具
        initial_pose_tool_ = manager_->getToolManager()->addTool("rviz/SetInitialPose");
        if (!initial_pose_tool_) {
            ROS_ERROR("Failed to create initial pose tool");
            return;
        }
        ROS_INFO("Initial pose tool created successfully");

        // 设置初始位姿工具的话题
        initial_pose_tool_->getPropertyContainer()->subProp("Topic")->setValue("/initialpose");
        ROS_INFO("Initial pose tool topic set to /initialpose");

    } catch (const std::exception& e) {
        ROS_ERROR("Exception in setupTools: %s", e.what());
        cleanup();
    }
}

void RVizView::onGoalToolActivated()
{
    ROS_INFO("Goal tool activated in RVizView");
    emit goalToolActivated();
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

void RVizView::onGoalPoseSelected(const Ogre::Vector3& position, const Ogre::Quaternion& orientation)
{
    ROS_INFO("Goal pose selected in RVizView");
    
    QMutexLocker locker(&mutex_);
    if (!robot_controller_) {
        ROS_ERROR("Robot controller not initialized, ignoring goal pose selection");
        return;
    }
    
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    
    // 转换Ogre类型到geometry_msgs类型
    goal.pose.position.x = position.x;
    goal.pose.position.y = position.y;
    goal.pose.position.z = position.z;
    goal.pose.orientation.x = orientation.x;
    goal.pose.orientation.y = orientation.y;
    goal.pose.orientation.z = orientation.z;
    goal.pose.orientation.w = orientation.w;

    try {
        robot_controller_->setNavigationGoal(goal);
        ROS_INFO("Navigation goal set successfully");
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to set navigation goal: %s", e.what());
    }
    
    // 切换回移动相机工具
    if (manager_ && manager_->getToolManager() && move_camera_tool_) {
        manager_->getToolManager()->setCurrentTool(move_camera_tool_);
        ROS_INFO("Switched back to MoveCamera tool");
    }
}

void RVizView::onInitialPoseSelected(const Ogre::Vector3& position, const Ogre::Quaternion& orientation)
{
    ROS_INFO("Initial pose selected in RVizView");
    
    QMutexLocker locker(&mutex_);
    if (!robot_controller_) {
        ROS_ERROR("Robot controller not initialized, ignoring initial pose selection");
        return;
    }

    geometry_msgs::PoseWithCovarianceStamped pose_with_cov;
    pose_with_cov.header.frame_id = "map";
    pose_with_cov.header.stamp = ros::Time::now();
    
    // 转换Ogre类型到geometry_msgs类型
    pose_with_cov.pose.pose.position.x = position.x;
    pose_with_cov.pose.pose.position.y = position.y;
    pose_with_cov.pose.pose.position.z = position.z;
    pose_with_cov.pose.pose.orientation.x = orientation.x;
    pose_with_cov.pose.pose.orientation.y = orientation.y;
    pose_with_cov.pose.pose.orientation.z = orientation.z;
    pose_with_cov.pose.pose.orientation.w = orientation.w;

    // 设置协方差矩阵
    for (int i = 0; i < 36; i++) {
        pose_with_cov.pose.covariance[i] = 0.0;
    }
    // 设置对角线元素
    pose_with_cov.pose.covariance[0] = 0.25;  // x
    pose_with_cov.pose.covariance[7] = 0.25;  // y
    pose_with_cov.pose.covariance[35] = 0.06853891945200942; // yaw

    try {
        robot_controller_->setInitialPose(pose_with_cov);
        ROS_INFO("Initial pose set successfully");
        emit initialPoseSelected(pose_with_cov);
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to set initial pose: %s", e.what());
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

void RVizView::onCurrentToolChanged(rviz::Tool* tool)
{
    if (tool == goal_tool_) {
        onGoalToolActivated();
        emit goalToolStatusChanged(true);
    } else if (tool == move_camera_tool_) {
        // 如果切换到移动相机工具，说明其他工具被停用
        if (manager_->getToolManager()->getCurrentTool() != goal_tool_) {
            emit goalToolStatusChanged(false);
        }
    }
    ROS_INFO("Current tool changed to: %s", tool ? tool->getName().toStdString().c_str() : "null");
}

void RVizView::setRobotController(std::shared_ptr<RobotController> controller)
{
    QMutexLocker locker(&mutex_);
    robot_controller_ = controller;
} 