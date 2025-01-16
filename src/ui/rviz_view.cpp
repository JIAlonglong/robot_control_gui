#include "ui/rviz_view.h"

#include <QVBoxLayout>
#include <QPainter>
#include <QPaintEvent>
#include <QTimer>
#include <QVariant>

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
    goal_tool_ = manager_->getToolManager()->addTool("rviz/SetGoal");
    initial_pose_tool_ = manager_->getToolManager()->addTool("rviz/SetInitialPose");

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
    map_display_->subProp("Color Scheme")->setValue("map");
    map_display_->subProp("Draw Behind")->setValue(true);
    map_display_->subProp("Alpha")->setValue(0.7);
    map_display_->subProp("Update Topic")->setValue("/map_updates");
    map_display_->subProp("Resolution")->setValue(0.05);  // 5cm per pixel

    // 创建激光扫描显示
    laser_scan_display_ = manager_->createDisplay("rviz/LaserScan", "LaserScan", true);
    if (!laser_scan_display_) {
        ROS_ERROR("Failed to create LaserScan display");
        return;
    }
    laser_scan_display_->subProp("Topic")->setValue("/scan");
    laser_scan_display_->subProp("Size (m)")->setValue(0.05);
    laser_scan_display_->subProp("Color")->setValue(QColor(255, 0, 0));

    // 创建路径显示
    path_display_ = manager_->createDisplay("rviz/Path", "Path", true);
    if (!path_display_) {
        ROS_ERROR("Failed to create Path display");
        return;
    }
    path_display_->subProp("Topic")->setValue("/move_base/NavfnROS/plan");
    path_display_->subProp("Color")->setValue(QColor(0, 0, 255));

    // 创建目标点显示
    goal_display_ = manager_->createDisplay("rviz/Pose", "Goal", true);
    if (!goal_display_) {
        ROS_ERROR("Failed to create Goal display");
        return;
    }
}

void RVizView::setRobotModel(const QString& model)
{
    if (robot_model_display_) {
        robot_model_display_->subProp("Robot Description")->setValue(QVariant(model));
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
        manager_->getFrameManager()->setFixedFrame(frame.toStdString());
    }
}

void RVizView::setDisplayEnabled(const QString& display_name, bool enabled)
{
    rviz::Display* display = nullptr;

    if (display_name == "Grid") {
        display = grid_display_;
    } else if (display_name == "Robot Model") {
        display = robot_model_display_;
    } else if (display_name == "Map") {
        display = map_display_;
    } else if (display_name == "LaserScan") {
        display = laser_scan_display_;
    } else if (display_name == "Path") {
        display = path_display_;
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
        // 确保地图显示已启用
        map_display_->setEnabled(true);
        
        // 检查话题是否正确设置
        if (map_display_->subProp("Topic")->getValue().toString() != "/map") {
            map_display_->subProp("Topic")->setValue("/map");
        }
        
        // 强制刷新显示
        map_display_->reset();
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
    if (path_display_) {
        path_display_->setEnabled(!poses.empty());
    }
}

void RVizView::onGoalToolActivated()
{
    if (manager_ && goal_tool_) {
        manager_->getToolManager()->setCurrentTool(goal_tool_);
    }
}

void RVizView::onInitialPoseToolActivated()
{
    if (manager_ && initial_pose_tool_) {
        manager_->getToolManager()->setCurrentTool(initial_pose_tool_);
    }
}

void RVizView::onGoalPositionSelected(const Ogre::Vector3& position, const Ogre::Quaternion& orientation)
{
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

    emit goalSelected(goal);
}

void RVizView::onInitialPoseSelected(const Ogre::Vector3& position, const Ogre::Quaternion& orientation)
{
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = manager_->getFixedFrame().toStdString();
    pose.header.stamp = ros::Time::now();
    pose.pose.pose.position.x = position.x;
    pose.pose.pose.position.y = position.y;
    pose.pose.pose.position.z = position.z;
    pose.pose.pose.orientation.x = orientation.x;
    pose.pose.pose.orientation.y = orientation.y;
    pose.pose.pose.orientation.z = orientation.z;
    pose.pose.pose.orientation.w = orientation.w;

    // 设置协方差矩阵
    for (int i = 0; i < 36; ++i) {
        pose.pose.covariance[i] = 0.0;
    }
    pose.pose.covariance[0] = 0.25;  // x
    pose.pose.covariance[7] = 0.25;  // y
    pose.pose.covariance[35] = 0.06853891945200942;  // yaw

    emit initialPoseSelected(pose);
}

void RVizView::paintEvent(QPaintEvent* event)
{
    QWidget::paintEvent(event);
    if (status_label_ && status_label_->isVisible()) {
        QPainter painter(this);
        painter.fillRect(rect(), QColor(0, 0, 0, 128));
    }
} 