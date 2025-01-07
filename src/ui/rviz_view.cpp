#include "ui/rviz_view.h"
#include <QVBoxLayout>
#include <QLabel>
#include <QTimer>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/view_manager.h>
#include <rviz/properties/property.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

RVizView::RVizView(QWidget* parent)
    : QWidget(parent)
    , manager_(nullptr)
    , render_panel_(nullptr)
    , grid_display_(nullptr)
    , map_display_(nullptr)
    , robot_model_display_(nullptr)
    , laser_scan_display_(nullptr)
    , path_display_(nullptr)
    , pose_display_(nullptr)
    , status_check_timer_(nullptr)
    , status_label_(nullptr)
    , tf_buffer_(std::make_shared<tf2_ros::Buffer>())
    , tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
    // 创建布局
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    // 创建状态标签
    status_label_ = new QLabel(this);
    status_label_->setStyleSheet("QLabel { color: #666666; background-color: #f0f0f0; padding: 5px; }");
    status_label_->setWordWrap(true);
    status_label_->hide();  // 初始时隐藏
    layout->addWidget(status_label_);

    // 创建RViz渲染面板
    render_panel_ = new rviz::RenderPanel();
    layout->addWidget(render_panel_);

    // 创建可视化管理器
    manager_ = new rviz::VisualizationManager(render_panel_);
    render_panel_->initialize(manager_->getSceneManager(), manager_);
    manager_->initialize();
    manager_->startUpdate();

    // 设置固定坐标系
    manager_->setFixedFrame(QString::fromStdString("map"));

    // 设置显示和工具
    setupDisplays();
    setupTools();

    // 设置默认视图
    setDefaultView();

    // 创建定时器检查机器人模型状态
    status_check_timer_ = new QTimer(this);
    connect(status_check_timer_, &QTimer::timeout, this, &RVizView::checkRobotModelStatus);
    status_check_timer_->start(1000);  // 每秒检查一次
}

RVizView::~RVizView()
{
    if (manager_) {
        delete manager_;
        manager_ = nullptr;
    }
}

void RVizView::setupDisplays()
{
    // 创建显示
    grid_display_ = manager_->createDisplay("rviz/Grid", "Grid", true);
    if (grid_display_) {
        grid_display_->subProp("Line Style")->setValue("Lines");
        grid_display_->subProp("Color")->setValue(QColor(160, 160, 160));
        grid_display_->subProp("Cell Size")->setValue(1.0f);
    }

    map_display_ = manager_->createDisplay("rviz/Map", "Map", true);
    if (map_display_) {
        map_display_->subProp("Topic")->setValue("/map");
    }

    robot_model_display_ = manager_->createDisplay("rviz/RobotModel", "Robot Model", true);
    if (robot_model_display_) {
        robot_model_display_->subProp("Robot Description")->setValue("robot_description");
    }

    laser_scan_display_ = manager_->createDisplay("rviz/LaserScan", "LaserScan", true);
    if (laser_scan_display_) {
        laser_scan_display_->subProp("Topic")->setValue("/scan");
        laser_scan_display_->subProp("Size (m)")->setValue(0.05f);
        laser_scan_display_->subProp("Color")->setValue(QColor(0, 255, 0));
    }

    path_display_ = manager_->createDisplay("rviz/Path", "Path", true);
    if (path_display_) {
        path_display_->subProp("Topic")->setValue("/move_base/NavfnROS/plan");
        path_display_->subProp("Color")->setValue(QColor(0, 0, 255));
    }

    pose_display_ = manager_->createDisplay("rviz/Pose", "Goal", true);
    if (pose_display_) {
        pose_display_->subProp("Topic")->setValue("/move_base_simple/goal");
    }
}

void RVizView::setupTools()
{
    manager_->getToolManager()->addTool("rviz/MoveCamera");
    manager_->getToolManager()->addTool("rviz/SetInitialPose");
    manager_->getToolManager()->addTool("rviz/SetGoal");
    setCurrentTool("rviz/MoveCamera");
}

void RVizView::setDisplayEnabled(const QString& display_name, bool enabled)
{
    rviz::Display* display = nullptr;
    
    if (display_name == "Grid") {
        display = grid_display_;
    } else if (display_name == "Map") {
        display = map_display_;
    } else if (display_name == "Robot Model") {
        display = robot_model_display_;
    } else if (display_name == "LaserScan") {
        display = laser_scan_display_;
    } else if (display_name == "Path") {
        display = path_display_;
    } else if (display_name == "Goal") {
        display = pose_display_;
    }

    if (display) {
        display->setEnabled(enabled);
    }
}

void RVizView::setFixedFrame(const QString& frame)
{
    if (manager_) {
        manager_->setFixedFrame(frame);
    }
}

void RVizView::setCurrentTool(const QString& tool_name)
{
    if (manager_) {
        rviz::Tool* tool = manager_->getToolManager()->addTool(tool_name);
        if (tool) {
            manager_->getToolManager()->setCurrentTool(tool);
        }
    }
}

void RVizView::setDefaultView()
{
    if (manager_ && manager_->getViewManager()) {
        manager_->getViewManager()->setCurrentViewControllerType("rviz/TopDownOrtho");
    }
}

void RVizView::updateMap(const std::shared_ptr<nav_msgs::OccupancyGrid>& map)
{
    // RViz会自动从话题更新地图，不需要手动更新
}

void RVizView::updateRobotPose(const std::shared_ptr<nav_msgs::Odometry>& odom)
{
    // RViz会自动从话题更新机器人位姿，不需要手动更新
}

void RVizView::updateLaserScan(const std::shared_ptr<sensor_msgs::LaserScan>& scan)
{
    // RViz会自动从话题更新激光扫描数据，不需要手动更新
}

void RVizView::updatePath(const std::vector<geometry_msgs::PoseStamped>& poses)
{
    // RViz会自动从话题更新路径，不需要手动更新
}

void RVizView::checkRobotModelStatus()
{
    if (!robot_model_display_) return;

    QString status_text;
    bool has_error = false;

    // 检查robot_description参数
    if (!ros::param::has("robot_description")) {
        status_text += tr("• 未找到robot_description参数\n");
        has_error = true;
    }

    // 检查TF变换
    try {
        bool has_base_footprint = tf_buffer_->canTransform("map", "base_footprint", ros::Time(0), ros::Duration(0.1));
        bool has_base_link = tf_buffer_->canTransform("map", "base_link", ros::Time(0), ros::Duration(0.1));
        
        if (!has_base_footprint && !has_base_link) {
            status_text += tr("• 缺少必要的TF变换（base_footprint或base_link）\n");
            has_error = true;
        }
    } catch (const tf2::TransformException& ex) {
        status_text += tr("• TF变换错误: %1\n").arg(ex.what());
        has_error = true;
    }

    // 检查fixed frame
    QString fixed_frame = manager_->getFixedFrame();
    try {
        if (fixed_frame.isEmpty() || 
            (fixed_frame == "map" && !tf_buffer_->canTransform(fixed_frame.toStdString(), "base_link", ros::Time(0), ros::Duration(0.1)))) {
            status_text += tr("• 固定坐标系（%1）不可用\n").arg(fixed_frame);
            has_error = true;
        }
    } catch (const tf2::TransformException& ex) {
        status_text += tr("• 固定坐标系错误: %1\n").arg(ex.what());
        has_error = true;
    }

    // 显示或隐藏状态标签
    if (has_error) {
        status_text = tr("机器人模型不可见的可能原因：\n") + status_text;
        status_label_->setText(status_text);
        status_label_->show();
    } else {
        status_label_->hide();
    }
} 