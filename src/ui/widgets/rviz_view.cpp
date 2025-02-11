/**
 * Copyright (c) 2024 JIAlonglong
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * @file rviz_view.cpp
 * @brief RViz可视化组件的实现,用于显示机器人的3D模型、地图、传感器数据等
 * @author JIAlonglong
 */

#include "rviz_view.h"
#include <QApplication>
#include <QButtonGroup>
#include <QDebug>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGroupBox>
#include <QIcon>
#include <QMenu>
#include <QMutexLocker>
#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QOpenGLWidget>
#include <QOpenGLWindow>
#include <QPainter>
#include <QSurfaceFormat>
#include <QThread>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>
#include <QVariant>
#include <QVector3D>
#include <QWindow>
#include <stdexcept>

#include <rviz/display.h>
#include <rviz/display_group.h>
#include <rviz/properties/property.h>
#include <rviz/properties/property_tree_model.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/render_panel.h>
#include <rviz/tool.h>
#include <rviz/tool_manager.h>
#include <rviz/view_manager.h>
#include <rviz/visualization_manager.h>
#include <tf2/utils.h>

// 私有实现类
class Private
{
public:
    rviz::RenderPanel*          render_panel_;
    rviz::VisualizationManager* manager_;
    rviz::ToolManager*          tool_manager_;
    rviz::DisplayGroup*         display_group_;

    rviz::Tool* move_camera_tool_;
    rviz::Tool* goal_tool_;
    rviz::Tool* initial_pose_tool_;
    rviz::Tool* select_tool_;
    rviz::Tool* interact_tool_;
    rviz::Tool* measure_tool_;
    rviz::Tool* focus_camera_tool_;
    rviz::Tool* point_tool_;

    rviz::Display* global_path_display_;
    rviz::Display* local_path_display_;
    rviz::Display* goal_display_;

    QLabel*                          status_label_;
    std::shared_ptr<RobotController> robot_controller_;
    rviz::Display*                   robot_model_display_;
    rviz::Display*                   map_display_;
    rviz::Display*                   laser_scan_display_;
    rviz::Display*                   pose_estimate_display_;
    rviz::Display*                   global_costmap_display_;
    rviz::Display*                   local_costmap_display_;
    QVBoxLayout*                     main_layout_;
    QToolBar*                        toolbar_;
    QActionGroup*                    tool_group_;
    QTreeWidget*                     display_tree_;

    Private()
        : render_panel_(nullptr)
        , manager_(nullptr)
        , tool_manager_(nullptr)
        , display_group_(nullptr)
        , move_camera_tool_(nullptr)
        , goal_tool_(nullptr)
        , initial_pose_tool_(nullptr)
        , select_tool_(nullptr)
        , interact_tool_(nullptr)
        , measure_tool_(nullptr)
        , focus_camera_tool_(nullptr)
        , point_tool_(nullptr)
        , global_path_display_(nullptr)
        , local_path_display_(nullptr)
        , goal_display_(nullptr)
        , status_label_(nullptr)
        , robot_controller_(nullptr)
        , robot_model_display_(nullptr)
        , map_display_(nullptr)
        , laser_scan_display_(nullptr)
        , pose_estimate_display_(nullptr)
        , global_costmap_display_(nullptr)
        , local_costmap_display_(nullptr)
        , main_layout_(nullptr)
        , toolbar_(nullptr)
        , tool_group_(nullptr)
        , display_tree_(nullptr)
    {
    }

    ~Private()
    {
        if (manager_) {
            manager_->removeAllDisplays();
            delete manager_;
            manager_ = nullptr;
        }
        if (render_panel_) {
            delete render_panel_;
            render_panel_ = nullptr;
        }
    }
};

RVizView::RVizView(QWidget* parent) : QWidget(parent), d_(std::make_unique<Private>())
{
    ROS_INFO("创建RVizView...");

    // 设置基础的OpenGL环境变量
    qputenv("LIBGL_ALWAYS_SOFTWARE", "1");
    qputenv("MESA_GL_VERSION_OVERRIDE", "2.1");
    qputenv("MESA_GLSL_VERSION_OVERRIDE", "120");
    qputenv("LIBGL_DRI3_DISABLE", "1");
    qputenv("LIBGL_SWRAST_NO_ASYNC", "1");
    qputenv("OGRE_RTT_MODE", "Copy");
    qputenv("OGRE_THREAD_SUPPORT", "0");

    // 设置最基础的OpenGL格式
    QSurfaceFormat format;
    format.setRenderableType(QSurfaceFormat::OpenGL);
    format.setVersion(2, 1);
    format.setProfile(QSurfaceFormat::NoProfile);
    format.setOption(QSurfaceFormat::DeprecatedFunctions);
    format.setDepthBufferSize(8);
    format.setStencilBufferSize(0);
    format.setSamples(0);
    format.setSwapBehavior(QSurfaceFormat::SingleBuffer);
    format.setRedBufferSize(8);
    format.setGreenBufferSize(8);
    format.setBlueBufferSize(8);
    format.setAlphaBufferSize(0);
    QSurfaceFormat::setDefaultFormat(format);

    // 设置窗口属性
    setAttribute(Qt::WA_OpaquePaintEvent);
    setAttribute(Qt::WA_NoSystemBackground);
    setAutoFillBackground(false);

    // 创建主布局
    d_->main_layout_ = new QVBoxLayout(this);
    d_->main_layout_->setContentsMargins(0, 0, 0, 0);
    d_->main_layout_->setSpacing(0);

    // 创建状态标签
    d_->status_label_ = new QLabel(this);
    d_->status_label_->setAlignment(Qt::AlignCenter);
    d_->status_label_->setText(tr("正在初始化RViz..."));
    d_->status_label_->setStyleSheet(
        "QLabel { background-color: rgba(0, 0, 0, 128); color: white; }");
    d_->status_label_->hide();
    d_->main_layout_->addWidget(d_->status_label_);

    // 设置大小策略
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setMinimumSize(600, 400);

    // 设置焦点策略
    setFocusPolicy(Qt::StrongFocus);
}

RVizView::~RVizView()
{
    cleanup();
}

bool RVizView::initializeOpenGL()
{
    return true;  // 简化OpenGL初始化
}

void RVizView::setupUi()
{
    try {
        // 创建渲染面板
        d_->render_panel_ = new rviz::RenderPanel();
        if (!d_->render_panel_) {
            throw std::runtime_error("渲染面板创建失败");
        }

        // 将渲染面板添加到布局之前先检查
        if (!d_->main_layout_) {
            throw std::runtime_error("主布局未初始化");
        }
        d_->main_layout_->addWidget(d_->render_panel_);

        // 创建可视化管理器
        d_->manager_ = new rviz::VisualizationManager(d_->render_panel_);
        if (!d_->manager_) {
            throw std::runtime_error("可视化管理器创建失败");
        }

        // 初始化渲染面板
        try {
            d_->render_panel_->initialize(d_->manager_->getSceneManager(), d_->manager_);
        } catch (const std::exception& e) {
            throw std::runtime_error(std::string("渲染面板初始化失败: ") + e.what());
        }

        // 初始化管理器
        try {
            d_->manager_->initialize();
            d_->manager_->startUpdate();
        } catch (const std::exception& e) {
            throw std::runtime_error(std::string("管理器初始化失败: ") + e.what());
        }

        // 设置固定坐标系
        d_->manager_->setFixedFrame("map");

        // 使用最简单的视图控制器
        d_->manager_->getViewManager()->setCurrentViewControllerType("rviz/TopDownOrtho");

    } catch (const std::exception& e) {
        ROS_ERROR("初始化失败: %s", e.what());
        if (d_->status_label_) {
            d_->status_label_->setText(tr("初始化失败: %1").arg(e.what()));
        }
        throw;
    }
}

void RVizView::initialize()
{
    try {
        ROS_INFO("开始初始化RVizView...");

        // 首先设置UI
        ROS_INFO("设置RViz UI...");
        setupUi();

        // 确保渲染面板已创建并初始化
        if (!d_->render_panel_ || !d_->manager_) {
            ROS_ERROR("渲染面板或可视化管理器未正确初始化");
            throw std::runtime_error("组件初始化失败");
        }

        // 创建工具管理器
        ROS_INFO("创建RViz工具管理器...");
        d_->tool_manager_ = d_->manager_->getToolManager();
        if (!d_->tool_manager_) {
            throw std::runtime_error("无法创建工具管理器");
        }

        // 创建显示组
        ROS_INFO("创建RViz显示组...");
        d_->display_group_ = d_->manager_->getRootDisplayGroup();
        if (!d_->display_group_) {
            throw std::runtime_error("无法创建显示组");
        }

        // 创建工具
        ROS_INFO("创建RViz工具...");
        createTools();

        // 创建显示
        ROS_INFO("创建RViz显示项...");
        createDisplays();

        // 启动更新
        ROS_INFO("启动RViz更新...");
        d_->manager_->startUpdate();

        // 更新相机
        ROS_INFO("更新RViz相机视角...");
        updateCamera();

        // 更新显示状态
        if (d_->status_label_) {
            d_->status_label_->setText(tr("RViz初始化完成"));
            d_->status_label_->hide();
        }

        ROS_INFO("RVizView初始化完成");
        Q_EMIT initializationSucceeded();

    } catch (const std::exception& e) {
        ROS_ERROR("RVizView初始化失败: %s", e.what());
        if (d_->status_label_) {
            d_->status_label_->setText(tr("初始化失败: %1").arg(e.what()));
            d_->status_label_->show();
        }
        Q_EMIT initializationFailed(QString::fromUtf8(e.what()));
    }
}

void RVizView::createDisplays()
{
    if (!d_->manager_ || !d_->display_group_) {
        return;
    }

    // 创建机器人模型显示
    d_->robot_model_display_ = d_->manager_->createDisplay("rviz/RobotModel", "机器人模型", true);
    if (d_->robot_model_display_) {
        d_->robot_model_display_->subProp("Robot Description")->setValue("robot_description");
    }

    // 创建地图显示
    d_->map_display_ = d_->manager_->createDisplay("rviz/Map", "地图", true);
    if (d_->map_display_) {
        d_->map_display_->subProp("Topic")->setValue("/map");
    }

    // 创建激光扫描显示
    d_->laser_scan_display_ = d_->manager_->createDisplay("rviz/LaserScan", "激光扫描", true);
    if (d_->laser_scan_display_) {
        d_->laser_scan_display_->subProp("Topic")->setValue("/scan");
    }

    // 创建全局路径显示
    d_->global_path_display_ = d_->manager_->createDisplay("rviz/Path", "全局路径", true);
    if (d_->global_path_display_) {
        d_->global_path_display_->subProp("Topic")->setValue("/move_base/NavfnROS/plan");
        d_->global_path_display_->subProp("Color")->setValue(QColor(0, 255, 0));
    }

    // 创建局部路径显示
    d_->local_path_display_ = d_->manager_->createDisplay("rviz/Path", "局部路径", true);
    if (d_->local_path_display_) {
        d_->local_path_display_->subProp("Topic")->setValue("/move_base/DWAPlannerROS/local_plan");
        d_->local_path_display_->subProp("Color")->setValue(QColor(255, 0, 0));
    }

    // 创建位姿估计显示
    d_->pose_estimate_display_ = d_->manager_->createDisplay("rviz/PoseArray", "位姿估计", true);
    if (d_->pose_estimate_display_) {
        d_->pose_estimate_display_->subProp("Topic")->setValue("/particlecloud");
    }

    // 创建全局代价地图显示
    d_->global_costmap_display_ = d_->manager_->createDisplay("rviz/Map", "全局代价地图", true);
    if (d_->global_costmap_display_) {
        d_->global_costmap_display_->subProp("Topic")->setValue(
            "/move_base/global_costmap/costmap");
        d_->global_costmap_display_->subProp("Color Scheme")->setValue("costmap");
    }

    // 创建局部代价地图显示
    d_->local_costmap_display_ = d_->manager_->createDisplay("rviz/Map", "局部代价地图", true);
    if (d_->local_costmap_display_) {
        d_->local_costmap_display_->subProp("Topic")->setValue("/move_base/local_costmap/costmap");
        d_->local_costmap_display_->subProp("Color Scheme")->setValue("costmap");
    }
}

void RVizView::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event)
    if (!d_->render_panel_) {
        QPainter painter(this);
        painter.fillRect(rect(), Qt::black);
        if (d_->status_label_) {
            d_->status_label_->setGeometry(0, height() / 2 - 20, width(), 40);
            d_->status_label_->show();
        }
    }
}

void RVizView::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    if (d_->render_panel_) {
        d_->render_panel_->resize(size());
    }
    if (d_->status_label_) {
        d_->status_label_->setGeometry(0, height() / 2 - 20, width(), 40);
    }
}

void RVizView::updateDisplays()
{
    if (!d_->manager_ || !d_->display_group_) return;

    try {
        // 更新所有显示组件
        for (int i = 0; i < d_->display_group_->numDisplays(); ++i) {
            if (auto* display = d_->display_group_->getDisplayAt(i)) {
                display->update(0, 0);
            }
        }
    } catch (const std::exception& e) {
        qWarning() << "更新显示组件失败:" << e.what();
    }
}

void RVizView::updateCamera()
{
    if (!d_->manager_) {
        return;
    }

    // 获取视图管理器
    rviz::ViewManager* view_manager = d_->manager_->getViewManager();
    if (!view_manager) {
        return;
    }

    // 获取当前视图
    rviz::ViewController* current_view = view_manager->getCurrent();
    if (!current_view) {
        return;
    }

    // 设置视图属性
    current_view->subProp("X")->setValue(0);
    current_view->subProp("Y")->setValue(0);
    current_view->subProp("Scale")->setValue(50);

    // 设置焦点位置
    if (auto* focal_point = current_view->subProp("Focal Point")) {
        focal_point->subProp("X")->setValue(0.0);
        focal_point->subProp("Y")->setValue(0.0);
        focal_point->subProp("Z")->setValue(0.0);
    }
}

void RVizView::handleMouseEvent(rviz::ViewportMouseEvent& event)
{
    if (!d_->manager_ || !d_->tool_manager_) return;

    try {
        if (auto* current_tool = d_->tool_manager_->getCurrentTool()) {
            current_tool->processMouseEvent(event);
        }
    } catch (const std::exception& e) {
        qWarning() << "处理鼠标事件失败:" << e.what();
    }
}

void RVizView::onGoalToolActivated()
{
    if (d_->goal_tool_) {
        d_->tool_manager_->setCurrentTool(d_->goal_tool_);
        Q_EMIT goalToolStatusChanged(true);
    }
}

void RVizView::onInitialPoseToolActivated()
{
    if (d_->initial_pose_tool_) {
        d_->tool_manager_->setCurrentTool(d_->initial_pose_tool_);
    }
}

void RVizView::onCurrentToolChanged(rviz::Tool* tool)
{
    if (tool == d_->goal_tool_) {
        Q_EMIT goalToolStatusChanged(true);
    } else {
        Q_EMIT goalToolStatusChanged(false);
    }
}

void RVizView::setRobotController(const std::shared_ptr<RobotController>& controller)
{
    d_->robot_controller_ = controller;
}

void RVizView::onGoalPoseSelected(const Ogre::Vector3&    position,
                                  const Ogre::Quaternion& orientation)
{
    if (!d_->robot_controller_) {
        return;
    }

    double yaw =
        tf2::getYaw(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));

    Q_EMIT goalPoseSelected(position.x, position.y, yaw);
}

void RVizView::onInitialPoseSelected(const Ogre::Vector3&    position,
                                     const Ogre::Quaternion& orientation)
{
    if (!d_->robot_controller_) {
        return;
    }

    double yaw =
        tf2::getYaw(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));

    Q_EMIT initialPoseSelected(position.x, position.y, yaw);
}

void RVizView::activateGoalTool()
{
    try {
        if (d_->manager_ && d_->tool_manager_ && d_->goal_tool_) {
            d_->tool_manager_->setCurrentTool(d_->goal_tool_);
            qInfo() << "目标点工具已激活";
        } else {
            qWarning() << QString("激活目标点工具失败: manager=%1, tool_manager=%2, goal_tool=%3")
                              .arg(d_->manager_ ? "valid" : "null")
                              .arg((d_->manager_ && d_->tool_manager_) ? "valid" : "null")
                              .arg(d_->goal_tool_ ? "valid" : "null");
        }
    } catch (const std::exception& e) {
        qWarning() << "激活目标点工具失败:" << e.what();
    }
}

void RVizView::activateInitialPoseTool()
{
    try {
        if (d_->manager_ && d_->tool_manager_ && d_->initial_pose_tool_) {
            d_->tool_manager_->setCurrentTool(d_->initial_pose_tool_);
            qInfo() << "初始位姿工具已激活";
        } else {
            qWarning() << "激活初始位姿工具失败";
        }
    } catch (const std::exception& e) {
        qWarning() << "激活初始位姿工具失败:" << e.what();
    }
}

void RVizView::visualizeFrontier(const geometry_msgs::Point& frontier)
{
    if (!d_->manager_) return;

    // 创建一个Marker用于显示frontier点
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp    = ros::Time::now();
    marker.ns              = "frontiers";
    marker.id              = 0;
    marker.type            = visualization_msgs::Marker::SPHERE;
    marker.action          = visualization_msgs::Marker::ADD;

    marker.pose.position      = frontier;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(5.0);

    // 发布marker
    static ros::NodeHandle nh;
    static ros::Publisher  marker_pub =
        nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    marker_pub.publish(marker);
}

void RVizView::cleanup()
{
    ROS_INFO("开始清理RVizView...");

    // 停止更新
    if (d_->manager_) {
        d_->manager_->stopUpdate();
    }

    // 清理显示组
    if (d_->display_group_) {
        d_->display_group_->removeAllDisplays();
        d_->display_group_ = nullptr;
    }

    // 清理工具管理器
    if (d_->tool_manager_) {
        d_->tool_manager_ = nullptr;
    }

    // 清理渲染面板
    if (d_->render_panel_) {
        delete d_->render_panel_;
        d_->render_panel_ = nullptr;
    }

    // 清理可视化管理器
    if (d_->manager_) {
        delete d_->manager_;
        d_->manager_ = nullptr;
    }

    ROS_INFO("RVizView清理完成");
}

void RVizView::onDisplayTreeItemChanged(QTreeWidgetItem* item, int column)
{
    if (!item || column != 0) {
        return;
    }

    QString text    = item->text(0);
    bool    checked = item->checkState(0) == Qt::Checked;

    if (text == tr("机器人模型")) {
        showRobotModel(checked);
    } else if (text == tr("地图")) {
        showMap(checked);
    } else if (text == tr("激光扫描")) {
        showLaserScan(checked);
    } else if (text == tr("路径")) {
        showPath(checked);
    } else if (text == tr("位姿估计")) {
        showPoseEstimate(checked);
    }
}

void RVizView::showRobotModel(bool show)
{
    if (d_->robot_model_display_) {
        d_->robot_model_display_->setEnabled(show);
    }
}

void RVizView::showMap(bool show)
{
    if (d_->map_display_) {
        d_->map_display_->setEnabled(show);
    }
}

void RVizView::showLaserScan(bool show)
{
    if (d_->laser_scan_display_) {
        d_->laser_scan_display_->setEnabled(show);
    }
}

void RVizView::showPath(bool show)
{
    if (d_->global_path_display_) {
        d_->global_path_display_->setEnabled(show);
    }
    if (d_->local_path_display_) {
        d_->local_path_display_->setEnabled(show);
    }
}

void RVizView::showPoseEstimate(bool show)
{
    if (d_->pose_estimate_display_) {
        d_->pose_estimate_display_->setEnabled(show);
    }
}

void RVizView::createTools()
{
    if (!d_->manager_ || !d_->tool_manager_) {
        return;
    }

    // 创建移动相机工具
    d_->move_camera_tool_ = d_->tool_manager_->addTool("rviz/MoveCamera");
    d_->move_camera_tool_->setName("移动相机");

    // 创建目标点工具
    d_->goal_tool_ = d_->tool_manager_->addTool("rviz/SetGoal");
    d_->goal_tool_->setName("设置目标点");

    // 创建初始位姿工具
    d_->initial_pose_tool_ = d_->tool_manager_->addTool("rviz/SetInitialPose");
    d_->initial_pose_tool_->setName("设置初始位姿");

    // 创建选择工具
    d_->select_tool_ = d_->tool_manager_->addTool("rviz/Select");
    d_->select_tool_->setName("选择");

    // 创建交互工具
    d_->interact_tool_ = d_->tool_manager_->addTool("rviz/Interact");
    d_->interact_tool_->setName("交互");

    // 创建测量工具
    d_->measure_tool_ = d_->tool_manager_->addTool("rviz/Measure");
    d_->measure_tool_->setName("测量");

    // 设置默认工具
    d_->tool_manager_->setCurrentTool(d_->move_camera_tool_);
}

void RVizView::createToolBar()
{
    d_->toolbar_ = new QToolBar(this);
    d_->main_layout_->addWidget(d_->toolbar_);

    // 视角控制
    auto* view_menu = new QMenu(tr("视角"), this);
    view_menu->addAction(tr("俯视图"), this, [this]() {
        if (d_->manager_) {
            auto* view = d_->manager_->getViewManager()->getCurrent();
            if (view) {
                view->subProp("Angle")->setValue(0);
                view->subProp("Distance")->setValue(10);
            }
        }
    });

    view_menu->addAction(tr("侧视图"), this, [this]() {
        if (d_->manager_) {
            auto* view = d_->manager_->getViewManager()->getCurrent();
            if (view) {
                view->subProp("Angle")->setValue(90);
                view->subProp("Distance")->setValue(10);
            }
        }
    });

    view_menu->addAction(tr("跟随机器人"), this, [this]() {
        if (d_->manager_) {
            auto* view = d_->manager_->getViewManager()->getCurrent();
            if (view) {
                view->subProp("Target Frame")->setValue("base_link");
                view->subProp("Distance")->setValue(5);
            }
        }
    });

    auto* view_button = new QToolButton(this);
    view_button->setMenu(view_menu);
    view_button->setPopupMode(QToolButton::InstantPopup);
    view_button->setText(tr("视角"));
    view_button->setToolTip(tr("切换视角"));
    d_->toolbar_->addWidget(view_button);

    d_->toolbar_->addSeparator();

    // 显示选项
    auto* robot_action = d_->toolbar_->addAction(tr("机器人模型"));
    robot_action->setCheckable(true);
    robot_action->setChecked(true);
    connect(robot_action, &QAction::toggled, this, &RVizView::showRobotModel);

    auto* laser_action = d_->toolbar_->addAction(tr("激光扫描"));
    laser_action->setCheckable(true);
    laser_action->setChecked(true);
    connect(laser_action, &QAction::toggled, this, &RVizView::showLaserScan);

    auto* map_action = d_->toolbar_->addAction(tr("地图"));
    map_action->setCheckable(true);
    map_action->setChecked(true);
    connect(map_action, &QAction::toggled, this, &RVizView::showMap);

    auto* path_action = d_->toolbar_->addAction(tr("路径"));
    path_action->setCheckable(true);
    path_action->setChecked(true);
    connect(path_action, &QAction::toggled, this, &RVizView::showPath);

    auto* costmap_action = d_->toolbar_->addAction(tr("代价地图"));
    costmap_action->setCheckable(true);
    costmap_action->setChecked(false);
    connect(costmap_action, &QAction::toggled, this, [this](bool checked) {
        if (d_->global_costmap_display_) {
            d_->global_costmap_display_->setEnabled(checked);
        }
        if (d_->local_costmap_display_) {
            d_->local_costmap_display_->setEnabled(checked);
        }
    });

    d_->toolbar_->addSeparator();

    // 交互工具
    auto* pose_tool = d_->toolbar_->addAction(tr("设置初始位姿"));
    pose_tool->setCheckable(true);
    connect(pose_tool, &QAction::triggered, this, [this]() {
        if (d_->tool_manager_) {
            d_->tool_manager_->setCurrentTool(d_->initial_pose_tool_);
        }
    });

    auto* goal_tool = d_->toolbar_->addAction(tr("设置目标点"));
    goal_tool->setCheckable(true);
    connect(goal_tool, &QAction::triggered, this, [this]() {
        if (d_->tool_manager_) {
            d_->tool_manager_->setCurrentTool(d_->goal_tool_);
        }
    });

    auto* measure_tool = d_->toolbar_->addAction(tr("测量"));
    measure_tool->setCheckable(true);
    connect(measure_tool, &QAction::triggered, this, [this]() {
        if (d_->tool_manager_) {
            d_->tool_manager_->setCurrentTool(d_->measure_tool_);
        }
    });

    // 将所有工具按钮加入按钮组
    d_->tool_group_ = new QButtonGroup(this);
    d_->tool_group_->setExclusive(true);

    for (auto* action : d_->toolbar_->actions()) {
        if (action->isCheckable()) {
            auto* button = qobject_cast<QToolButton*>(d_->toolbar_->widgetForAction(action));
            if (button) {
                d_->tool_group_->addButton(button);
            }
        }
    }
}