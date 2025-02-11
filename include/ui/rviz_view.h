/**
 * @file rviz_view.h
 * @brief RViz可视化界面的封装类
 * 
 * 该类封装了RViz的功能,提供:
 * - 3D场景可视化
 * - 机器人模型显示
 * - 传感器数据显示
 * - 导航和路径规划可视化
 * - 交互式工具
 */

#pragma once

#ifndef ROBOT_CONTROL_GUI_RVIZ_VIEW_H
#define ROBOT_CONTROL_GUI_RVIZ_VIEW_H

#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QOpenGLContext>
#include <QSurfaceFormat>
#include <QToolBar>
#include <QDockWidget>
#include <QTreeWidget>
#include <QCheckBox>
#include <QActionGroup>
#include <QWindow>
#include <QOpenGLWindow>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>

#include <memory>
#include <vector>
#include <stdexcept>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/tool.h>
#include <rviz/view_manager.h>
#include <rviz/properties/property.h>
#include <rviz/properties/property_tree_model.h>
#include <rviz/viewport_mouse_event.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

#include "ros/robot_controller.h"

namespace rviz {
    class VisualizationManager;
    class RenderPanel;
    class Display;
    class Tool;
    class ViewController;
    class ToolManager;
    class DisplayGroup;
}

class RobotController;

class RVizView : public QWidget {
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param parent 父窗口部件
     */
    explicit RVizView(QWidget* parent = nullptr);
    
    /**
     * @brief 析构函数
     */
    ~RVizView() override;

    /**
     * @brief 初始化RViz组件
     */
    void initialize();

    /**
     * @brief 清理资源
     */
    void cleanup();

    void setFixedFrame(const QString& frame);
    void setTargetFrame(const QString& frame);
    void setDisplayEnabled(const QString& display_name, bool enabled);
    void setDisplayProperty(const QString& display_name, const QString& property_name, const QVariant& value);
    void setToolEnabled(const QString& tool_name, bool enabled);
    void setBackgroundColor(const QColor& color);
    void setInteractionMode(int mode);

    // 显示控制方法
    void showRobotModel(bool show);
    void showMap(bool show);
    void showLaserScan(bool show);
    void showPath(bool show);
    void showGoal(bool show);
    void showPoseEstimate(bool show);
    void showGlobalCostmap(bool show);
    void showLocalCostmap(bool show);

    void activateGoalTool();
    void activateInitialPoseTool();
    void visualizeFrontier(const geometry_msgs::Point& frontier);

    void setRobotController(const std::shared_ptr<RobotController>& controller);

public Q_SLOTS:
    void onPoseUpdated(const geometry_msgs::PoseWithCovarianceStamped& pose);
    void onMapUpdated(const nav_msgs::OccupancyGrid& map);
    void onLaserScanUpdated(const sensor_msgs::LaserScan& scan);
    void onPathUpdated(const nav_msgs::Path& path);
    void onGoalToolActivated();
    void onInitialPoseToolActivated();
    void onCurrentToolChanged(rviz::Tool* tool);
    void onDisplayTreeItemChanged(QTreeWidgetItem* item, int column);
    void onGoalPoseSelected(const Ogre::Vector3& position, const Ogre::Quaternion& orientation);
    void onInitialPoseSelected(const Ogre::Vector3& position, const Ogre::Quaternion& orientation);

Q_SIGNALS:
    /**
     * @brief 初始化成功信号
     */
    void initializationSucceeded();

    /**
     * @brief 初始化失败信号
     * @param error 错误信息
     */
    void initializationFailed(const QString& error);

    /**
     * @brief 目标工具状态改变信号
     * @param active 是否激活
     */
    void goalToolStatusChanged(bool active);

    /**
     * @brief 初始位姿已选择信号
     * @param pose 选择的位姿
     */
    void initialPoseSelected(const geometry_msgs::PoseWithCovarianceStamped& pose);

    void goalSelected(const geometry_msgs::PoseStamped& goal);
    void initialPoseToolStatusChanged(bool active);

    void goalPoseSelected(double x, double y, double theta);
    void initialPoseSelected(double x, double y, double theta);

    void initialized();
    void displayAdded(const QString& display_name);
    void displayRemoved(const QString& display_name);
    void displayEnabled(const QString& display_name, bool enabled);
    void displayPropertyChanged(const QString& display_name, const QString& property_name, const QVariant& value);
    void toolEnabled(const QString& tool_name, bool enabled);
    void interactionModeChanged(int mode);

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    bool initializeOpenGL();
    void setupUi();
    void setupDisplays();
    void setupTools();
    void connectSignals();
    void updateFixedFrame();
    void updateRobotModel();
    void updateLaserScan();
    void updateMap();
    void updatePath();
    void updatePose();
    void handleMouseEvent(rviz::ViewportMouseEvent& event);
    void createDisplays();
    void updateDisplays();
    void updateCamera();
    void createTools();
    void createToolBar();

    struct Private {
        rviz::RenderPanel* render_panel_ = nullptr;
        rviz::VisualizationManager* manager_ = nullptr;
        rviz::ToolManager* tool_manager_ = nullptr;
        rviz::Tool* goal_tool_ = nullptr;
        rviz::Tool* initial_pose_tool_ = nullptr;
        rviz::Tool* move_camera_tool_ = nullptr;
        rviz::Tool* select_tool_ = nullptr;
        rviz::Tool* interact_tool_ = nullptr;
        rviz::Tool* measure_tool_ = nullptr;
        rviz::DisplayGroup* display_group_ = nullptr;
        rviz::Display* robot_model_display_ = nullptr;
        rviz::Display* map_display_ = nullptr;
        rviz::Display* laser_scan_display_ = nullptr;
        rviz::Display* global_path_display_ = nullptr;
        rviz::Display* local_path_display_ = nullptr;
        rviz::Display* pose_estimate_display_ = nullptr;
        rviz::Display* global_costmap_display_ = nullptr;
        rviz::Display* local_costmap_display_ = nullptr;
        QLabel* status_label_ = nullptr;
        QVBoxLayout* main_layout_ = nullptr;
        std::shared_ptr<RobotController> robot_controller_;
        QToolBar* toolbar_;
        QButtonGroup* tool_group_;
    };

    std::unique_ptr<Private> d_;
};

#endif // ROBOT_CONTROL_GUI_RVIZ_VIEW_H 