/*
 * Copyright (c) 2025 JIAlonglong
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
 */

#pragma once

#include <QWidget>
#include <QLabel>
#include <QMutex>
#include <memory>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool.h>
#include <rviz/tool_manager.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>
#include <ros/ros.h>
#include "ros/robot_controller.h"

class RVizView : public QWidget {
    Q_OBJECT
public:
    explicit RVizView(QWidget* parent = nullptr);
    ~RVizView() override;

    void setRobotModel(const QString& model);
    void setFixedFrame(const QString& frame);
    void setTargetFrame(const QString& frame);
    void setDisplayEnabled(const QString& display_name, bool enabled);
    void showMappingView(bool show);
    void updateMappingDisplay(const nav_msgs::OccupancyGrid& map);

public slots:
    void updateMap(const nav_msgs::OccupancyGridConstPtr& map);
    void updateRobotPose(const nav_msgs::OdometryConstPtr& odom);
    void updateLaserScan(const sensor_msgs::LaserScanConstPtr& scan);
    void updatePath(const std::vector<geometry_msgs::PoseStamped>& poses);
    void onInitialPoseSet();  // 设置初始位姿
    void onGoalSet();         // 设置目标点

signals:
    void goalSelected(const geometry_msgs::PoseStamped& goal);
    void initialPoseSelected(const geometry_msgs::PoseWithCovarianceStamped& pose);

private slots:
    void onInitialPoseSelected(const Ogre::Vector3& position, const Ogre::Quaternion& orientation);
    void onGoalPositionSelected(const Ogre::Vector3& position, const Ogre::Quaternion& orientation);

private:
    void cleanup();
    void setupDisplays();
    void setupTools();

    QLabel* status_label_{nullptr};
    rviz::RenderPanel* render_panel_{nullptr};
    rviz::VisualizationManager* manager_{nullptr};
    rviz::ToolManager* tool_manager_{nullptr};
    rviz::Tool* move_camera_tool_{nullptr};
    rviz::Tool* initial_pose_tool_{nullptr};
    rviz::Tool* goal_tool_{nullptr};
    rviz::DisplayGroup* display_group_{nullptr};
    rviz::Display* grid_display_{nullptr};
    rviz::Display* robot_model_display_{nullptr};
    rviz::Display* map_display_{nullptr};
    rviz::Display* laser_scan_display_{nullptr};
    rviz::Display* global_path_display_{nullptr};  // 全局路径显示
    rviz::Display* local_path_display_{nullptr};   // 局部路径显示
    rviz::Display* goal_display_{nullptr};
    rviz::Display* mapping_display_{nullptr};
    QMutex mutex_;
    ros::NodeHandle nh_;
    std::shared_ptr<RobotController> robot_controller_;
    bool is_mapping_{false};
}; 