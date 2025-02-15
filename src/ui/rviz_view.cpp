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
    // 创建主布局
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    
    try {
        // 创建 RViz 渲染面板
        render_panel_ = new rviz::RenderPanel();
        layout->addWidget(render_panel_);
        
        // 初始化 VisualizationManager
        manager_ = new rviz::VisualizationManager(render_panel_);
        render_panel_->initialize(manager_->getSceneManager(), manager_);
        
        // 设置固定帧
        manager_->setFixedFrame("map");
        
        // 创建工具管理器
        tool_manager_ = manager_->getToolManager();
        
        // 添加默认工具
        move_camera_tool_ = tool_manager_->addTool("rviz/MoveCamera");
        if (move_camera_tool_) {
            tool_manager_->setCurrentTool(move_camera_tool_);
        }
        
        // 创建显示组
        display_group_ = manager_->getRootDisplayGroup();
        
        // 创建基础显示
        grid_display_ = manager_->createDisplay("rviz/Grid", "Grid", true);
        robot_model_display_ = manager_->createDisplay("rviz/RobotModel", "Robot Model", true);
        map_display_ = manager_->createDisplay("rviz/Map", "Map", true);
        laser_scan_display_ = manager_->createDisplay("rviz/LaserScan", "LaserScan", true);
        global_path_display_ = manager_->createDisplay("rviz/Path", "Global Path", true);
        local_path_display_ = manager_->createDisplay("rviz/Path", "Local Path", true);
        goal_display_ = manager_->createDisplay("rviz/Pose", "Goal", true);
        mapping_display_ = manager_->createDisplay("rviz/Map", "Mapping", false);
        
        // 设置显示属性
        if (grid_display_) {
            grid_display_->subProp("Cell Size")->setValue(1.0f);
            grid_display_->subProp("Color")->setValue(QColor(160, 160, 160));
        }
        
        if (map_display_) {
            map_display_->subProp("Topic")->setValue("/map");
            map_display_->subProp("Color Scheme")->setValue("map");
        }
        
        if (laser_scan_display_) {
            laser_scan_display_->subProp("Topic")->setValue("/scan");
            laser_scan_display_->subProp("Size (m)")->setValue(0.05);
            laser_scan_display_->subProp("Color")->setValue(QColor(255, 0, 0));
        }
        
        if (robot_model_display_) {
            robot_model_display_->subProp("Robot Description")->setValue("robot_description");
        }
        
        if (global_path_display_) {
            global_path_display_->subProp("Topic")->setValue("/move_base/GlobalPlanner/plan");
            global_path_display_->subProp("Color")->setValue(QColor(0, 255, 0));
        }
        
        if (local_path_display_) {
            local_path_display_->subProp("Topic")->setValue("/move_base/LocalPlanner/plan");
            local_path_display_->subProp("Color")->setValue(QColor(255, 255, 0));
        }
        
        if (mapping_display_) {
            mapping_display_->subProp("Topic")->setValue("/map");
            mapping_display_->subProp("Color Scheme")->setValue("map");
        }
        
        // 完成初始化
        manager_->initialize();
        manager_->startUpdate();
        
        ROS_INFO("RVizView initialized successfully");
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Failed to initialize RVizView: " << e.what());
        cleanup();
    }
}

RVizView::~RVizView()
{
    cleanup();
}

void RVizView::cleanup()
{
    if (manager_) {
        manager_->stopUpdate();
        
        // 清理显示组件
        if (display_group_) {
            if (grid_display_) {
                grid_display_->setEnabled(false);
                delete grid_display_;
                grid_display_ = nullptr;
            }
            if (robot_model_display_) {
                robot_model_display_->setEnabled(false);
                delete robot_model_display_;
                robot_model_display_ = nullptr;
            }
            if (map_display_) {
                map_display_->setEnabled(false);
                delete map_display_;
                map_display_ = nullptr;
            }
            if (laser_scan_display_) {
                laser_scan_display_->setEnabled(false);
                delete laser_scan_display_;
                laser_scan_display_ = nullptr;
            }
            if (global_path_display_) {
                global_path_display_->setEnabled(false);
                delete global_path_display_;
                global_path_display_ = nullptr;
            }
            if (local_path_display_) {
                local_path_display_->setEnabled(false);
                delete local_path_display_;
                local_path_display_ = nullptr;
            }
            if (goal_display_) {
                goal_display_->setEnabled(false);
                delete goal_display_;
                goal_display_ = nullptr;
            }
            if (mapping_display_) {
                mapping_display_->setEnabled(false);
                delete mapping_display_;
                mapping_display_ = nullptr;
            }
        }
        
        // 清理工具
        if (tool_manager_) {
            tool_manager_->removeAll();
            tool_manager_ = nullptr;
        }
        
        // 清理显示组
        display_group_ = nullptr;  // 不需要删除，它由 manager_ 管理
        
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
    } else if (display_name == "Mapping") {
        display = mapping_display_;
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

void RVizView::showMappingView(bool show)
{
    is_mapping_ = show;
    
    if (mapping_display_) {
        mapping_display_->setEnabled(show);
        if (show) {
            // 确保订阅正确的话题
            mapping_display_->subProp("Topic")->setValue("/map");
            mapping_display_->subProp("Update Topic")->setValue(true);
            mapping_display_->subProp("Color Scheme")->setValue("map");
            
            // 强制刷新显示
            mapping_display_->reset();
        }
    }
    
    // 隐藏导航相关的显示
    if (global_path_display_) {
        global_path_display_->setEnabled(!show);
    }
    if (local_path_display_) {
        local_path_display_->setEnabled(!show);
    }
    if (goal_display_) {
        goal_display_->setEnabled(!show);
    }
    
    ROS_INFO("Mapping view %s", show ? "enabled" : "disabled");
}

void RVizView::updateMappingDisplay(const nav_msgs::OccupancyGrid& map)
{
    if (!is_mapping_ || !mapping_display_) {
        ROS_WARN_THROTTLE(1.0, "Mapping display not ready");
        return;
    }
    
    try {
        // 确保地图显示已启用
        mapping_display_->setEnabled(true);
        
        // 设置地图话题
        mapping_display_->subProp("Topic")->setValue("/map");
        mapping_display_->subProp("Update Topic")->setValue(true);
        
        // 强制刷新显示
        QMetaObject::invokeMethod(mapping_display_, "reset", Qt::QueuedConnection);
        
        ROS_INFO_THROTTLE(1.0, "Map display updated: %dx%d", map.info.width, map.info.height);
    } catch (const std::exception& e) {
        ROS_ERROR("Error updating mapping display: %s", e.what());
    }
}

void RVizView::setupDisplays()
{
    if (!manager_) return;

    // 创建基础显示
    grid_display_ = manager_->createDisplay("rviz/Grid", "Grid", true);
    robot_model_display_ = manager_->createDisplay("rviz/RobotModel", "Robot Model", true);
    map_display_ = manager_->createDisplay("rviz/Map", "Map", true);
    laser_scan_display_ = manager_->createDisplay("rviz/LaserScan", "LaserScan", true);
    global_path_display_ = manager_->createDisplay("rviz/Path", "Global Path", true);
    local_path_display_ = manager_->createDisplay("rviz/Path", "Local Path", true);
    goal_display_ = manager_->createDisplay("rviz/Pose", "Goal", true);
    mapping_display_ = manager_->createDisplay("rviz/Map", "Mapping", false);

    // 设置显示属性
    if (grid_display_) {
        grid_display_->subProp("Cell Size")->setValue(1.0f);
        grid_display_->subProp("Color")->setValue(QColor(160, 160, 160));
    }

    if (map_display_) {
        map_display_->subProp("Topic")->setValue("/map");
        map_display_->subProp("Color Scheme")->setValue("map");
    }

    if (laser_scan_display_) {
        laser_scan_display_->subProp("Topic")->setValue("/scan");
        laser_scan_display_->subProp("Size (m)")->setValue(0.05);
        laser_scan_display_->subProp("Color")->setValue(QColor(255, 0, 0));
    }

    if (robot_model_display_) {
        robot_model_display_->subProp("Robot Description")->setValue("robot_description");
    }

    if (global_path_display_) {
        global_path_display_->subProp("Topic")->setValue("/move_base/GlobalPlanner/plan");
        global_path_display_->subProp("Color")->setValue(QColor(0, 255, 0));
    }

    if (local_path_display_) {
        local_path_display_->subProp("Topic")->setValue("/move_base/LocalPlanner/plan");
        local_path_display_->subProp("Color")->setValue(QColor(255, 255, 0));
    }

    if (mapping_display_) {
        mapping_display_->subProp("Topic")->setValue("/map");
        mapping_display_->subProp("Color Scheme")->setValue("map");
    }
}

void RVizView::setupTools()
{
    if (!manager_) {
        ROS_ERROR("Manager not initialized");
        return;
    }

    tool_manager_ = manager_->getToolManager();
    if (!tool_manager_) {
        ROS_ERROR("Failed to get tool manager");
        return;
    }

    try {
        // 添加工具
        move_camera_tool_ = tool_manager_->addTool("rviz/MoveCamera");
        initial_pose_tool_ = tool_manager_->addTool("rviz/SetInitialPose");
        goal_tool_ = tool_manager_->addTool("rviz/SetGoal");

        // 设置工具属性和连接信号
        if (initial_pose_tool_) {
            // 设置初始位姿工具的属性
            auto* property_container = initial_pose_tool_->getPropertyContainer();
            if (property_container) {
                auto* topic_property = property_container->childAt(0);
                if (topic_property) {
                    topic_property->setValue("/initialpose");
                }
            }

            // 连接信号
            connect(initial_pose_tool_, SIGNAL(poseSet(const Ogre::Vector3&, const Ogre::Quaternion&)),
                    this, SLOT(onInitialPoseSelected(const Ogre::Vector3&, const Ogre::Quaternion&)));
            ROS_INFO("Initial pose tool created and connected");
        } else {
            ROS_ERROR("Failed to create initial pose tool");
        }
        
        if (goal_tool_) {
            // 设置目标点工具的属性
            auto* property_container = goal_tool_->getPropertyContainer();
            if (property_container) {
                auto* topic_property = property_container->childAt(0);
                if (topic_property) {
                    topic_property->setValue("/move_base_simple/goal");
                }
            }

            // 连接信号
            connect(goal_tool_, SIGNAL(poseSet(const Ogre::Vector3&, const Ogre::Quaternion&)),
                    this, SLOT(onGoalPositionSelected(const Ogre::Vector3&, const Ogre::Quaternion&)));
            ROS_INFO("Goal tool created and connected");
        } else {
            ROS_ERROR("Failed to create goal tool");
        }

        // 设置默认工具
        if (move_camera_tool_) {
            tool_manager_->setCurrentTool(move_camera_tool_);
            ROS_INFO("Move camera tool set as default");
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Error in setupTools: %s", e.what());
    }
}

void RVizView::onGoalPositionSelected(const Ogre::Vector3& position, const Ogre::Quaternion& orientation)
{
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    
    // 转换位置
    goal.pose.position.x = position.x;
    goal.pose.position.y = position.y;
    goal.pose.position.z = position.z;
    
    // 转换方向
    goal.pose.orientation.x = orientation.x;
    goal.pose.orientation.y = orientation.y;
    goal.pose.orientation.z = orientation.z;
    goal.pose.orientation.w = orientation.w;
    
    // 发送目标点信号
    emit goalSelected(goal);
    
    // 切换回移动相机工具
    if (move_camera_tool_) {
        tool_manager_->setCurrentTool(move_camera_tool_);
    }
}

void RVizView::onInitialPoseSelected(const Ogre::Vector3& position, const Ogre::Quaternion& orientation)
{
    try {
        ROS_INFO("Initial pose selected: x=%.2f, y=%.2f", position.x, position.y);
        
        geometry_msgs::PoseWithCovarianceStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        
        // 设置位置
        pose.pose.pose.position.x = position.x;
        pose.pose.pose.position.y = position.y;
        pose.pose.pose.position.z = 0.0;
        
        // 设置方向
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

        // 发送初始位姿信号
        emit initialPoseSelected(pose);
        ROS_INFO("Initial pose signal emitted");

        // 切换回移动相机工具
        if (move_camera_tool_) {
            tool_manager_->setCurrentTool(move_camera_tool_);
            ROS_INFO("Switched back to move camera tool");
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Error in onInitialPoseSelected: %s", e.what());
    }
}

void RVizView::onInitialPoseSet()
{
    if (!manager_ || !tool_manager_) {
        ROS_ERROR("Manager or tool manager not initialized");
        return;
    }

    if (!initial_pose_tool_) {
        ROS_ERROR("Initial pose tool not available");
        return;
    }

    try {
        tool_manager_->setCurrentTool(initial_pose_tool_);
        ROS_INFO("Initial pose tool activated");
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to activate initial pose tool: %s", e.what());
    }
}

void RVizView::onGoalSet()
{
    if (!manager_ || !tool_manager_) {
        ROS_ERROR("Manager or tool manager not initialized");
        return;
    }

    if (!goal_tool_) {
        ROS_ERROR("Goal tool not available");
        return;
    }

    try {
        tool_manager_->setCurrentTool(goal_tool_);
        ROS_INFO("Goal tool activated");
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to activate goal tool: %s", e.what());
    }
} 