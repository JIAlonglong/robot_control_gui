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
 * @file main.cpp
 * @brief 主程序入口
 * @author JIAlonglong
 */

#include <QApplication>
#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QMessageBox>
#include <QStandardPaths>
#include <QSurfaceFormat>
#include <QTextStream>
#include <QThread>
#include <QTimer>
#include <csignal>
#include <memory>

#include <ros/console.h>
#include <ros/ros.h>

#include "ros/robot_controller.h"
#include "ui/main_window.h"

int main(int argc, char* argv[])
{
    try {
        // 设置OpenGL环境变量
        qputenv("LIBGL_DEBUG", "verbose");
        qputenv("LIBGL_ALWAYS_SOFTWARE", "1");
        qputenv("LIBGL_DRI3_DISABLE", "1");
        qputenv("GALLIUM_DRIVER", "llvmpipe");
        qputenv("MESA_GL_VERSION_OVERRIDE", "2.1");
        qputenv("MESA_GLSL_VERSION_OVERRIDE", "120");
        qputenv("QT_OPENGL", "software");
        qputenv("QT_XCB_FORCE_SOFTWARE_OPENGL", "1");
        qputenv("QT_QUICK_BACKEND", "software");
        qputenv("QT_QUICK_RENDER_LOOP", "basic");

        // 配置OpenGL格式
        QSurfaceFormat format;
        format.setRenderableType(QSurfaceFormat::OpenGL);
        format.setVersion(2, 1);
        format.setProfile(QSurfaceFormat::NoProfile);
        format.setOption(QSurfaceFormat::DeprecatedFunctions);
        format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
        format.setRedBufferSize(8);
        format.setGreenBufferSize(8);
        format.setBlueBufferSize(8);
        format.setAlphaBufferSize(8);
        format.setDepthBufferSize(16);
        format.setStencilBufferSize(0);
        format.setSamples(0);
        QSurfaceFormat::setDefaultFormat(format);

        // 初始化Qt应用
        QApplication app(argc, argv);

        // 设置Qt应用程序属性
        QCoreApplication::setOrganizationName("RobotControlGUI");
        QCoreApplication::setApplicationName("RobotControlGUI");

        // 初始化ROS节点
        ros::init(argc, argv, "robot_control_gui_node");

        // 检查ROS Master连接
        if (!ros::master::check()) {
            QMessageBox::critical(nullptr, QObject::tr("错误"),
                                  QObject::tr("无法连接到ROS Master，请确保roscore正在运行。"));
            return 1;
        }

        // 创建ROS节点句柄
        ros::NodeHandle nh;
        if (!nh.ok()) {
            QMessageBox::critical(nullptr, QObject::tr("错误"),
                                  QObject::tr("ROS节点句柄初始化失败。"));
            return 1;
        }

        // 等待必要的ROS节点和服务就绪
        ROS_INFO("等待必要的ROS节点和服务就绪...");
        ros::Time     start = ros::Time::now();
        ros::Duration timeout(10.0);  // 最多等待10秒
        bool          services_ready = false;

        while (ros::ok() && (ros::Time::now() - start) < timeout) {
            // 检查必要的服务和话题
            bool has_robot_description = ros::param::has("robot_description");
            bool has_move_base         = ros::service::exists("/move_base/make_plan", false);

            // 使用ros::master::getTopics来检查话题
            std::vector<ros::master::TopicInfo> topics;
            ros::master::getTopics(topics);

            bool has_map_server = false;
            bool has_amcl       = false;

            for (const auto& topic : topics) {
                if (topic.name == "/map") {
                    has_map_server = true;
                }
                if (topic.name == "/amcl_pose") {
                    has_amcl = true;
                }
            }

            if (has_robot_description && has_move_base && has_map_server && has_amcl) {
                services_ready = true;
                ROS_INFO("所有必要的ROS节点和服务已就绪");
                break;
            }

            // 输出等待状态
            ROS_INFO_THROTTLE(
                1, "等待ROS节点就绪: robot_description=%d, move_base=%d, map_server=%d, amcl=%d",
                has_robot_description, has_move_base, has_map_server, has_amcl);

            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }

        if (!services_ready) {
            QMessageBox::warning(nullptr, QObject::tr("警告"),
                                 QObject::tr("部分ROS节点或服务未就绪，部分功能可能不可用。"));
        }

        // 创建定时器用于处理ROS消息
        std::unique_ptr<QTimer> ros_timer = std::make_unique<QTimer>();
        if (!ros_timer) {
            QMessageBox::critical(nullptr, QObject::tr("错误"),
                                  QObject::tr("无法创建ROS消息处理定时器。"));
            return 1;
        }

        QObject::connect(ros_timer.get(), &QTimer::timeout, [&nh]() {
            if (!ros::ok()) {
                QCoreApplication::quit();
                return;
            }
            if (nh.ok()) {
                ros::spinOnce();
            }
        });
        ros_timer->start(100);  // 10Hz

        // 创建主窗口
        std::unique_ptr<MainWindow> window;
        try {
            ROS_INFO("正在创建主窗口...");
            window = std::make_unique<MainWindow>();
            if (!window) {
                throw std::runtime_error("主窗口创建失败");
            }
            ROS_INFO("主窗口创建成功");
            window->show();
        } catch (const std::exception& e) {
            ROS_ERROR("主窗口创建失败: %s", e.what());
            QMessageBox::critical(nullptr, QObject::tr("错误"),
                                  QObject::tr("主窗口创建失败: %1").arg(e.what()));
            return 1;
        }

        // 设置信号处理
        signal(SIGINT, [](int sig) {
            ROS_INFO("接收到中断信号，正在关闭...");
            ros::shutdown();
            QCoreApplication::quit();
        });

        // 运行Qt事件循环
        ROS_INFO("启动事件循环");
        int result = app.exec();

        // 清理资源
        ROS_INFO("正在清理资源...");
        window.reset();
        ros_timer.reset();
        ros::shutdown();
        ROS_INFO("程序正常退出");

        return result;

    } catch (const std::exception& e) {
        ROS_ERROR("程序发生异常: %s", e.what());
        QMessageBox::critical(nullptr, QObject::tr("错误"),
                              QObject::tr("程序启动失败: %1").arg(e.what()));
        ros::shutdown();
        return 1;
    }
}