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
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QTimer>
#include <memory>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/BatteryState.h>
#include <diagnostic_msgs/DiagnosticArray.h>

class RobotController;
class JoystickWidget;
class SpeedDashboard;
class QCamera;
class QCameraViewfinder;

class ControlPanel : public QWidget {
    Q_OBJECT

public:
    explicit ControlPanel(const std::shared_ptr<RobotController>& robot_controller,
                         QWidget* parent = nullptr);
    ~ControlPanel() override;

private slots:
    void onEmergencyStop();
    void updateBatteryStatus(const sensor_msgs::BatteryState& status);
    void updateDiagnostics(const diagnostic_msgs::DiagnosticArray& diagnostics);
    void updateCameraImage(const sensor_msgs::Image& image);
    void updateSpeedDisplay(double linear, double angular);
    void toggleCamera();
    void onJoystickMoved(double x, double y);

private:
    void setupUi();
    void setupCamera();
    void setupJoystick();
    void connectSignals();
    void updateRobotStatus();
    void setStatusWarning(QLabel* label, const QString& text, bool is_normal);

    class ControlPanelPrivate {
    public:
        // 布局
        QVBoxLayout* main_layout{nullptr};
        
        // 相机组件
        QGroupBox* camera_group{nullptr};
        QCameraViewfinder* viewfinder{nullptr};
        QPushButton* camera_toggle_button{nullptr};
        QCamera* camera{nullptr};
        
        // 状态显示
        QGroupBox* status_group{nullptr};
        QLabel* battery_label{nullptr};
        QLabel* voltage_label{nullptr};
        QLabel* current_label{nullptr};
        QLabel* temperature_label{nullptr};
        QLabel* motor_status_label{nullptr};
        QLabel* connection_status_label{nullptr};
        
        // 速度控制
        QGroupBox* speed_control_group{nullptr};
        std::shared_ptr<JoystickWidget> joystick{nullptr};
        std::shared_ptr<SpeedDashboard> speed_dashboard{nullptr};
        QPushButton* emergency_stop_button{nullptr};
        
        // 机器人控制器
        std::shared_ptr<RobotController> robot_controller;
        
        // 速度限制
        double max_linear_speed{1.0};  // m/s
        double max_angular_speed{2.0};  // rad/s
        
        // 状态更新定时器
        QTimer* status_timer{nullptr};
    };

    std::unique_ptr<ControlPanelPrivate> d_ptr;
}; 