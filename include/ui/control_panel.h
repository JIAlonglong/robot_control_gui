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