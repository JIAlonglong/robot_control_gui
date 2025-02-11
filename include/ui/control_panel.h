#ifndef ROBOT_CONTROL_GUI_CONTROL_PANEL_H
#define ROBOT_CONTROL_GUI_CONTROL_PANEL_H

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QProgressBar>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QMap>
#include <memory>

class RobotController;
class JoystickWidget;
class BatteryIndicator;
class SpeedDisplay;
class ControlPanel;
class QKeyEvent;

// 前向声明完成后再包含完整的头文件
#include "ros/robot_controller.h"
#include "ui/joystick_widget.h"
#include "widgets/battery_indicator.h"
#include "widgets/speed_display.h"

class ControlPanelPrivate {
public:
    explicit ControlPanelPrivate(ControlPanel* q)
        : q_ptr(q)
        , is_controlling_(false)
        , keyboard_linear_speed_(0.2)
        , keyboard_angular_speed_(0.5)
    {
        key_pressed_[Qt::Key_W] = false;
        key_pressed_[Qt::Key_S] = false;
        key_pressed_[Qt::Key_A] = false;
        key_pressed_[Qt::Key_D] = false;
    }

    ControlPanel* q_ptr;
    std::shared_ptr<RobotController> robot_controller;

    // UI组件
    QVBoxLayout* main_layout_{nullptr};
    QGroupBox* joystick_group_{nullptr};
    QGroupBox* battery_group_{nullptr};
    QGroupBox* speed_group_{nullptr};
    QGroupBox* control_group_{nullptr};

    JoystickWidget* joystick_{nullptr};
    BatteryIndicator* battery_indicator_{nullptr};
    SpeedDisplay* speed_display_{nullptr};

    QPushButton* emergency_stop_button_{nullptr};
    QComboBox* control_mode_combo_{nullptr};
    QDoubleSpinBox* speed_limit_spin_{nullptr};
    QDoubleSpinBox* accel_limit_spin_{nullptr};

    QLabel* battery_level_label_{nullptr};
    QLabel* temperature_label_{nullptr};
    QLabel* voltage_label_{nullptr};
    QLabel* current_label_{nullptr};

    // 控制状态
    bool is_controlling_;
    QMap<int, bool> key_pressed_;
    double keyboard_linear_speed_;
    double keyboard_angular_speed_;
};

class ControlPanel : public QWidget {
    Q_OBJECT

public:
    explicit ControlPanel(const std::shared_ptr<RobotController>& controller, QWidget* parent = nullptr);
    ~ControlPanel() override;

    void setRobotController(const std::shared_ptr<RobotController>& controller);
    void handleKeyEvent(QKeyEvent* event, bool pressed);

Q_SIGNALS:
    void velocityChanged(double linear, double angular);
    void emergencyStopTriggered();
    void joystickEnabled(bool enabled);
    void joystickDisabled();

public Q_SLOTS:
    void updateVelocityDisplay(double linear, double angular);
    void updateBatteryState(double percentage, double voltage, double current, double temperature);
    void updateMotorStatus(const QString& status);
    void onJoystickToggled(bool checked);
    void onJoystickMoved();
    void onJoystickReleased();
    void onStopButtonClicked();

private Q_SLOTS:
    void onVelocityChanged();
    void onJoystickStateChanged(bool enabled);
    void onControlModeChanged(int index);
    void onSpeedLimitChanged(double value);
    void onAccelLimitChanged(double value);
    void onKeyboardControl();
    void onEmergencyStop();
    void onMaxSpeedChanged(double value);
    void onMaxAngularSpeedChanged(double value);
    void onMaxAccelChanged(double value);

protected:
    void keyPressEvent(QKeyEvent* event) override;
    void keyReleaseEvent(QKeyEvent* event) override;

private:
    void setupUi();
    void createControlGroup();
    void createStatusGroup();
    void createJoystick();
    void setupKeyboardControl();
    void connectSignalsAndSlots();
    void updateKeyboardControl();
    void updateKeyboardVelocity();

    std::unique_ptr<ControlPanelPrivate> d_;
};

#endif // ROBOT_CONTROL_GUI_CONTROL_PANEL_H 