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
 * @file control_panel.cpp
 * @brief 机器人控制面板类的实现
 * @author JIAlonglong
 */

#include "control_panel.h"
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QMap>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>
#include "robot_controller.h"
#include "joystick_widget.h"
#include "battery_indicator.h"
#include "speed_dashboard.h"
#include "speed_display.h"

class ControlPanelPrivate
{
public:
    explicit ControlPanelPrivate(ControlPanel* q) : q_ptr(q) {}

    ControlPanel* const              q_ptr;
    std::shared_ptr<RobotController> robot_controller;

    QVBoxLayout* main_layout_{nullptr};
    QGroupBox*   joystick_group_{nullptr};
    QGroupBox*   control_group_{nullptr};
    QGroupBox*   battery_group_{nullptr};

    JoystickWidget* joystick_{nullptr};
    SpeedDisplay*   speed_display_{nullptr};
    QComboBox*      control_mode_combo_{nullptr};
    QDoubleSpinBox* speed_limit_spin_{nullptr};
    QDoubleSpinBox* accel_limit_spin_{nullptr};
    QPushButton*    emergency_stop_button_{nullptr};

    BatteryIndicator* battery_indicator_{nullptr};
    QLabel*           battery_level_label_{nullptr};
    QLabel*           temperature_label_{nullptr};
    QLabel*           voltage_label_{nullptr};
    QLabel*           current_label_{nullptr};

    QMap<int, bool> key_pressed_;
    bool            is_controlling_{false};
    double          keyboard_linear_speed_{0.5};
    double          keyboard_angular_speed_{0.5};
};

ControlPanel::ControlPanel(const std::shared_ptr<RobotController>& controller, QWidget* parent)
    : QWidget(parent), d_(new ControlPanelPrivate(this))
{
    d_->robot_controller = controller;
    setupUi();
    connectSignalsAndSlots();
}

ControlPanel::~ControlPanel() = default;

void ControlPanel::setupUi()
{
    d_->main_layout_ = new QVBoxLayout(this);

    // 创建速度仪表盘组
    d_->speed_group_     = new QGroupBox("速度显示", this);
    auto* speed_layout   = new QVBoxLayout(d_->speed_group_);
    d_->speed_dashboard_ = new SpeedDashboard(this);
    speed_layout->addWidget(d_->speed_dashboard_);
    d_->main_layout_->addWidget(d_->speed_group_);

    // 创建摇杆控制组
    d_->joystick_group_   = new QGroupBox("摇杆控制", this);
    auto* joystick_layout = new QVBoxLayout(d_->joystick_group_);
    d_->joystick_         = new JoystickWidget(this);
    joystick_layout->addWidget(d_->joystick_);
    d_->main_layout_->addWidget(d_->joystick_group_);

    // 创建控制模式组
    d_->control_group_   = new QGroupBox("控制设置", this);
    auto* control_layout = new QGridLayout(d_->control_group_);

    // 控制模式选择
    control_layout->addWidget(new QLabel("控制模式:", this), 0, 0);
    d_->control_mode_combo_ = new QComboBox(this);
    d_->control_mode_combo_->addItem("键盘控制");
    d_->control_mode_combo_->addItem("摇杆控制");
    control_layout->addWidget(d_->control_mode_combo_, 0, 1);

    // 速度限制设置
    control_layout->addWidget(new QLabel("速度限制:", this), 1, 0);
    d_->speed_limit_spin_ = new QDoubleSpinBox(this);
    d_->speed_limit_spin_->setRange(0.1, 2.0);
    d_->speed_limit_spin_->setValue(1.0);
    d_->speed_limit_spin_->setSingleStep(0.1);
    control_layout->addWidget(d_->speed_limit_spin_, 1, 1);

    // 加速度限制设置
    control_layout->addWidget(new QLabel("加速度限制:", this), 2, 0);
    d_->accel_limit_spin_ = new QDoubleSpinBox(this);
    d_->accel_limit_spin_->setRange(0.1, 2.0);
    d_->accel_limit_spin_->setValue(1.0);
    d_->accel_limit_spin_->setSingleStep(0.1);
    control_layout->addWidget(d_->accel_limit_spin_, 2, 1);

    // 紧急停止按钮
    d_->emergency_stop_button_ = new QPushButton("紧急停止", this);
    d_->emergency_stop_button_->setStyleSheet("background-color: red; color: white;");
    control_layout->addWidget(d_->emergency_stop_button_, 3, 0, 1, 2);

    d_->main_layout_->addWidget(d_->control_group_);
}

void ControlPanel::createStatusGroup()
{
    d_->battery_group_ = new QGroupBox(tr("状态信息"), this);
    auto* layout       = new QGridLayout(d_->battery_group_);

    // 电池状态显示
    d_->battery_indicator_ = new BatteryIndicator(this);
    layout->addWidget(d_->battery_indicator_, 0, 0, 1, 2);

    // 电池详细信息
    d_->battery_level_label_ = new QLabel(tr("电量: 100%"), this);
    d_->temperature_label_   = new QLabel(tr("温度: 25°C"), this);
    d_->voltage_label_       = new QLabel(tr("电压: 12.0V"), this);
    d_->current_label_       = new QLabel(tr("电流: 0.0A"), this);

    layout->addWidget(d_->battery_level_label_, 1, 0);
    layout->addWidget(d_->temperature_label_, 1, 1);
    layout->addWidget(d_->voltage_label_, 2, 0);
    layout->addWidget(d_->current_label_, 2, 1);

    d_->main_layout_->addWidget(d_->battery_group_);
}

void ControlPanel::connectSignalsAndSlots()
{
    // 连接摇杆信号
    connect(d_->joystick_, &JoystickWidget::linearJoystickMoved, this,
            &ControlPanel::onJoystickMoved);

    // 连接控制模式切换信号
    connect(d_->control_mode_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            &ControlPanel::onControlModeChanged);

    // 连接速度限制信号
    connect(d_->speed_limit_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &ControlPanel::onSpeedLimitChanged);

    // 连接加速度限制信号
    connect(d_->accel_limit_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &ControlPanel::onAccelLimitChanged);

    // 连接紧急停止信号
    connect(d_->emergency_stop_button_, &QPushButton::clicked, this,
            &ControlPanel::onEmergencyStop);

    // 连接速度更新信号到仪表盘
    if (d_->robot_controller) {
        connect(d_->robot_controller.get(), &RobotController::linearVelocityChanged,
                d_->speed_dashboard_, &SpeedDashboard::setLinearSpeed);
        connect(d_->robot_controller.get(), &RobotController::angularVelocityChanged,
                d_->speed_dashboard_, &SpeedDashboard::setAngularSpeed);
    }
}

void ControlPanel::onJoystickMoved()
{
    if (!d_->is_controlling_ || !d_->robot_controller) return;

    double linear  = d_->joystick_->getLinearVelocity();
    double angular = d_->joystick_->getAngularVelocity();

    d_->robot_controller->setLinearVelocity(linear);
    d_->robot_controller->setAngularVelocity(angular);
    updateVelocityDisplay(linear, angular);
    Q_EMIT velocityChanged(linear, angular);
}

void ControlPanel::onControlModeChanged(int index)
{
    if (!d_->robot_controller) return;

    d_->robot_controller->setControlMode(index);
}

void ControlPanel::onSpeedLimitChanged(double value)
{
    if (!d_->robot_controller) return;

    d_->robot_controller->setMaxSpeed(value);
}

void ControlPanel::onAccelLimitChanged(double value)
{
    if (!d_->robot_controller) return;

    d_->robot_controller->setMaxAcceleration(value);
}

void ControlPanel::onEmergencyStop()
{
    if (d_->robot_controller) {
        d_->robot_controller->emergencyStop();
    }
    Q_EMIT emergencyStopTriggered();
}

void ControlPanel::setupKeyboardControl()
{
    // 设置焦点策略以接收键盘事件
    setFocusPolicy(Qt::StrongFocus);
}

void ControlPanel::updateVelocityDisplay(double linear, double angular)
{
    if (d_->speed_dashboard_) {
        d_->speed_dashboard_->setLinearSpeed(linear);
        d_->speed_dashboard_->setAngularSpeed(angular);
    }
}

void ControlPanel::updateBatteryState(double percentage, double voltage, double current,
                                      double temperature)
{
    d_->battery_indicator_->setBatteryLevel(percentage);
    d_->battery_level_label_->setText(tr("电量: %1%").arg(percentage, 0, 'f', 1));
    d_->voltage_label_->setText(tr("电压: %1V").arg(voltage, 0, 'f', 1));
    d_->current_label_->setText(tr("电流: %1A").arg(current, 0, 'f', 1));
    d_->temperature_label_->setText(tr("温度: %1°C").arg(temperature, 0, 'f', 1));
}

void ControlPanel::updateMotorStatus(const QString& status)
{
    // 可以添加电机状态显示
    // TODO: 添加电机状态标签和显示逻辑
}

void ControlPanel::onKeyboardControl()
{
    if (!d_->robot_controller || !d_->is_controlling_) return;

    double linear  = 0.0;
    double angular = 0.0;

    if (d_->key_pressed_[Qt::Key_W]) linear += d_->keyboard_linear_speed_;
    if (d_->key_pressed_[Qt::Key_S]) linear -= d_->keyboard_linear_speed_;
    if (d_->key_pressed_[Qt::Key_A]) angular += d_->keyboard_angular_speed_;
    if (d_->key_pressed_[Qt::Key_D]) angular -= d_->keyboard_angular_speed_;

    d_->robot_controller->setLinearVelocity(linear);
    d_->robot_controller->setAngularVelocity(angular);
    d_->speed_display_->updateSpeed(linear, angular);
}

void ControlPanel::keyPressEvent(QKeyEvent* event)
{
    if (d_->control_mode_combo_->currentIndex() != 1) {
        QWidget::keyPressEvent(event);
        return;
    }

    if (!event->isAutoRepeat()) {
        d_->key_pressed_[event->key()] = true;
        updateKeyboardControl();
    }
}

void ControlPanel::keyReleaseEvent(QKeyEvent* event)
{
    if (d_->control_mode_combo_->currentIndex() != 1) {
        QWidget::keyReleaseEvent(event);
        return;
    }

    if (!event->isAutoRepeat()) {
        d_->key_pressed_[event->key()] = false;
        updateKeyboardControl();
    }
}

void ControlPanel::updateKeyboardControl()
{
    double linear  = 0.0;
    double angular = 0.0;

    if (d_->key_pressed_[Qt::Key_W]) linear += d_->keyboard_linear_speed_;
    if (d_->key_pressed_[Qt::Key_S]) linear -= d_->keyboard_linear_speed_;
    if (d_->key_pressed_[Qt::Key_A]) angular += d_->keyboard_angular_speed_;
    if (d_->key_pressed_[Qt::Key_D]) angular -= d_->keyboard_angular_speed_;

    d_->robot_controller->setLinearVelocity(linear);
    d_->robot_controller->setAngularVelocity(angular);
    Q_EMIT velocityChanged(linear, angular);
}

void ControlPanel::setRobotController(const std::shared_ptr<RobotController>& controller)
{
    d_->robot_controller = controller;
}

void ControlPanel::handleKeyEvent(QKeyEvent* event, bool pressed)
{
    if (!d_->robot_controller) {
        return;
    }

    double linear  = 0.0;
    double angular = 0.0;

    if (pressed) {
        switch (event->key()) {
            case Qt::Key_W:
                linear = d_->keyboard_linear_speed_;
                break;
            case Qt::Key_S:
                linear = -d_->keyboard_linear_speed_;
                break;
            case Qt::Key_A:
                angular = d_->keyboard_angular_speed_;
                break;
            case Qt::Key_D:
                angular = -d_->keyboard_angular_speed_;
                break;
            default:
                break;
        }
    }

    d_->robot_controller->publishVelocity(linear, angular);
}

void ControlPanel::onMaxSpeedChanged(double value)
{
    if (d_->robot_controller) {
        d_->robot_controller->setMaxSpeed(value);
    }
}

void ControlPanel::onMaxAngularSpeedChanged(double value)
{
    if (d_->robot_controller) {
        d_->robot_controller->setMaxAngularSpeed(value);
    }
}

void ControlPanel::onMaxAccelChanged(double value)
{
    if (d_->robot_controller) {
        d_->robot_controller->setMaxAcceleration(value);
    }
}