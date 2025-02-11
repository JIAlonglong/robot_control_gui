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
 * @file teleoperation_panel.cpp
 * @brief 遥操作面板类的实现,用于远程控制机器人
 * @author JIAlonglong
 */

#include "teleoperation_panel.h"
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>
#include "joystick_widget.h"

struct TeleoperationPanel::Private {
    std::shared_ptr<RobotController> controller;

    QVBoxLayout* main_layout{nullptr};
    QGroupBox*   control_group{nullptr};
    QGroupBox*   joystick_group{nullptr};
    QGroupBox*   keyboard_group{nullptr};
    QGroupBox*   status_group{nullptr};

    // 控制组件
    QComboBox*      control_mode_combo{nullptr};
    QDoubleSpinBox* speed_limit_spin{nullptr};
    QDoubleSpinBox* accel_limit_spin{nullptr};
    QPushButton*    emergency_stop_button{nullptr};

    // 摇杆组件
    JoystickWidget* joystick{nullptr};
    QLabel*         joystick_status_label{nullptr};

    // 键盘控制组件
    QLabel*         keyboard_status_label{nullptr};
    QMap<int, bool> key_pressed;
    double          keyboard_linear_speed{0.2};
    double          keyboard_angular_speed{0.5};

    // 状态组件
    QLabel*       velocity_label{nullptr};
    QProgressBar* linear_velocity_bar{nullptr};
    QProgressBar* angular_velocity_bar{nullptr};

    bool is_controlling{false};
};

TeleoperationPanel::TeleoperationPanel(QWidget* parent)
    : QWidget(parent), d_(std::make_unique<Private>())
{
    setupUi();
    connectSignalsAndSlots();
}

TeleoperationPanel::~TeleoperationPanel() = default;

void TeleoperationPanel::setRobotController(const std::shared_ptr<RobotController>& controller)
{
    d_->controller = controller;
    if (d_->controller) {
        connect(d_->controller.get(), &RobotController::velocityChanged, this,
                &TeleoperationPanel::updateVelocityDisplay);
    }
}

void TeleoperationPanel::setupUi()
{
    d_->main_layout = new QVBoxLayout(this);
    d_->main_layout->setContentsMargins(4, 4, 4, 4);
    d_->main_layout->setSpacing(4);

    createControlGroup();
    createJoystickGroup();
    createKeyboardGroup();
    createStatusGroup();

    d_->main_layout->addStretch();
}

void TeleoperationPanel::createControlGroup()
{
    d_->control_group = new QGroupBox(tr("控制设置"), this);
    auto* layout      = new QGridLayout(d_->control_group);

    // 控制模式选择
    layout->addWidget(new QLabel(tr("控制模式:")), 0, 0);
    d_->control_mode_combo = new QComboBox(this);
    d_->control_mode_combo->addItem(tr("键盘控制"), "keyboard");
    d_->control_mode_combo->addItem(tr("摇杆控制"), "joystick");
    layout->addWidget(d_->control_mode_combo, 0, 1);

    // 速度限制设置
    layout->addWidget(new QLabel(tr("最大速度(m/s):")), 1, 0);
    d_->speed_limit_spin = new QDoubleSpinBox(this);
    d_->speed_limit_spin->setRange(0.1, 2.0);
    d_->speed_limit_spin->setValue(1.0);
    d_->speed_limit_spin->setSingleStep(0.1);
    layout->addWidget(d_->speed_limit_spin, 1, 1);

    // 加速度限制设置
    layout->addWidget(new QLabel(tr("最大加速度(m/s²):")), 2, 0);
    d_->accel_limit_spin = new QDoubleSpinBox(this);
    d_->accel_limit_spin->setRange(0.1, 2.0);
    d_->accel_limit_spin->setValue(1.0);
    d_->accel_limit_spin->setSingleStep(0.1);
    layout->addWidget(d_->accel_limit_spin, 2, 1);

    // 紧急停止按钮
    d_->emergency_stop_button = new QPushButton(tr("紧急停止"), this);
    d_->emergency_stop_button->setStyleSheet(
        "QPushButton { background-color: red; color: white; }");
    layout->addWidget(d_->emergency_stop_button, 3, 0, 1, 2);

    d_->main_layout->addWidget(d_->control_group);
}

void TeleoperationPanel::createJoystickGroup()
{
    d_->joystick_group = new QGroupBox(tr("摇杆控制"), this);
    auto* layout       = new QVBoxLayout(d_->joystick_group);

    d_->joystick = new JoystickWidget(this);
    layout->addWidget(d_->joystick);

    d_->joystick_status_label = new QLabel(tr("未控制"), this);
    d_->joystick_status_label->setAlignment(Qt::AlignCenter);
    layout->addWidget(d_->joystick_status_label);

    d_->main_layout->addWidget(d_->joystick_group);
}

void TeleoperationPanel::createKeyboardGroup()
{
    d_->keyboard_group = new QGroupBox(tr("键盘控制"), this);
    auto* layout       = new QVBoxLayout(d_->keyboard_group);

    auto* help_label = new QLabel(tr("使用以下按键控制机器人移动:\n"
                                     "↑: 前进\n"
                                     "↓: 后退\n"
                                     "←: 左转\n"
                                     "→: 右转\n"
                                     "空格: 停止"),
                                  this);
    help_label->setAlignment(Qt::AlignCenter);
    layout->addWidget(help_label);

    d_->keyboard_status_label = new QLabel(tr("未控制"), this);
    d_->keyboard_status_label->setAlignment(Qt::AlignCenter);
    layout->addWidget(d_->keyboard_status_label);

    d_->main_layout->addWidget(d_->keyboard_group);
}

void TeleoperationPanel::createStatusGroup()
{
    d_->status_group = new QGroupBox(tr("状态显示"), this);
    auto* layout     = new QVBoxLayout(d_->status_group);

    d_->velocity_label = new QLabel(tr("当前速度: 0.0 m/s, 0.0 rad/s"), this);
    layout->addWidget(d_->velocity_label);

    auto* linear_layout = new QHBoxLayout;
    linear_layout->addWidget(new QLabel(tr("线速度:")));
    d_->linear_velocity_bar = new QProgressBar(this);
    d_->linear_velocity_bar->setRange(-100, 100);
    d_->linear_velocity_bar->setValue(0);
    linear_layout->addWidget(d_->linear_velocity_bar);
    layout->addLayout(linear_layout);

    auto* angular_layout = new QHBoxLayout;
    angular_layout->addWidget(new QLabel(tr("角速度:")));
    d_->angular_velocity_bar = new QProgressBar(this);
    d_->angular_velocity_bar->setRange(-100, 100);
    d_->angular_velocity_bar->setValue(0);
    angular_layout->addWidget(d_->angular_velocity_bar);
    layout->addLayout(angular_layout);

    d_->main_layout->addWidget(d_->status_group);
}

void TeleoperationPanel::connectSignalsAndSlots()
{
    connect(d_->control_mode_combo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            &TeleoperationPanel::onControlModeChanged);

    connect(d_->speed_limit_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &TeleoperationPanel::onSpeedLimitChanged);

    connect(d_->accel_limit_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &TeleoperationPanel::onAccelLimitChanged);

    connect(d_->emergency_stop_button, &QPushButton::clicked, this,
            &TeleoperationPanel::onEmergencyStop);

    connect(d_->joystick, &JoystickWidget::linearJoystickMoved, this,
            &TeleoperationPanel::onJoystickMoved);
}

void TeleoperationPanel::onControlModeChanged(int index)
{
    QString mode = d_->control_mode_combo->itemData(index).toString();
    if (mode == "keyboard") {
        d_->keyboard_group->setEnabled(true);
        d_->joystick_group->setEnabled(false);
        onKeyboardControl();
    } else if (mode == "joystick") {
        d_->keyboard_group->setEnabled(false);
        d_->joystick_group->setEnabled(true);
        onJoystickControl();
    }
}

void TeleoperationPanel::onSpeedLimitChanged(double value)
{
    if (d_->controller) {
        d_->controller->setMaxSpeed(value);
    }
}

void TeleoperationPanel::onAccelLimitChanged(double value)
{
    if (d_->controller) {
        d_->controller->setMaxAcceleration(value);
    }
}

void TeleoperationPanel::onJoystickMoved()
{
    if (!d_->controller || !d_->joystick_group->isEnabled()) return;

    QPointF pos     = d_->joystick->normalizedPosition();
    double  linear  = -pos.y() * d_->speed_limit_spin->value();
    double  angular = -pos.x() * d_->speed_limit_spin->value();

    d_->controller->publishVelocity(linear, angular);
    emit velocityChanged(linear, angular);
}

void TeleoperationPanel::onKeyboardControl()
{
    d_->keyboard_status_label->setText(tr("键盘控制已启用"));
    setFocus();
}

void TeleoperationPanel::onJoystickControl()
{
    d_->joystick_status_label->setText(tr("摇杆控制已启用"));
    d_->joystick->setFocus();
}

void TeleoperationPanel::onEmergencyStop()
{
    if (d_->controller) {
        d_->controller->emergencyStop();
    }
    emit emergencyStopTriggered();
}

void TeleoperationPanel::updateVelocityDisplay(double linear, double angular)
{
    d_->velocity_label->setText(
        tr("当前速度: %1 m/s, %2 rad/s").arg(linear, 0, 'f', 2).arg(angular, 0, 'f', 2));

    d_->linear_velocity_bar->setValue(static_cast<int>(linear * 100.0));
    d_->angular_velocity_bar->setValue(static_cast<int>(angular * 100.0));
}

void TeleoperationPanel::keyPressEvent(QKeyEvent* event)
{
    if (!d_->keyboard_group->isEnabled()) {
        QWidget::keyPressEvent(event);
        return;
    }

    if (!d_->key_pressed.contains(event->key())) {
        d_->key_pressed[event->key()] = true;
        updateKeyboardControl();
    }
}

void TeleoperationPanel::keyReleaseEvent(QKeyEvent* event)
{
    if (!d_->keyboard_group->isEnabled()) {
        QWidget::keyReleaseEvent(event);
        return;
    }

    d_->key_pressed.remove(event->key());
    updateKeyboardControl();
}

void TeleoperationPanel::updateKeyboardControl()
{
    if (!d_->controller) return;

    double linear  = 0.0;
    double angular = 0.0;

    if (d_->key_pressed.contains(Qt::Key_Up)) linear += d_->keyboard_linear_speed;
    if (d_->key_pressed.contains(Qt::Key_Down)) linear -= d_->keyboard_linear_speed;
    if (d_->key_pressed.contains(Qt::Key_Left)) angular += d_->keyboard_angular_speed;
    if (d_->key_pressed.contains(Qt::Key_Right)) angular -= d_->keyboard_angular_speed;

    d_->controller->publishVelocity(linear, angular);
    emit velocityChanged(linear, angular);
}