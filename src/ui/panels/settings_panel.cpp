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
 * @file settings_panel.cpp
 * @brief 设置面板类的实现
 * @author JIAlonglong
 */

#include "settings_panel.h"
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSettings>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QtSerialPort/QSerialPortInfo>
#include "robot_controller.h"

struct SettingsPanel::Private {
    std::shared_ptr<RobotController> robot_controller;
    QComboBox*                       robot_model_combo{nullptr};
    QComboBox*                       serial_port_combo{nullptr};
    QComboBox*                       baudrate_combo{nullptr};
    QDoubleSpinBox*                  max_speed_spin{nullptr};
    QDoubleSpinBox*                  max_accel_spin{nullptr};
    QDoubleSpinBox*                  safety_distance_spin{nullptr};
    QSpinBox*                        update_frequency_spin{nullptr};
    QCheckBox*                       auto_connect_check{nullptr};
    QCheckBox*                       debug_mode_check{nullptr};
    QComboBox*                       planner_type_combo{nullptr};
    QSpinBox*                        planning_frequency_spin{nullptr};
    QDoubleSpinBox*                  goal_tolerance_spin{nullptr};
    QPushButton*                     save_button{nullptr};
    QPushButton*                     restore_button{nullptr};
    QPushButton*                     refresh_ports_button{nullptr};
    QVBoxLayout*                     main_layout{nullptr};
    QGroupBox*                       connection_group{nullptr};
    QGroupBox*                       planner_group{nullptr};
    QPushButton*                     save_settings_button{nullptr};
    QPushButton*                     restore_defaults_button{nullptr};
    QDoubleSpinBox*                  max_angular_speed_spin{nullptr};
};

SettingsPanel::SettingsPanel(const std::shared_ptr<RobotController>& controller, QWidget* parent)
    : QWidget(parent), d_(std::make_unique<Private>())
{
    d_->robot_controller = controller;

    setupUi();
    connectSignalsAndSlots();
    loadSettings();
}

SettingsPanel::~SettingsPanel() = default;

void SettingsPanel::setupUi()
{
    d_->main_layout = new QVBoxLayout(this);

    createConnectionGroup();
    createMotionGroup();
    createPlannerGroup();
    createButtonGroup();

    d_->main_layout->addStretch();
}

void SettingsPanel::createConnectionGroup()
{
    d_->connection_group = new QGroupBox(tr("连接设置"), this);
    auto* layout         = new QGridLayout(d_->connection_group);

    // 机器人型号选择
    layout->addWidget(new QLabel(tr("机器人型号:"), this), 0, 0);
    d_->robot_model_combo = new QComboBox(this);
    d_->robot_model_combo->addItem(tr("差速轮机器人"), "differential");
    d_->robot_model_combo->addItem(tr("全向轮机器人"), "omni");
    d_->robot_model_combo->addItem(tr("阿克曼机器人"), "ackermann");
    layout->addWidget(d_->robot_model_combo, 0, 1);

    // 串口设置
    layout->addWidget(new QLabel(tr("串口:"), this), 1, 0);
    d_->serial_port_combo = new QComboBox(this);
    updateSerialPorts();
    layout->addWidget(d_->serial_port_combo, 1, 1);

    // 波特率设置
    layout->addWidget(new QLabel(tr("波特率:"), this), 2, 0);
    d_->baudrate_combo = new QComboBox(this);
    d_->baudrate_combo->addItem("9600", 9600);
    d_->baudrate_combo->addItem("19200", 19200);
    d_->baudrate_combo->addItem("38400", 38400);
    d_->baudrate_combo->addItem("57600", 57600);
    d_->baudrate_combo->addItem("115200", 115200);
    d_->baudrate_combo->setCurrentText("115200");
    layout->addWidget(d_->baudrate_combo, 2, 1);

    // 自动连接设置
    d_->auto_connect_check = new QCheckBox(tr("自动连接"), this);
    layout->addWidget(d_->auto_connect_check, 3, 0, 1, 2);

    // 调试模式设置
    d_->debug_mode_check = new QCheckBox(tr("调试模式"), this);
    layout->addWidget(d_->debug_mode_check, 4, 0, 1, 2);

    // 刷新串口按钮
    d_->refresh_ports_button = new QPushButton(tr("刷新串口"), this);
    layout->addWidget(d_->refresh_ports_button, 5, 0, 1, 2);

    d_->main_layout->addWidget(d_->connection_group);
}

void SettingsPanel::createMotionGroup()
{
    auto* group_box = new QGroupBox(tr("运动设置"), this);
    auto* layout    = new QGridLayout(group_box);

    // Max speed
    auto* max_speed_label = new QLabel(tr("最大线速度 (m/s):"), this);
    d_->max_speed_spin    = new QDoubleSpinBox(this);
    d_->max_speed_spin->setRange(0.1, 2.0);
    d_->max_speed_spin->setValue(1.0);
    d_->max_speed_spin->setSingleStep(0.1);
    layout->addWidget(max_speed_label, 0, 0);
    layout->addWidget(d_->max_speed_spin, 0, 1);

    // Max angular speed
    auto* max_angular_speed_label = new QLabel(tr("最大角速度 (rad/s):"), this);
    d_->max_angular_speed_spin    = new QDoubleSpinBox(this);
    d_->max_angular_speed_spin->setRange(0.1, 3.14);
    d_->max_angular_speed_spin->setValue(1.57);
    d_->max_angular_speed_spin->setSingleStep(0.1);
    layout->addWidget(max_angular_speed_label, 1, 0);
    layout->addWidget(d_->max_angular_speed_spin, 1, 1);

    // Max acceleration
    auto* max_accel_label = new QLabel(tr("最大加速度 (m/s²):"), this);
    d_->max_accel_spin    = new QDoubleSpinBox(this);
    d_->max_accel_spin->setRange(0.1, 2.0);
    d_->max_accel_spin->setValue(0.5);
    d_->max_accel_spin->setSingleStep(0.1);
    layout->addWidget(max_accel_label, 2, 0);
    layout->addWidget(d_->max_accel_spin, 2, 1);

    // Safety distance
    auto* safety_distance_label = new QLabel(tr("安全距离 (m):"), this);
    d_->safety_distance_spin    = new QDoubleSpinBox(this);
    d_->safety_distance_spin->setRange(0.1, 1.0);
    d_->safety_distance_spin->setValue(0.3);
    d_->safety_distance_spin->setSingleStep(0.1);
    layout->addWidget(safety_distance_label, 3, 0);
    layout->addWidget(d_->safety_distance_spin, 3, 1);

    // Update frequency
    auto* update_frequency_label = new QLabel(tr("更新频率 (Hz):"), this);
    d_->update_frequency_spin    = new QSpinBox(this);
    d_->update_frequency_spin->setRange(1, 50);
    d_->update_frequency_spin->setValue(20);
    layout->addWidget(update_frequency_label, 4, 0);
    layout->addWidget(d_->update_frequency_spin, 4, 1);

    d_->main_layout->addWidget(group_box);
}

void SettingsPanel::createPlannerGroup()
{
    d_->planner_group = new QGroupBox(tr("规划器设置"), this);
    auto* layout      = new QGridLayout(d_->planner_group);

    // 规划器类型
    layout->addWidget(new QLabel(tr("规划器类型:")), 0, 0);
    d_->planner_type_combo = new QComboBox(this);
    d_->planner_type_combo->addItem("DWA", "dwa");
    d_->planner_type_combo->addItem("TEB", "teb");
    layout->addWidget(d_->planner_type_combo, 0, 1);

    // 路径规划频率
    layout->addWidget(new QLabel(tr("规划频率(Hz):")), 1, 0);
    d_->planning_frequency_spin = new QSpinBox(this);
    d_->planning_frequency_spin->setRange(1, 20);
    d_->planning_frequency_spin->setValue(10);
    layout->addWidget(d_->planning_frequency_spin, 1, 1);

    // 目标点容差
    layout->addWidget(new QLabel(tr("目标点容差(m):")), 2, 0);
    d_->goal_tolerance_spin = new QDoubleSpinBox(this);
    d_->goal_tolerance_spin->setRange(0.01, 0.5);
    d_->goal_tolerance_spin->setValue(0.1);
    d_->goal_tolerance_spin->setSingleStep(0.01);
    layout->addWidget(d_->goal_tolerance_spin, 2, 1);

    d_->main_layout->addWidget(d_->planner_group);
}

void SettingsPanel::createButtonGroup()
{
    auto* button_layout = new QHBoxLayout;

    // 保存设置按钮
    d_->save_settings_button = new QPushButton(tr("保存设置"), this);
    button_layout->addWidget(d_->save_settings_button);

    // 恢复默认设置按钮
    d_->restore_defaults_button = new QPushButton(tr("恢复默认"), this);
    button_layout->addWidget(d_->restore_defaults_button);

    d_->main_layout->addLayout(button_layout);
}

void SettingsPanel::connectSignalsAndSlots()
{
    // Motion settings
    connect(d_->max_speed_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &SettingsPanel::onMaxSpeedChanged);
    connect(d_->max_angular_speed_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &SettingsPanel::onMaxAngularSpeedChanged);
    connect(d_->max_accel_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &SettingsPanel::onMaxAccelChanged);
    connect(d_->safety_distance_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &SettingsPanel::onSafetyDistanceChanged);
    connect(d_->update_frequency_spin, QOverload<int>::of(&QSpinBox::valueChanged), this,
            &SettingsPanel::onUpdateFrequencyChanged);

    // Connection settings
    connect(d_->robot_model_combo, &QComboBox::currentTextChanged, this,
            &SettingsPanel::onRobotModelChanged);
    connect(d_->serial_port_combo, &QComboBox::currentTextChanged, this,
            &SettingsPanel::onSerialPortChanged);
    connect(d_->baudrate_combo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            &SettingsPanel::onBaudrateChanged);
    connect(d_->auto_connect_check, &QCheckBox::stateChanged, this,
            &SettingsPanel::onAutoConnectChanged);
    connect(d_->debug_mode_check, &QCheckBox::stateChanged, this,
            &SettingsPanel::onDebugModeChanged);

    // Planning settings
    connect(d_->planner_type_combo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            &SettingsPanel::onPlannerTypeChanged);
    connect(d_->planning_frequency_spin, QOverload<int>::of(&QSpinBox::valueChanged), this,
            &SettingsPanel::onPlanningFreqChanged);
    connect(d_->goal_tolerance_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &SettingsPanel::onGoalToleranceChanged);

    connect(d_->save_settings_button, &QPushButton::clicked, this, &SettingsPanel::onSaveSettings);

    connect(d_->restore_defaults_button, &QPushButton::clicked, this,
            &SettingsPanel::onRestoreDefaults);

    connect(d_->refresh_ports_button, &QPushButton::clicked, this,
            &SettingsPanel::updateSerialPorts);
}

void SettingsPanel::updateSerialPorts()
{
    d_->serial_port_combo->clear();
    const auto ports = QSerialPortInfo::availablePorts();
    for (const auto& port : ports) {
        d_->serial_port_combo->addItem(port.portName());
    }
}

void SettingsPanel::loadSettings()
{
    QSettings settings;

    // 加载连接设置
    d_->robot_model_combo->setCurrentText(settings.value("robot_model", "differential").toString());
    d_->serial_port_combo->setCurrentText(settings.value("serial_port", "").toString());
    d_->baudrate_combo->setCurrentText(settings.value("baudrate", "115200").toString());
    d_->auto_connect_check->setChecked(settings.value("auto_connect", false).toBool());
    d_->debug_mode_check->setChecked(settings.value("debug_mode", false).toBool());

    // 加载运动设置
    d_->max_speed_spin->setValue(settings.value("max_speed", 1.0).toDouble());
    d_->max_angular_speed_spin->setValue(settings.value("max_angular_speed", 1.57).toDouble());
    d_->max_accel_spin->setValue(settings.value("max_accel", 0.5).toDouble());
    d_->safety_distance_spin->setValue(settings.value("safety_distance", 0.3).toDouble());
    d_->update_frequency_spin->setValue(settings.value("update_frequency", 20).toInt());

    // 加载规划器设置
    d_->planner_type_combo->setCurrentText(settings.value("planner_type", "DWA").toString());
    d_->planning_frequency_spin->setValue(settings.value("planning_frequency", 10).toInt());
    d_->goal_tolerance_spin->setValue(settings.value("goal_tolerance", 0.1).toDouble());
}

void SettingsPanel::onSaveSettings()
{
    QSettings settings;

    // 保存连接设置
    settings.setValue("robot_model", d_->robot_model_combo->currentText());
    settings.setValue("serial_port", d_->serial_port_combo->currentText());
    settings.setValue("baudrate", d_->baudrate_combo->currentText());
    settings.setValue("auto_connect", d_->auto_connect_check->isChecked());
    settings.setValue("debug_mode", d_->debug_mode_check->isChecked());

    // 保存运动设置
    settings.setValue("max_speed", d_->max_speed_spin->value());
    settings.setValue("max_angular_speed", d_->max_angular_speed_spin->value());
    settings.setValue("max_accel", d_->max_accel_spin->value());
    settings.setValue("safety_distance", d_->safety_distance_spin->value());
    settings.setValue("update_frequency", d_->update_frequency_spin->value());

    // 保存规划器设置
    settings.setValue("planner_type", d_->planner_type_combo->currentText());
    settings.setValue("planning_frequency", d_->planning_frequency_spin->value());
    settings.setValue("goal_tolerance", d_->goal_tolerance_spin->value());

    QMessageBox::information(this, tr("成功"), tr("设置已保存"));
}

void SettingsPanel::onRestoreDefaults()
{
    // 恢复连接设置默认值
    d_->robot_model_combo->setCurrentText("differential");
    d_->baudrate_combo->setCurrentText("115200");
    d_->auto_connect_check->setChecked(false);
    d_->debug_mode_check->setChecked(false);

    // 恢复运动设置默认值
    d_->max_speed_spin->setValue(1.0);
    d_->max_angular_speed_spin->setValue(1.57);
    d_->max_accel_spin->setValue(0.5);
    d_->safety_distance_spin->setValue(0.3);
    d_->update_frequency_spin->setValue(20);

    // 恢复规划器设置默认值
    d_->planner_type_combo->setCurrentText("DWA");
    d_->planning_frequency_spin->setValue(10);
    d_->goal_tolerance_spin->setValue(0.1);

    QMessageBox::information(this, tr("成功"), tr("已恢复默认设置"));
}

void SettingsPanel::onMaxSpeedChanged(double value)
{
    if (d_->robot_controller) {
        d_->robot_controller->setMaxSpeed(value);
    }
}

void SettingsPanel::onMaxAngularSpeedChanged(double value)
{
    if (d_->robot_controller) {
        d_->robot_controller->setMaxAngularSpeed(value);
    }
}

void SettingsPanel::onMaxAccelChanged(double value)
{
    if (d_->robot_controller) {
        d_->robot_controller->setMaxAcceleration(value);
    }
}

void SettingsPanel::onSafetyDistanceChanged(double value)
{
    if (d_->robot_controller) {
        d_->robot_controller->setSafetyDistance(value);
    }
}

void SettingsPanel::onUpdateFrequencyChanged(int value)
{
    if (d_->robot_controller) {
        d_->robot_controller->setUpdateFrequency(value);
    }
}

void SettingsPanel::onRobotModelChanged(const QString&)
{
    if (d_->robot_controller) {
        d_->robot_controller->setRobotModel(d_->robot_model_combo->currentData().toString());
    }
}

void SettingsPanel::onSerialPortChanged(const QString&)
{
    if (d_->robot_controller) {
        d_->robot_controller->setSerialPort(d_->serial_port_combo->currentText());
    }
}

void SettingsPanel::onBaudrateChanged(int)
{
    if (d_->robot_controller) {
        d_->robot_controller->setBaudrate(d_->baudrate_combo->currentData().toInt());
    }
}

void SettingsPanel::onAutoConnectChanged(int)
{
    if (d_->robot_controller) {
        d_->robot_controller->setAutoConnect(d_->auto_connect_check->isChecked());
    }
}

void SettingsPanel::onDebugModeChanged(int)
{
    if (d_->robot_controller) {
        d_->robot_controller->setDebugMode(d_->debug_mode_check->isChecked());
    }
}

void SettingsPanel::onPlannerTypeChanged(int)
{
    if (d_->robot_controller) {
        d_->robot_controller->setPlannerType(d_->planner_type_combo->currentData().toString());
    }
}

void SettingsPanel::onPlanningFreqChanged(int value)
{
    if (d_->robot_controller) {
        d_->robot_controller->setPlanningFrequency(value);
    }
}

void SettingsPanel::onGoalToleranceChanged(double value)
{
    if (d_->robot_controller) {
        d_->robot_controller->setGoalTolerance(value);
    }
}