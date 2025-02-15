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

#include "ui/settings_panel.h"
#include "ros/robot_controller.h"
#include <QLineEdit>
#include <QPushButton>
#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QMessageBox>
#include <QSettings>
#include <QDir>
#include <QSerialPortInfo>
#include <QHostInfo>
#include <QNetworkInterface>

SettingsPanel::SettingsPanel(std::shared_ptr<RobotController> robot_controller, QWidget* parent)
    : QWidget(parent)
    , d_ptr(std::make_unique<SettingsPanelPrivate>())
{
    d_ptr->robot_controller = std::move(robot_controller);
    d_ptr->config_file_path = QDir::homePath() + "/.robot_control/settings.ini";
    
    setupUi();
    loadSettings();
}

SettingsPanel::~SettingsPanel() = default;

void SettingsPanel::setupUi()
{
    auto* main_layout = new QVBoxLayout(this);
    main_layout->setSpacing(20);
    
    // 设置组件样式
    setStyleSheet(
        "QGroupBox {"
        "    border: 1px solid #cccccc;"
        "    border-radius: 6px;"
        "    margin-top: 1em;"
        "    padding: 10px;"
        "    background-color: #ffffff;"
        "}"
        "QGroupBox::title {"
        "    subcontrol-origin: margin;"
        "    left: 10px;"
        "    padding: 0 3px 0 3px;"
        "    color: #333333;"
        "}"
        "QPushButton {"
        "    border: 1px solid #cccccc;"
        "    border-radius: 4px;"
        "    padding: 8px 16px;"
        "    background-color: #ffffff;"
        "    color: #333333;"
        "}"
        "QPushButton:hover {"
        "    background-color: #f5f5f5;"
        "}"
        "QPushButton:pressed {"
        "    background-color: #e5e5e5;"
        "}"
        "QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox {"
        "    border: 1px solid #cccccc;"
        "    border-radius: 4px;"
        "    padding: 5px;"
        "    background-color: #ffffff;"
        "}"
        "QLabel {"
        "    color: #333333;"
        "}"
    );
    
    // 创建各个设置组
    setupRosNetworkGroup();
    setupRobotConfigGroup();
    setupNavigationGroup();
    
    // 创建底部按钮
    auto* button_layout = new QHBoxLayout();
    button_layout->setSpacing(10);
    
    d_ptr->save_button = new QPushButton(tr("保存设置"), this);
    d_ptr->load_button = new QPushButton(tr("加载设置"), this);
    d_ptr->apply_button = new QPushButton(tr("应用设置"), this);
    
    d_ptr->apply_button->setStyleSheet(
        "QPushButton {"
        "    background-color: #28a745;"
        "    color: white;"
        "    border: none;"
        "}"
        "QPushButton:hover {"
        "    background-color: #218838;"
        "}"
        "QPushButton:pressed {"
        "    background-color: #1e7e34;"
        "}"
    );
    
    button_layout->addStretch();
    button_layout->addWidget(d_ptr->load_button);
    button_layout->addWidget(d_ptr->save_button);
    button_layout->addWidget(d_ptr->apply_button);
    
    // 添加到主布局
    main_layout->addWidget(d_ptr->ros_network_group);
    main_layout->addWidget(d_ptr->robot_config_group);
    main_layout->addWidget(d_ptr->navigation_group);
    main_layout->addLayout(button_layout);
    
    // 连接信号
    connect(d_ptr->test_connection_button, &QPushButton::clicked,
            this, &SettingsPanel::onTestConnection);
    connect(d_ptr->save_button, &QPushButton::clicked,
            this, &SettingsPanel::onSaveSettings);
    connect(d_ptr->load_button, &QPushButton::clicked,
            this, &SettingsPanel::onLoadSettings);
    connect(d_ptr->apply_button, &QPushButton::clicked,
            this, &SettingsPanel::onApplySettings);
}

void SettingsPanel::setupRosNetworkGroup()
{
    d_ptr->ros_network_group = new QGroupBox(tr("ROS网络设置"), this);
    auto* layout = new QGridLayout(d_ptr->ros_network_group);
    layout->setSpacing(10);
    
    // Master URI
    auto* master_uri_label = new QLabel(tr("ROS Master URI:"), this);
    d_ptr->master_uri_edit = new QLineEdit(this);
    d_ptr->master_uri_edit->setPlaceholderText("http://192.168.1.100:11311");
    
    // 本机IP
    auto* local_ip_label = new QLabel(tr("本机IP:"), this);
    d_ptr->local_ip_combo = new QComboBox(this);
    
    // 获取本机所有网络接口的IP地址
    foreach(const QHostAddress &address, QNetworkInterface::allAddresses()) {
        if (address.protocol() == QAbstractSocket::IPv4Protocol && 
            address != QHostAddress::LocalHost) {
            d_ptr->local_ip_combo->addItem(address.toString());
        }
    }
    d_ptr->local_ip_combo->addItem("localhost");
    
    // 机器人IP
    auto* robot_ip_label = new QLabel(tr("机器人IP:"), this);
    d_ptr->robot_ip_edit = new QLineEdit(this);
    d_ptr->robot_ip_edit->setPlaceholderText("192.168.1.101");
    
    // 网络模式
    auto* network_mode_label = new QLabel(tr("网络模式:"), this);
    d_ptr->network_mode_combo = new QComboBox(this);
    d_ptr->network_mode_combo->addItems({
        tr("主机模式 (Master)"),
        tr("从机模式 (Slave)")
    });
    
    // 连接状态和按钮
    d_ptr->test_connection_button = new QPushButton(tr("测试连接"), this);
    d_ptr->connection_status_label = new QLabel(tr("未连接"), this);
    d_ptr->connection_status_label->setStyleSheet("color: #dc3545;");
    
    // 添加到布局
    int row = 0;
    layout->addWidget(network_mode_label, row, 0);
    layout->addWidget(d_ptr->network_mode_combo, row++, 1, 1, 2);
    
    layout->addWidget(master_uri_label, row, 0);
    layout->addWidget(d_ptr->master_uri_edit, row++, 1, 1, 2);
    
    layout->addWidget(local_ip_label, row, 0);
    layout->addWidget(d_ptr->local_ip_combo, row++, 1, 1, 2);
    
    layout->addWidget(robot_ip_label, row, 0);
    layout->addWidget(d_ptr->robot_ip_edit, row++, 1, 1, 2);
    
    layout->addWidget(d_ptr->test_connection_button, row, 1);
    layout->addWidget(d_ptr->connection_status_label, row++, 2);
    
    // 连接信号
    connect(d_ptr->network_mode_combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &SettingsPanel::onNetworkModeChanged);
    connect(d_ptr->local_ip_combo, &QComboBox::currentTextChanged,
            this, &SettingsPanel::updateMasterURI);
}

void SettingsPanel::onNetworkModeChanged(int index)
{
    bool is_master = (index == 0);
    
    // 根据模式更新界面
    d_ptr->robot_ip_edit->setEnabled(!is_master);
    
    // 更新 Master URI
    updateMasterURI();
}

void SettingsPanel::updateMasterURI()
{
    bool is_master = (d_ptr->network_mode_combo->currentIndex() == 0);
    QString ip = is_master ? d_ptr->local_ip_combo->currentText() : d_ptr->robot_ip_edit->text();
    
    if (ip.isEmpty()) {
        ip = "localhost";
    }
    
    d_ptr->master_uri_edit->setText(QString("http://%1:11311").arg(ip));
}

void SettingsPanel::applyNetworkSettings()
{
    if (!d_ptr->robot_controller) return;
    
    try {
        bool is_master = (d_ptr->network_mode_combo->currentIndex() == 0);
        QString master_uri = d_ptr->master_uri_edit->text();
        QString local_ip = d_ptr->local_ip_combo->currentText();
        
        // 设置环境变量
        d_ptr->robot_controller->setMasterURI(master_uri);
        d_ptr->robot_controller->setHostname(local_ip);
        
        // 保存设置
        QSettings settings;
        settings.beginGroup("Network");
        settings.setValue("mode", is_master ? "master" : "slave");
        settings.setValue("master_uri", master_uri);
        settings.setValue("local_ip", local_ip);
        settings.setValue("robot_ip", d_ptr->robot_ip_edit->text());
        settings.endGroup();
        
        QMessageBox::information(this, tr("设置成功"), 
            tr("ROS网络设置已更新，请重启程序以使设置生效。"));
            
    } catch (const std::exception& e) {
        QMessageBox::warning(this, tr("设置失败"), 
            tr("更新网络设置失败: %1").arg(e.what()));
    }
}

void SettingsPanel::setupRobotConfigGroup()
{
    d_ptr->robot_config_group = new QGroupBox(tr("机器人配置"), this);
    auto* layout = new QGridLayout(d_ptr->robot_config_group);
    layout->setSpacing(10);
    
    // 机器人型号
    auto* model_label = new QLabel(tr("机器人型号:"), this);
    d_ptr->robot_model_combo = new QComboBox(this);
    d_ptr->robot_model_combo->addItems({
        "TurtleBot3 Burger",
        "TurtleBot3 Waffle",
        "TurtleBot3 Waffle Pi",
        "Custom Robot"
    });
    
    // 串口设备
    auto* serial_label = new QLabel(tr("串口设备:"), this);
    d_ptr->serial_port_combo = new QComboBox(this);
    // 获取可用串口列表
    const auto ports = QSerialPortInfo::availablePorts();
    for (const auto& port : ports) {
        d_ptr->serial_port_combo->addItem(port.portName());
    }
    
    // 波特率
    auto* baudrate_label = new QLabel(tr("波特率:"), this);
    d_ptr->baudrate_combo = new QComboBox(this);
    d_ptr->baudrate_combo->addItems({
        "9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"
    });
    d_ptr->baudrate_combo->setCurrentText("115200");
    
    // 机器人名称
    auto* name_label = new QLabel(tr("机器人名称:"), this);
    d_ptr->robot_name_edit = new QLineEdit(this);
    d_ptr->robot_name_edit->setPlaceholderText("my_robot");
    
    // 机器人ID
    auto* id_label = new QLabel(tr("机器人ID:"), this);
    d_ptr->robot_id_edit = new QLineEdit(this);
    d_ptr->robot_id_edit->setPlaceholderText("robot_001");
    
    // 添加到布局
    layout->addWidget(model_label, 0, 0);
    layout->addWidget(d_ptr->robot_model_combo, 0, 1, 1, 2);
    layout->addWidget(serial_label, 1, 0);
    layout->addWidget(d_ptr->serial_port_combo, 1, 1, 1, 2);
    layout->addWidget(baudrate_label, 2, 0);
    layout->addWidget(d_ptr->baudrate_combo, 2, 1, 1, 2);
    layout->addWidget(name_label, 3, 0);
    layout->addWidget(d_ptr->robot_name_edit, 3, 1, 1, 2);
    layout->addWidget(id_label, 4, 0);
    layout->addWidget(d_ptr->robot_id_edit, 4, 1, 1, 2);
}

void SettingsPanel::setupNavigationGroup()
{
    d_ptr->navigation_group = new QGroupBox(tr("导航参数"), this);
    auto* layout = new QGridLayout(d_ptr->navigation_group);
    layout->setSpacing(10);
    
    // 全局路径规划频率
    auto* global_freq_label = new QLabel(tr("全局规划频率(Hz):"), this);
    d_ptr->global_planner_frequency_spin = new QSpinBox(this);
    d_ptr->global_planner_frequency_spin->setRange(1, 100);
    d_ptr->global_planner_frequency_spin->setValue(5);
    
    // 局部路径规划频率
    auto* local_freq_label = new QLabel(tr("局部规划频率(Hz):"), this);
    d_ptr->local_planner_frequency_spin = new QSpinBox(this);
    d_ptr->local_planner_frequency_spin->setRange(1, 100);
    d_ptr->local_planner_frequency_spin->setValue(20);
    
    // 膨胀半径
    auto* inflation_label = new QLabel(tr("膨胀半径(m):"), this);
    d_ptr->inflation_radius_spin = new QDoubleSpinBox(this);
    d_ptr->inflation_radius_spin->setRange(0.1, 1.0);
    d_ptr->inflation_radius_spin->setSingleStep(0.05);
    d_ptr->inflation_radius_spin->setValue(0.55);
    
    // 机器人半径
    auto* robot_radius_label = new QLabel(tr("机器人半径(m):"), this);
    d_ptr->robot_radius_spin = new QDoubleSpinBox(this);
    d_ptr->robot_radius_spin->setRange(0.1, 1.0);
    d_ptr->robot_radius_spin->setSingleStep(0.05);
    d_ptr->robot_radius_spin->setValue(0.2);
    
    // 最大线速度
    auto* max_vel_x_label = new QLabel(tr("最大线速度(m/s):"), this);
    d_ptr->max_vel_x_spin = new QDoubleSpinBox(this);
    d_ptr->max_vel_x_spin->setRange(0.1, 2.0);
    d_ptr->max_vel_x_spin->setSingleStep(0.1);
    d_ptr->max_vel_x_spin->setValue(0.5);
    
    // 最小线速度
    auto* min_vel_x_label = new QLabel(tr("最小线速度(m/s):"), this);
    d_ptr->min_vel_x_spin = new QDoubleSpinBox(this);
    d_ptr->min_vel_x_spin->setRange(-0.5, 0.2);
    d_ptr->min_vel_x_spin->setSingleStep(0.05);
    d_ptr->min_vel_x_spin->setValue(0.0);
    
    // 最大角速度
    auto* max_vel_theta_label = new QLabel(tr("最大角速度(rad/s):"), this);
    d_ptr->max_vel_theta_spin = new QDoubleSpinBox(this);
    d_ptr->max_vel_theta_spin->setRange(0.1, 3.0);
    d_ptr->max_vel_theta_spin->setSingleStep(0.1);
    d_ptr->max_vel_theta_spin->setValue(1.0);
    
    // 最小角速度
    auto* min_vel_theta_label = new QLabel(tr("最小角速度(rad/s):"), this);
    d_ptr->min_vel_theta_spin = new QDoubleSpinBox(this);
    d_ptr->min_vel_theta_spin->setRange(-1.5, 0.2);
    d_ptr->min_vel_theta_spin->setSingleStep(0.1);
    d_ptr->min_vel_theta_spin->setValue(0.0);
    
    // 线加速度限制
    auto* acc_lim_x_label = new QLabel(tr("线加速度限制(m/s²):"), this);
    d_ptr->acc_lim_x_spin = new QDoubleSpinBox(this);
    d_ptr->acc_lim_x_spin->setRange(0.1, 3.0);
    d_ptr->acc_lim_x_spin->setSingleStep(0.1);
    d_ptr->acc_lim_x_spin->setValue(1.0);
    
    // 角加速度限制
    auto* acc_lim_theta_label = new QLabel(tr("角加速度限制(rad/s²):"), this);
    d_ptr->acc_lim_theta_spin = new QDoubleSpinBox(this);
    d_ptr->acc_lim_theta_spin->setRange(0.1, 5.0);
    d_ptr->acc_lim_theta_spin->setSingleStep(0.1);
    d_ptr->acc_lim_theta_spin->setValue(2.0);
    
    // 添加到布局
    int row = 0;
    layout->addWidget(global_freq_label, row, 0);
    layout->addWidget(d_ptr->global_planner_frequency_spin, row++, 1);
    layout->addWidget(local_freq_label, row, 0);
    layout->addWidget(d_ptr->local_planner_frequency_spin, row++, 1);
    layout->addWidget(inflation_label, row, 0);
    layout->addWidget(d_ptr->inflation_radius_spin, row++, 1);
    layout->addWidget(robot_radius_label, row, 0);
    layout->addWidget(d_ptr->robot_radius_spin, row++, 1);
    layout->addWidget(max_vel_x_label, row, 0);
    layout->addWidget(d_ptr->max_vel_x_spin, row++, 1);
    layout->addWidget(min_vel_x_label, row, 0);
    layout->addWidget(d_ptr->min_vel_x_spin, row++, 1);
    layout->addWidget(max_vel_theta_label, row, 0);
    layout->addWidget(d_ptr->max_vel_theta_spin, row++, 1);
    layout->addWidget(min_vel_theta_label, row, 0);
    layout->addWidget(d_ptr->min_vel_theta_spin, row++, 1);
    layout->addWidget(acc_lim_x_label, row, 0);
    layout->addWidget(d_ptr->acc_lim_x_spin, row++, 1);
    layout->addWidget(acc_lim_theta_label, row, 0);
    layout->addWidget(d_ptr->acc_lim_theta_spin, row++, 1);
}

void SettingsPanel::onTestConnection()
{
    if (!d_ptr->robot_controller) {
        d_ptr->connection_status_label->setText(tr("错误：控制器未初始化"));
        return;
    }

    d_ptr->test_connection_button->setEnabled(false);
    d_ptr->connection_status_label->setText(tr("正在测试..."));
    
    try {
        QString master_uri = d_ptr->master_uri_edit->text();
        if (master_uri.isEmpty()) {
            master_uri = "http://localhost:11311";
        }
        
        bool success = d_ptr->robot_controller->testConnection(master_uri.toStdString());
        
        if (success) {
            d_ptr->connection_status_label->setText(tr("连接成功"));
            d_ptr->connection_status_label->setStyleSheet("color: #28a745;");
        } else {
            d_ptr->connection_status_label->setText(tr("连接失败"));
            d_ptr->connection_status_label->setStyleSheet("color: #dc3545;");
        }
        
    } catch (const std::exception& e) {
        d_ptr->connection_status_label->setText(tr("测试出错: %1").arg(e.what()));
        d_ptr->connection_status_label->setStyleSheet("color: #dc3545;");
    }
    
    d_ptr->test_connection_button->setEnabled(true);
}

void SettingsPanel::onSaveSettings()
{
    if (!validateSettings()) {
        return;
    }
    
    saveSettings();
    QMessageBox::information(this, tr("保存成功"), tr("设置已保存"));
}

void SettingsPanel::onLoadSettings()
{
    loadSettings();
    QMessageBox::information(this, tr("加载成功"), tr("设置已加载"));
}

void SettingsPanel::onApplySettings()
{
    if (!validateSettings()) {
        return;
    }
    
    if (!d_ptr->robot_controller) {
        QMessageBox::warning(this, tr("错误"), tr("机器人控制器未初始化"));
        return;
    }
    
    // 应用ROS网络设置
    d_ptr->robot_controller->setMasterURI(d_ptr->master_uri_edit->text());
    d_ptr->robot_controller->setHostname(d_ptr->local_ip_combo->currentText());
    
    // 应用机器人配置
    d_ptr->robot_controller->setRobotModel(d_ptr->robot_model_combo->currentText().toStdString());
    d_ptr->robot_controller->setSerialPort(d_ptr->serial_port_combo->currentText().toStdString());
    d_ptr->robot_controller->setBaudrate(d_ptr->baudrate_combo->currentText().toInt());
    
    // 应用导航参数
    d_ptr->robot_controller->setPlannerFrequency(d_ptr->global_planner_frequency_spin->value());
    d_ptr->robot_controller->setControllerFrequency(d_ptr->local_planner_frequency_spin->value());
    d_ptr->robot_controller->setInflationRadius(d_ptr->inflation_radius_spin->value());
    
    // 设置速度限制
    d_ptr->robot_controller->setMaxLinearVelocity(d_ptr->max_vel_x_spin->value());
    d_ptr->robot_controller->setMaxAngularVelocity(d_ptr->max_vel_theta_spin->value());
    
    QMessageBox::information(this, tr("应用成功"), tr("设置已应用"));
}

void SettingsPanel::loadSettings()
{
    QSettings settings(d_ptr->config_file_path, QSettings::IniFormat);
    
    // 加载ROS网络设置
    settings.beginGroup("ROS");
    d_ptr->master_uri_edit->setText(settings.value("master_uri", "http://localhost:11311").toString());
    d_ptr->local_ip_combo->setCurrentText(settings.value("local_ip", "localhost").toString());
    settings.endGroup();
    
    // 加载机器人配置
    settings.beginGroup("Robot");
    d_ptr->robot_model_combo->setCurrentText(settings.value("model", "TurtleBot3 Burger").toString());
    d_ptr->serial_port_combo->setCurrentText(settings.value("serial_port", "").toString());
    d_ptr->baudrate_combo->setCurrentText(settings.value("baudrate", "115200").toString());
    d_ptr->robot_name_edit->setText(settings.value("name", "my_robot").toString());
    d_ptr->robot_id_edit->setText(settings.value("id", "robot_001").toString());
    settings.endGroup();
    
    // 加载导航参数
    settings.beginGroup("Navigation");
    d_ptr->global_planner_frequency_spin->setValue(settings.value("global_planner_frequency", 5).toInt());
    d_ptr->local_planner_frequency_spin->setValue(settings.value("local_planner_frequency", 20).toInt());
    d_ptr->inflation_radius_spin->setValue(settings.value("inflation_radius", 0.55).toDouble());
    d_ptr->robot_radius_spin->setValue(settings.value("robot_radius", 0.2).toDouble());
    d_ptr->max_vel_x_spin->setValue(settings.value("max_vel_x", 0.5).toDouble());
    d_ptr->min_vel_x_spin->setValue(settings.value("min_vel_x", 0.0).toDouble());
    d_ptr->max_vel_theta_spin->setValue(settings.value("max_vel_theta", 1.0).toDouble());
    d_ptr->min_vel_theta_spin->setValue(settings.value("min_vel_theta", 0.0).toDouble());
    d_ptr->acc_lim_x_spin->setValue(settings.value("acc_lim_x", 1.0).toDouble());
    d_ptr->acc_lim_theta_spin->setValue(settings.value("acc_lim_theta", 2.0).toDouble());
    settings.endGroup();
}

void SettingsPanel::saveSettings()
{
    QSettings settings(d_ptr->config_file_path, QSettings::IniFormat);
    
    // 保存ROS网络设置
    settings.beginGroup("ROS");
    settings.setValue("master_uri", d_ptr->master_uri_edit->text());
    settings.setValue("local_ip", d_ptr->local_ip_combo->currentText());
    settings.endGroup();
    
    // 保存机器人配置
    settings.beginGroup("Robot");
    settings.setValue("model", d_ptr->robot_model_combo->currentText());
    settings.setValue("serial_port", d_ptr->serial_port_combo->currentText());
    settings.setValue("baudrate", d_ptr->baudrate_combo->currentText());
    settings.setValue("name", d_ptr->robot_name_edit->text());
    settings.setValue("id", d_ptr->robot_id_edit->text());
    settings.endGroup();
    
    // 保存导航参数
    settings.beginGroup("Navigation");
    settings.setValue("global_planner_frequency", d_ptr->global_planner_frequency_spin->value());
    settings.setValue("local_planner_frequency", d_ptr->local_planner_frequency_spin->value());
    settings.setValue("inflation_radius", d_ptr->inflation_radius_spin->value());
    settings.setValue("robot_radius", d_ptr->robot_radius_spin->value());
    settings.setValue("max_vel_x", d_ptr->max_vel_x_spin->value());
    settings.setValue("min_vel_x", d_ptr->min_vel_x_spin->value());
    settings.setValue("max_vel_theta", d_ptr->max_vel_theta_spin->value());
    settings.setValue("min_vel_theta", d_ptr->min_vel_theta_spin->value());
    settings.setValue("acc_lim_x", d_ptr->acc_lim_x_spin->value());
    settings.setValue("acc_lim_theta", d_ptr->acc_lim_theta_spin->value());
    settings.endGroup();
    
    settings.sync();
}

bool SettingsPanel::validateSettings()
{
    // 验证ROS Master URI
    if (d_ptr->master_uri_edit->text().isEmpty()) {
        QMessageBox::warning(this, tr("验证失败"), tr("ROS Master URI不能为空"));
        return false;
    }
    
    // 验证主机名
    if (d_ptr->local_ip_combo->currentText().isEmpty()) {
        QMessageBox::warning(this, tr("验证失败"), tr("本机IP不能为空"));
        return false;
    }
    
    // 验证机器人名称
    if (d_ptr->robot_name_edit->text().isEmpty()) {
        QMessageBox::warning(this, tr("验证失败"), tr("机器人名称不能为空"));
        return false;
    }
    
    // 验证机器人ID
    if (d_ptr->robot_id_edit->text().isEmpty()) {
        QMessageBox::warning(this, tr("验证失败"), tr("机器人ID不能为空"));
        return false;
    }
    
    // 验证速度限制
    if (d_ptr->max_vel_x_spin->value() <= d_ptr->min_vel_x_spin->value()) {
        QMessageBox::warning(this, tr("验证失败"), tr("最大线速度必须大于最小线速度"));
        return false;
    }
    
    if (d_ptr->max_vel_theta_spin->value() <= d_ptr->min_vel_theta_spin->value()) {
        QMessageBox::warning(this, tr("验证失败"), tr("最大角速度必须大于最小角速度"));
        return false;
    }
    
    return true;
}

void SettingsPanel::updateConnectionState(bool connected)
{
    if (!d_ptr) return;

    d_ptr->connect_button->setEnabled(!connected);
    d_ptr->disconnect_button->setEnabled(connected);
    d_ptr->master_uri_edit->setEnabled(!connected);
    d_ptr->local_ip_combo->setEnabled(!connected);
    d_ptr->robot_ip_edit->setEnabled(!connected && 
        d_ptr->network_mode_combo->currentIndex() == 1);  // 只在从机模式下启用
    
    d_ptr->status_label->setText(connected ? tr("已连接") : tr("未连接"));
}

void SettingsPanel::onConnectClicked()
{
    if (!d_ptr->robot_controller) return;
    
    QString master_uri = d_ptr->master_uri_edit->text();
    if (master_uri.isEmpty()) {
        master_uri = "http://localhost:11311";
    }
    
    QString hostname = d_ptr->local_ip_combo->currentText();
    if (hostname.isEmpty()) {
        hostname = "localhost";
    }
    
    d_ptr->robot_controller->setMasterURI(master_uri);
    d_ptr->robot_controller->setHostname(hostname);
    
    if (!d_ptr->robot_controller->connectToRobot()) {
        d_ptr->status_label->setText(tr("连接失败"));
        return;
    }
}

void SettingsPanel::onDisconnectClicked()
{
    if (!d_ptr->robot_controller) return;
    d_ptr->robot_controller->disconnectFromRobot();
}

void SettingsPanel::onConnectionStateChanged(bool connected)
{
    updateConnectionState(connected);
}

void SettingsPanel::onConnectionError(const QString& error)
{
    if (d_ptr && d_ptr->status_label) {
        d_ptr->status_label->setText(error);
    }
} 