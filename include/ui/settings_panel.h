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

#ifndef ROBOT_CONTROL_GUI_SETTINGS_PANEL_H
#define ROBOT_CONTROL_GUI_SETTINGS_PANEL_H

#include <QWidget>
#include <memory>

class QLineEdit;
class QPushButton;
class QComboBox;
class QSpinBox;
class QDoubleSpinBox;
class QLabel;
class QGroupBox;
class RobotController;

class SettingsPanel : public QWidget
{
    Q_OBJECT

public:
    explicit SettingsPanel(std::shared_ptr<RobotController> robot_controller, QWidget* parent = nullptr);
    ~SettingsPanel() override;

private slots:
    void onTestConnection();
    void onSaveSettings();
    void onLoadSettings();
    void onApplySettings();
    void onConnectClicked();
    void onDisconnectClicked();
    void onConnectionStateChanged(bool connected);
    void onConnectionError(const QString& error);
    void onNetworkModeChanged(int index);
    void updateMasterURI();
    void applyNetworkSettings();

private:
    void setupUi();
    void setupRosNetworkGroup();
    void setupRobotConfigGroup();
    void setupNavigationGroup();
    void loadSettings();
    void saveSettings();
    bool validateSettings();
    void updateConnectionState(bool connected);

    class SettingsPanelPrivate {
    public:
        // ROS网络设置
        QGroupBox* ros_network_group{nullptr};
        QLineEdit* master_uri_edit{nullptr};
        QLineEdit* hostname_edit{nullptr};
        QPushButton* test_connection_button{nullptr};
        QLabel* connection_status_label{nullptr};
        QComboBox* network_mode_combo{nullptr};
        QComboBox* local_ip_combo{nullptr};
        QLineEdit* robot_ip_edit{nullptr};
        QPushButton* connect_button{nullptr};
        QPushButton* disconnect_button{nullptr};
        QLabel* status_label{nullptr};

        // 机器人配置
        QGroupBox* robot_config_group{nullptr};
        QComboBox* robot_model_combo{nullptr};
        QComboBox* serial_port_combo{nullptr};
        QComboBox* baudrate_combo{nullptr};
        QLineEdit* robot_name_edit{nullptr};
        QLineEdit* robot_id_edit{nullptr};

        // 导航参数
        QGroupBox* navigation_group{nullptr};
        QSpinBox* global_planner_frequency_spin{nullptr};
        QSpinBox* local_planner_frequency_spin{nullptr};
        QDoubleSpinBox* inflation_radius_spin{nullptr};
        QDoubleSpinBox* robot_radius_spin{nullptr};
        QDoubleSpinBox* max_vel_x_spin{nullptr};
        QDoubleSpinBox* min_vel_x_spin{nullptr};
        QDoubleSpinBox* max_vel_theta_spin{nullptr};
        QDoubleSpinBox* min_vel_theta_spin{nullptr};
        QDoubleSpinBox* acc_lim_x_spin{nullptr};
        QDoubleSpinBox* acc_lim_theta_spin{nullptr};

        // 按钮
        QPushButton* save_button{nullptr};
        QPushButton* load_button{nullptr};
        QPushButton* apply_button{nullptr};

        // 机器人控制器
        std::shared_ptr<RobotController> robot_controller;

        // 配置文件路径
        QString config_file_path;
    };

    std::unique_ptr<SettingsPanelPrivate> d_ptr;
};

#endif // ROBOT_CONTROL_GUI_SETTINGS_PANEL_H 