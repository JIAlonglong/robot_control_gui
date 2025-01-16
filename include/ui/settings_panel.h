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
    explicit SettingsPanel(const std::shared_ptr<RobotController>& robot_controller,
                          QWidget* parent = nullptr);
    ~SettingsPanel() override;

private slots:
    void onTestConnection();
    void onSaveSettings();
    void onLoadSettings();
    void onApplySettings();
    void updateConnectionStatus(bool connected);

private:
    void setupUi();
    void setupRosNetworkGroup();
    void setupRobotConfigGroup();
    void setupNavigationGroup();
    void loadSettings();
    void saveSettings();
    bool validateSettings();

    class SettingsPanelPrivate {
    public:
        // ROS网络设置
        QGroupBox* ros_network_group{nullptr};
        QLineEdit* master_uri_edit{nullptr};
        QLineEdit* hostname_edit{nullptr};
        QPushButton* test_connection_button{nullptr};
        QLabel* connection_status_label{nullptr};

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