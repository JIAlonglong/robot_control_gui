#ifndef SETTINGS_PANEL_H
#define SETTINGS_PANEL_H

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QComboBox>
#include <QLineEdit>

#include "ros/robot_controller.h"

class SettingsPanel : public QWidget
{
    Q_OBJECT

public:
    explicit SettingsPanel(RobotController* robot_controller, QWidget* parent = nullptr);
    ~SettingsPanel();

private slots:
    void onYawToleranceChanged(double value);
    void onInflationRadiusChanged(double value);
    void onTransformToleranceChanged(double value);
    void onPlannerFrequencyChanged(double value);
    void onControllerFrequencyChanged(double value);
    void onGlobalCostmapUpdateFrequencyChanged(double value);
    void onLocalCostmapUpdateFrequencyChanged(double value);
    void onPlannedPathBiasChanged(double value);
    void onRecoveryBehaviorEnabledChanged(bool enabled);
    void onClearingRotationAllowedChanged(bool allowed);
    void onApplySettingsClicked();
    void onResetSettingsClicked();

private:
    void setupUi();
    void setupConnections();
    void loadSettings();
    void saveSettings();
    void applySettings();
    void resetSettings();

    RobotController* robot_controller_;

    // UI组件
    QDoubleSpinBox* yaw_tolerance_spin_;
    QDoubleSpinBox* inflation_radius_spin_;
    QDoubleSpinBox* transform_tolerance_spin_;
    QDoubleSpinBox* planner_frequency_spin_;
    QDoubleSpinBox* controller_frequency_spin_;
    QDoubleSpinBox* global_costmap_update_frequency_spin_;
    QDoubleSpinBox* local_costmap_update_frequency_spin_;
    QDoubleSpinBox* planned_path_bias_spin_;
    QCheckBox* recovery_behavior_enabled_check_;
    QCheckBox* clearing_rotation_allowed_check_;
    QPushButton* apply_settings_button_;
    QPushButton* reset_settings_button_;

    // 默认值
    static constexpr double DEFAULT_YAW_TOLERANCE = 0.1;
    static constexpr double DEFAULT_INFLATION_RADIUS = 0.55;
    static constexpr double DEFAULT_TRANSFORM_TOLERANCE = 0.2;
    static constexpr double DEFAULT_PLANNER_FREQUENCY = 0.0;
    static constexpr double DEFAULT_CONTROLLER_FREQUENCY = 20.0;
    static constexpr double DEFAULT_GLOBAL_COSTMAP_UPDATE_FREQUENCY = 5.0;
    static constexpr double DEFAULT_LOCAL_COSTMAP_UPDATE_FREQUENCY = 5.0;
    static constexpr double DEFAULT_PLANNED_PATH_BIAS = 0.75;
};

#endif // SETTINGS_PANEL_H 