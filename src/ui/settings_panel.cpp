#include "ui/settings_panel.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QSettings>
#include <QMessageBox>

SettingsPanel::SettingsPanel(RobotController* robot_controller, QWidget* parent)
    : QWidget(parent)
    , robot_controller_(robot_controller)
{
    setupUi();
    setupConnections();
    loadSettings();
}

SettingsPanel::~SettingsPanel()
{
    saveSettings();
}

void SettingsPanel::setupUi()
{
    QVBoxLayout* main_layout = new QVBoxLayout(this);
    main_layout->setSpacing(10);
    main_layout->setContentsMargins(10, 10, 10, 10);

    // 导航参数组
    QGroupBox* navigation_group = new QGroupBox(tr("导航参数"), this);
    QGridLayout* navigation_layout = new QGridLayout(navigation_group);

    // 偏航容差
    navigation_layout->addWidget(new QLabel(tr("偏航容差 (rad):")), 0, 0);
    yaw_tolerance_spin_ = new QDoubleSpinBox(this);
    yaw_tolerance_spin_->setRange(0.01, 1.0);
    yaw_tolerance_spin_->setSingleStep(0.01);
    yaw_tolerance_spin_->setValue(DEFAULT_YAW_TOLERANCE);
    navigation_layout->addWidget(yaw_tolerance_spin_, 0, 1);

    // 膨胀半径
    navigation_layout->addWidget(new QLabel(tr("膨胀半径 (m):")), 1, 0);
    inflation_radius_spin_ = new QDoubleSpinBox(this);
    inflation_radius_spin_->setRange(0.1, 2.0);
    inflation_radius_spin_->setSingleStep(0.05);
    inflation_radius_spin_->setValue(DEFAULT_INFLATION_RADIUS);
    navigation_layout->addWidget(inflation_radius_spin_, 1, 1);

    // 变换容差
    navigation_layout->addWidget(new QLabel(tr("变换容差 (s):")), 2, 0);
    transform_tolerance_spin_ = new QDoubleSpinBox(this);
    transform_tolerance_spin_->setRange(0.1, 1.0);
    transform_tolerance_spin_->setSingleStep(0.1);
    transform_tolerance_spin_->setValue(DEFAULT_TRANSFORM_TOLERANCE);
    navigation_layout->addWidget(transform_tolerance_spin_, 2, 1);

    // 规划器频率组
    QGroupBox* planner_group = new QGroupBox(tr("规划器频率"), this);
    QGridLayout* planner_layout = new QGridLayout(planner_group);

    // 规划器频率
    planner_layout->addWidget(new QLabel(tr("规划器频率 (Hz):")), 0, 0);
    planner_frequency_spin_ = new QDoubleSpinBox(this);
    planner_frequency_spin_->setRange(0.0, 100.0);
    planner_frequency_spin_->setSingleStep(1.0);
    planner_frequency_spin_->setValue(DEFAULT_PLANNER_FREQUENCY);
    planner_layout->addWidget(planner_frequency_spin_, 0, 1);

    // 控制器频率
    planner_layout->addWidget(new QLabel(tr("控制器频率 (Hz):")), 1, 0);
    controller_frequency_spin_ = new QDoubleSpinBox(this);
    controller_frequency_spin_->setRange(1.0, 100.0);
    controller_frequency_spin_->setSingleStep(1.0);
    controller_frequency_spin_->setValue(DEFAULT_CONTROLLER_FREQUENCY);
    planner_layout->addWidget(controller_frequency_spin_, 1, 1);

    // 代价地图更新频率组
    QGroupBox* costmap_group = new QGroupBox(tr("代价地图更新频率"), this);
    QGridLayout* costmap_layout = new QGridLayout(costmap_group);

    // 全局代价地图更新频率
    costmap_layout->addWidget(new QLabel(tr("全局代价地图 (Hz):")), 0, 0);
    global_costmap_update_frequency_spin_ = new QDoubleSpinBox(this);
    global_costmap_update_frequency_spin_->setRange(0.1, 10.0);
    global_costmap_update_frequency_spin_->setSingleStep(0.1);
    global_costmap_update_frequency_spin_->setValue(DEFAULT_GLOBAL_COSTMAP_UPDATE_FREQUENCY);
    costmap_layout->addWidget(global_costmap_update_frequency_spin_, 0, 1);

    // 局部代价地图更新频率
    costmap_layout->addWidget(new QLabel(tr("局部代价地图 (Hz):")), 1, 0);
    local_costmap_update_frequency_spin_ = new QDoubleSpinBox(this);
    local_costmap_update_frequency_spin_->setRange(0.1, 10.0);
    local_costmap_update_frequency_spin_->setSingleStep(0.1);
    local_costmap_update_frequency_spin_->setValue(DEFAULT_LOCAL_COSTMAP_UPDATE_FREQUENCY);
    costmap_layout->addWidget(local_costmap_update_frequency_spin_, 1, 1);

    // 路径规划组
    QGroupBox* path_group = new QGroupBox(tr("路径规划"), this);
    QGridLayout* path_layout = new QGridLayout(path_group);

    // 规划路径偏差
    path_layout->addWidget(new QLabel(tr("规划路径偏差:")), 0, 0);
    planned_path_bias_spin_ = new QDoubleSpinBox(this);
    planned_path_bias_spin_->setRange(0.1, 2.0);
    planned_path_bias_spin_->setSingleStep(0.05);
    planned_path_bias_spin_->setValue(DEFAULT_PLANNED_PATH_BIAS);
    path_layout->addWidget(planned_path_bias_spin_, 0, 1);

    // 恢复行为组
    QGroupBox* recovery_group = new QGroupBox(tr("恢复行为"), this);
    QVBoxLayout* recovery_layout = new QVBoxLayout(recovery_group);

    recovery_behavior_enabled_check_ = new QCheckBox(tr("启用恢复行为"), this);
    recovery_behavior_enabled_check_->setChecked(true);
    clearing_rotation_allowed_check_ = new QCheckBox(tr("允许清除旋转"), this);
    clearing_rotation_allowed_check_->setChecked(true);

    recovery_layout->addWidget(recovery_behavior_enabled_check_);
    recovery_layout->addWidget(clearing_rotation_allowed_check_);

    // 按钮组
    QHBoxLayout* button_layout = new QHBoxLayout;
    apply_settings_button_ = new QPushButton(tr("应用设置"), this);
    reset_settings_button_ = new QPushButton(tr("重置设置"), this);
    button_layout->addWidget(apply_settings_button_);
    button_layout->addWidget(reset_settings_button_);

    // 添加所有组到主布局
    main_layout->addWidget(navigation_group);
    main_layout->addWidget(planner_group);
    main_layout->addWidget(costmap_group);
    main_layout->addWidget(path_group);
    main_layout->addWidget(recovery_group);
    main_layout->addLayout(button_layout);
    main_layout->addStretch();
}

void SettingsPanel::setupConnections()
{
    // 导航参数连接
    connect(yaw_tolerance_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SettingsPanel::onYawToleranceChanged);
    connect(inflation_radius_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SettingsPanel::onInflationRadiusChanged);
    connect(transform_tolerance_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SettingsPanel::onTransformToleranceChanged);

    // 规划器频率连接
    connect(planner_frequency_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SettingsPanel::onPlannerFrequencyChanged);
    connect(controller_frequency_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SettingsPanel::onControllerFrequencyChanged);

    // 代价地图更新频率连接
    connect(global_costmap_update_frequency_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SettingsPanel::onGlobalCostmapUpdateFrequencyChanged);
    connect(local_costmap_update_frequency_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SettingsPanel::onLocalCostmapUpdateFrequencyChanged);

    // 路径规划连接
    connect(planned_path_bias_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SettingsPanel::onPlannedPathBiasChanged);

    // 恢复行为连接
    connect(recovery_behavior_enabled_check_, &QCheckBox::toggled,
            this, &SettingsPanel::onRecoveryBehaviorEnabledChanged);
    connect(clearing_rotation_allowed_check_, &QCheckBox::toggled,
            this, &SettingsPanel::onClearingRotationAllowedChanged);

    // 按钮连接
    connect(apply_settings_button_, &QPushButton::clicked,
            this, &SettingsPanel::onApplySettingsClicked);
    connect(reset_settings_button_, &QPushButton::clicked,
            this, &SettingsPanel::onResetSettingsClicked);
}

void SettingsPanel::loadSettings()
{
    QSettings settings("RobotControlGUI", "Settings");

    yaw_tolerance_spin_->setValue(settings.value("yaw_tolerance", DEFAULT_YAW_TOLERANCE).toDouble());
    inflation_radius_spin_->setValue(settings.value("inflation_radius", DEFAULT_INFLATION_RADIUS).toDouble());
    transform_tolerance_spin_->setValue(settings.value("transform_tolerance", DEFAULT_TRANSFORM_TOLERANCE).toDouble());
    planner_frequency_spin_->setValue(settings.value("planner_frequency", DEFAULT_PLANNER_FREQUENCY).toDouble());
    controller_frequency_spin_->setValue(settings.value("controller_frequency", DEFAULT_CONTROLLER_FREQUENCY).toDouble());
    global_costmap_update_frequency_spin_->setValue(settings.value("global_costmap_update_frequency", DEFAULT_GLOBAL_COSTMAP_UPDATE_FREQUENCY).toDouble());
    local_costmap_update_frequency_spin_->setValue(settings.value("local_costmap_update_frequency", DEFAULT_LOCAL_COSTMAP_UPDATE_FREQUENCY).toDouble());
    planned_path_bias_spin_->setValue(settings.value("planned_path_bias", DEFAULT_PLANNED_PATH_BIAS).toDouble());
    recovery_behavior_enabled_check_->setChecked(settings.value("recovery_behavior_enabled", true).toBool());
    clearing_rotation_allowed_check_->setChecked(settings.value("clearing_rotation_allowed", true).toBool());
}

void SettingsPanel::saveSettings()
{
    QSettings settings("RobotControlGUI", "Settings");

    settings.setValue("yaw_tolerance", yaw_tolerance_spin_->value());
    settings.setValue("inflation_radius", inflation_radius_spin_->value());
    settings.setValue("transform_tolerance", transform_tolerance_spin_->value());
    settings.setValue("planner_frequency", planner_frequency_spin_->value());
    settings.setValue("controller_frequency", controller_frequency_spin_->value());
    settings.setValue("global_costmap_update_frequency", global_costmap_update_frequency_spin_->value());
    settings.setValue("local_costmap_update_frequency", local_costmap_update_frequency_spin_->value());
    settings.setValue("planned_path_bias", planned_path_bias_spin_->value());
    settings.setValue("recovery_behavior_enabled", recovery_behavior_enabled_check_->isChecked());
    settings.setValue("clearing_rotation_allowed", clearing_rotation_allowed_check_->isChecked());
}

void SettingsPanel::applySettings()
{
    if (!robot_controller_->isInitialized()) {
        QMessageBox::warning(this, tr("错误"), tr("导航系统未初始化"));
        return;
    }

    robot_controller_->setYawTolerance(yaw_tolerance_spin_->value());
    robot_controller_->setInflationRadius(inflation_radius_spin_->value());
    robot_controller_->setTransformTolerance(transform_tolerance_spin_->value());
    robot_controller_->setPlannerFrequency(planner_frequency_spin_->value());
    robot_controller_->setControllerFrequency(controller_frequency_spin_->value());
    robot_controller_->setGlobalCostmapUpdateFrequency(global_costmap_update_frequency_spin_->value());
    robot_controller_->setLocalCostmapUpdateFrequency(local_costmap_update_frequency_spin_->value());
    robot_controller_->setPlannedPathBias(planned_path_bias_spin_->value());
    robot_controller_->setRecoveryBehaviorEnabled(recovery_behavior_enabled_check_->isChecked());
    robot_controller_->setClearingRotationAllowed(clearing_rotation_allowed_check_->isChecked());

    QMessageBox::information(this, tr("成功"), tr("设置已应用"));
}

void SettingsPanel::resetSettings()
{
    yaw_tolerance_spin_->setValue(DEFAULT_YAW_TOLERANCE);
    inflation_radius_spin_->setValue(DEFAULT_INFLATION_RADIUS);
    transform_tolerance_spin_->setValue(DEFAULT_TRANSFORM_TOLERANCE);
    planner_frequency_spin_->setValue(DEFAULT_PLANNER_FREQUENCY);
    controller_frequency_spin_->setValue(DEFAULT_CONTROLLER_FREQUENCY);
    global_costmap_update_frequency_spin_->setValue(DEFAULT_GLOBAL_COSTMAP_UPDATE_FREQUENCY);
    local_costmap_update_frequency_spin_->setValue(DEFAULT_LOCAL_COSTMAP_UPDATE_FREQUENCY);
    planned_path_bias_spin_->setValue(DEFAULT_PLANNED_PATH_BIAS);
    recovery_behavior_enabled_check_->setChecked(true);
    clearing_rotation_allowed_check_->setChecked(true);
}

void SettingsPanel::onYawToleranceChanged(double value)
{
    if (robot_controller_->isInitialized()) {
        robot_controller_->setYawTolerance(value);
    }
}

void SettingsPanel::onInflationRadiusChanged(double value)
{
    if (robot_controller_->isInitialized()) {
        robot_controller_->setInflationRadius(value);
    }
}

void SettingsPanel::onTransformToleranceChanged(double value)
{
    if (robot_controller_->isInitialized()) {
        robot_controller_->setTransformTolerance(value);
    }
}

void SettingsPanel::onPlannerFrequencyChanged(double value)
{
    if (robot_controller_->isInitialized()) {
        robot_controller_->setPlannerFrequency(value);
    }
}

void SettingsPanel::onControllerFrequencyChanged(double value)
{
    if (robot_controller_->isInitialized()) {
        robot_controller_->setControllerFrequency(value);
    }
}

void SettingsPanel::onGlobalCostmapUpdateFrequencyChanged(double value)
{
    if (robot_controller_->isInitialized()) {
        robot_controller_->setGlobalCostmapUpdateFrequency(value);
    }
}

void SettingsPanel::onLocalCostmapUpdateFrequencyChanged(double value)
{
    if (robot_controller_->isInitialized()) {
        robot_controller_->setLocalCostmapUpdateFrequency(value);
    }
}

void SettingsPanel::onPlannedPathBiasChanged(double value)
{
    if (robot_controller_->isInitialized()) {
        robot_controller_->setPlannedPathBias(value);
    }
}

void SettingsPanel::onRecoveryBehaviorEnabledChanged(bool enabled)
{
    if (robot_controller_->isInitialized()) {
        robot_controller_->setRecoveryBehaviorEnabled(enabled);
    }
}

void SettingsPanel::onClearingRotationAllowedChanged(bool allowed)
{
    if (robot_controller_->isInitialized()) {
        robot_controller_->setClearingRotationAllowed(allowed);
    }
}

void SettingsPanel::onApplySettingsClicked()
{
    applySettings();
}

void SettingsPanel::onResetSettingsClicked()
{
    resetSettings();
} 