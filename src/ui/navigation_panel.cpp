/**
 * @file navigation_panel.cpp
 * @brief 导航控制面板类的实现
 */

#include "ui/navigation_panel.h"
#include "ros/robot_controller.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFileDialog>
#include <QMessageBox>
#include <QTabWidget>
#include <QGroupBox>
#include <QDoubleValidator>

NavigationPanel::NavigationPanel(std::shared_ptr<RobotController> robot_controller, QWidget* parent)
    : QWidget(parent)
    , robot_controller_(robot_controller)
    , is_mapping_(false)
    , has_map_(false)
    , is_navigating_(false)
{
    createUI();
    setupStyle();
    updateButtonStates();
}

void NavigationPanel::createUI()
{
    // 创建主布局
    QVBoxLayout* main_layout = new QVBoxLayout(this);
    
    // 创建标签页
    tab_widget_ = new QTabWidget(this);
    createNavigationTab();
    createMappingTab();
    createSettingsTab();
    
    main_layout->addWidget(tab_widget_);
}

void NavigationPanel::createNavigationTab()
{
    QWidget* nav_tab = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(nav_tab);

    // 创建目标点设置组
    QGroupBox* goal_group = new QGroupBox("目标点设置", nav_tab);
    QGridLayout* goal_layout = new QGridLayout(goal_group);

    x_input_ = new QLineEdit(goal_group);
    y_input_ = new QLineEdit(goal_group);
    theta_input_ = new QLineEdit(goal_group);
    
    // 设置数值验证器
    QDoubleValidator* validator = new QDoubleValidator(this);
    x_input_->setValidator(validator);
    y_input_->setValidator(validator);
    theta_input_->setValidator(validator);

    goal_layout->addWidget(new QLabel("X (米):"), 0, 0);
    goal_layout->addWidget(x_input_, 0, 1);
    goal_layout->addWidget(new QLabel("Y (米):"), 1, 0);
    goal_layout->addWidget(y_input_, 1, 1);
    goal_layout->addWidget(new QLabel("角度 (弧度):"), 2, 0);
    goal_layout->addWidget(theta_input_, 2, 1);

    // 创建导航模式选择
    QHBoxLayout* mode_layout = new QHBoxLayout();
    mode_layout->addWidget(new QLabel("导航模式:"));
    navigation_mode_ = new QComboBox(nav_tab);
    navigation_mode_->addItem("默认导航");
    navigation_mode_->addItem("快速导航");
    navigation_mode_->addItem("精确导航");
    mode_layout->addWidget(navigation_mode_);

    // 创建控制按钮
    QHBoxLayout* btn_layout = new QHBoxLayout();
    set_goal_btn_ = new QPushButton("设置目标", nav_tab);
    cancel_btn_ = new QPushButton("取消导航", nav_tab);
    btn_layout->addWidget(set_goal_btn_);
    btn_layout->addWidget(cancel_btn_);

    // 创建状态显示
    status_label_ = new QLabel("就绪", nav_tab);
    status_label_->setAlignment(Qt::AlignCenter);

    // 组装布局
    layout->addWidget(goal_group);
    layout->addLayout(mode_layout);
    layout->addLayout(btn_layout);
    layout->addWidget(status_label_);
    layout->addStretch();

    // 连接信号
    connect(set_goal_btn_, &QPushButton::clicked, this, &NavigationPanel::onSetGoalClicked);
    connect(cancel_btn_, &QPushButton::clicked, this, &NavigationPanel::onCancelNavigationClicked);
    connect(navigation_mode_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &NavigationPanel::onNavigationModeChanged);

    tab_widget_->addTab(nav_tab, "导航控制");
}

void NavigationPanel::createMappingTab()
{
    QWidget* map_tab = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(map_tab);

    // 创建建图控制组
    QGroupBox* mapping_group = new QGroupBox("建图控制", map_tab);
    QVBoxLayout* mapping_layout = new QVBoxLayout(mapping_group);

    start_mapping_btn_ = new QPushButton("开始建图", mapping_group);
    stop_mapping_btn_ = new QPushButton("停止建图", mapping_group);
    mapping_status_ = new QLabel("未开始建图", mapping_group);
    mapping_status_->setAlignment(Qt::AlignCenter);

    mapping_layout->addWidget(start_mapping_btn_);
    mapping_layout->addWidget(stop_mapping_btn_);
    mapping_layout->addWidget(mapping_status_);

    // 创建地图操作组
    QGroupBox* map_ops_group = new QGroupBox("地图操作", map_tab);
    QVBoxLayout* map_ops_layout = new QVBoxLayout(map_ops_group);

    save_map_btn_ = new QPushButton("保存地图", map_ops_group);
    load_map_btn_ = new QPushButton("加载地图", map_ops_group);
    set_initial_pose_btn_ = new QPushButton("设置初始位姿", map_ops_group);

    map_ops_layout->addWidget(save_map_btn_);
    map_ops_layout->addWidget(load_map_btn_);
    map_ops_layout->addWidget(set_initial_pose_btn_);

    // 组装布局
    layout->addWidget(mapping_group);
    layout->addWidget(map_ops_group);
    layout->addStretch();

    // 连接信号
    connect(start_mapping_btn_, &QPushButton::clicked, this, &NavigationPanel::onStartMappingClicked);
    connect(stop_mapping_btn_, &QPushButton::clicked, this, &NavigationPanel::onStopMappingClicked);
    connect(save_map_btn_, &QPushButton::clicked, this, &NavigationPanel::onSaveMapClicked);
    connect(load_map_btn_, &QPushButton::clicked, this, &NavigationPanel::onLoadMapClicked);
    connect(set_initial_pose_btn_, &QPushButton::clicked, this, &NavigationPanel::onInitialPoseClicked);

    tab_widget_->addTab(map_tab, "建图");
}

void NavigationPanel::createSettingsTab()
{
    QWidget* settings_tab = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(settings_tab);

    // 创建导航参数组
    QGroupBox* nav_params_group = new QGroupBox("导航参数", settings_tab);
    QGridLayout* params_layout = new QGridLayout(nav_params_group);

    max_vel_linear_ = new QLineEdit(nav_params_group);
    max_vel_angular_ = new QLineEdit(nav_params_group);
    min_obstacle_dist_ = new QLineEdit(nav_params_group);
    
    // 设置数值验证器
    QDoubleValidator* validator = new QDoubleValidator(0.0, 10.0, 2, this);
    max_vel_linear_->setValidator(validator);
    max_vel_angular_->setValidator(validator);
    min_obstacle_dist_->setValidator(validator);

    params_layout->addWidget(new QLabel("最大线速度 (m/s):"), 0, 0);
    params_layout->addWidget(max_vel_linear_, 0, 1);
    params_layout->addWidget(new QLabel("最大角速度 (rad/s):"), 1, 0);
    params_layout->addWidget(max_vel_angular_, 1, 1);
    params_layout->addWidget(new QLabel("最小避障距离 (m):"), 2, 0);
    params_layout->addWidget(min_obstacle_dist_, 2, 1);

    // 创建其他设置组
    QGroupBox* other_settings_group = new QGroupBox("其他设置", settings_tab);
    QVBoxLayout* other_layout = new QVBoxLayout(other_settings_group);

    enable_recovery_ = new QCheckBox("启用恢复行为", other_settings_group);
    update_costmap_btn_ = new QPushButton("更新代价地图", other_settings_group);

    other_layout->addWidget(enable_recovery_);
    other_layout->addWidget(update_costmap_btn_);

    // 组装布局
    layout->addWidget(nav_params_group);
    layout->addWidget(other_settings_group);
    layout->addStretch();

    // 连接信号
    connect(update_costmap_btn_, &QPushButton::clicked, this, &NavigationPanel::onCostmapUpdateClicked);

    tab_widget_->addTab(settings_tab, "设置");
}

void NavigationPanel::setupStyle()
{
    setStyleSheet(R"(
        QGroupBox {
            font-weight: bold;
            border: 1px solid #cccccc;
            border-radius: 6px;
            margin-top: 6px;
            padding-top: 10px;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 7px;
            padding: 0px 5px 0px 5px;
        }
        QLineEdit {
            padding: 5px;
            border: 1px solid #cccccc;
            border-radius: 3px;
        }
        QPushButton {
            padding: 8px;
            margin: 2px;
            background-color: #f0f0f0;
            border: 1px solid #cccccc;
            border-radius: 3px;
        }
        QPushButton:hover {
            background-color: #e0e0e0;
        }
        QLabel {
            margin: 2px;
        }
    )");
}

void NavigationPanel::updateButtonStates()
{
    // 更新导航相关按钮状态
    set_goal_btn_->setEnabled(has_map_ && !is_mapping_);
    cancel_btn_->setEnabled(is_navigating_);
    navigation_mode_->setEnabled(!is_navigating_);

    // 更新建图相关按钮状态
    start_mapping_btn_->setEnabled(!is_mapping_ && !is_navigating_);
    stop_mapping_btn_->setEnabled(is_mapping_);
    save_map_btn_->setEnabled(has_map_);
    load_map_btn_->setEnabled(!is_mapping_);
    set_initial_pose_btn_->setEnabled(has_map_ && !is_mapping_);

    // 更新设置相关按钮状态
    update_costmap_btn_->setEnabled(has_map_);
    max_vel_linear_->setEnabled(!is_navigating_);
    max_vel_angular_->setEnabled(!is_navigating_);
    min_obstacle_dist_->setEnabled(!is_navigating_);
    enable_recovery_->setEnabled(!is_navigating_);
}

void NavigationPanel::updateNavigationState()
{
    is_navigating_ = robot_controller_->isNavigating();
    status_label_->setText(is_navigating_ ? "正在导航..." : "就绪");
    updateButtonStates();
    emit navigationStateChanged(is_navigating_);
}

void NavigationPanel::updateMapState(bool has_map)
{
    has_map_ = has_map;
    updateButtonStates();
    emit mapUpdated();
}

void NavigationPanel::updatePoseEstimate(double x, double y, double theta)
{
    // 更新位姿估计显示
    QString pose_text = QString("当前位置: (%.2f, %.2f, %.2f)").arg(x).arg(y).arg(theta);
    status_label_->setText(pose_text);
}

void NavigationPanel::onSetGoalClicked()
{
    bool ok;
    double x = x_input_->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::warning(this, "错误", "X坐标格式无效");
        return;
    }
    
    double y = y_input_->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::warning(this, "错误", "Y坐标格式无效");
        return;
    }
    
    double theta = theta_input_->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::warning(this, "错误", "角度格式无效");
        return;
    }
    
    if (robot_controller_->setNavigationGoal(x, y, theta)) {
        is_navigating_ = true;
        status_label_->setText("正在导航...");
        updateButtonStates();
        emit navigationStateChanged(true);
    } else {
        QMessageBox::warning(this, "错误", "设置导航目标失败");
    }
}

void NavigationPanel::onCancelNavigationClicked()
{
    robot_controller_->cancelNavigation();
    is_navigating_ = false;
    status_label_->setText("导航已取消");
    updateButtonStates();
    emit navigationStateChanged(false);
}

void NavigationPanel::onSaveMapClicked()
{
    QString filename = QFileDialog::getSaveFileName(
        this,
        "保存地图",
        QString(),
        "地图文件 (*.pgm *.yaml);;所有文件 (*.*)"
    );
    
    if (!filename.isEmpty()) {
        if (robot_controller_->saveMap(filename.toStdString())) {
            QMessageBox::information(this, "成功", "地图保存成功");
        } else {
            QMessageBox::warning(this, "错误", "保存地图失败");
        }
    }
}

void NavigationPanel::onLoadMapClicked()
{
    QString filename = QFileDialog::getOpenFileName(
        this,
        "加载地图",
        QString(),
        "地图文件 (*.pgm *.yaml);;所有文件 (*.*)"
    );
    
    if (!filename.isEmpty()) {
        if (robot_controller_->loadMap(filename.toStdString())) {
            has_map_ = true;
            QMessageBox::information(this, "成功", "地图加载成功");
            updateButtonStates();
            emit mapUpdated();
        } else {
            QMessageBox::warning(this, "错误", "加载地图失败");
        }
    }
}

void NavigationPanel::onStartMappingClicked()
{
    if (robot_controller_->startMapping()) {
        is_mapping_ = true;
        mapping_status_->setText("正在建图...");
        updateButtonStates();
    } else {
        QMessageBox::warning(this, "错误", "启动建图失败");
    }
}

void NavigationPanel::onStopMappingClicked()
{
    if (robot_controller_->stopMapping()) {
        is_mapping_ = false;
        has_map_ = true;
        mapping_status_->setText("建图完成");
        updateButtonStates();
        emit mapUpdated();
    } else {
        QMessageBox::warning(this, "错误", "停止建图失败");
    }
}

void NavigationPanel::onInitialPoseClicked()
{
    bool ok;
    double x = x_input_->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::warning(this, "错误", "X坐标格式无效");
        return;
    }
    
    double y = y_input_->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::warning(this, "错误", "Y坐标格式无效");
        return;
    }
    
    double theta = theta_input_->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::warning(this, "错误", "角度格式无效");
        return;
    }
    
    if (robot_controller_->setInitialPose(x, y, theta)) {
        QMessageBox::information(this, "成功", "初始位姿设置成功");
    } else {
        QMessageBox::warning(this, "错误", "设置初始位姿失败");
    }
}

void NavigationPanel::onNavigationModeChanged(int index)
{
    if (robot_controller_->setNavigationMode(index)) {
        QMessageBox::information(this, "成功", "导航模式切换成功");
    } else {
        QMessageBox::warning(this, "错误", "导航模式切换失败");
        // 恢复之前的选择
        navigation_mode_->setCurrentIndex(robot_controller_->getNavigationMode());
    }
}

void NavigationPanel::onCostmapUpdateClicked()
{
    if (robot_controller_->updateCostmap()) {
        QMessageBox::information(this, "成功", "代价地图更新成功");
    } else {
        QMessageBox::warning(this, "错误", "代价地图更新失败");
    }
} 