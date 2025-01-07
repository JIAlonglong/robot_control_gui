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
#include <QInputDialog>

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
    QVBoxLayout* main_layout = new QVBoxLayout(this);
    main_layout->setSpacing(10);

    // 创建目标点设置组
    QGroupBox* goal_group = new QGroupBox(tr("目标点设置"));
    QGridLayout* goal_layout = new QGridLayout(goal_group);

    x_input_ = new QLineEdit(goal_group);
    y_input_ = new QLineEdit(goal_group);
    theta_input_ = new QLineEdit(goal_group);
    
    // 设置数值验证器
    QDoubleValidator* validator = new QDoubleValidator(this);
    x_input_->setValidator(validator);
    y_input_->setValidator(validator);
    theta_input_->setValidator(validator);

    goal_layout->addWidget(new QLabel(tr("X (米):")), 0, 0);
    goal_layout->addWidget(x_input_, 0, 1);
    goal_layout->addWidget(new QLabel(tr("Y (米):")), 1, 0);
    goal_layout->addWidget(y_input_, 1, 1);
    goal_layout->addWidget(new QLabel(tr("角度 (弧度):")), 2, 0);
    goal_layout->addWidget(theta_input_, 2, 1);

    // 创建路径规划设置组
    QGroupBox* planner_group = new QGroupBox(tr("路径规划设置"));
    QVBoxLayout* planner_layout = new QVBoxLayout(planner_group);

    // 添加规划器选择
    QHBoxLayout* planner_select_layout = new QHBoxLayout();
    planner_select_layout->addWidget(new QLabel(tr("规划器:")));
    planner_type_ = new QComboBox();
    planner_type_->addItem(tr("Dijkstra"), "dijkstra");
    planner_type_->addItem(tr("A*"), "astar");
    planner_type_->addItem(tr("RRT"), "rrt");
    planner_type_->addItem(tr("RRT*"), "rrtstar");
    planner_select_layout->addWidget(planner_type_);
    planner_layout->addLayout(planner_select_layout);

    // 添加启发式函数选择
    QHBoxLayout* heuristic_layout = new QHBoxLayout();
    heuristic_layout->addWidget(new QLabel(tr("启发式函数:")));
    heuristic_type_ = new QComboBox();
    heuristic_type_->addItem(tr("欧几里得距离"), "euclidean");
    heuristic_type_->addItem(tr("曼哈顿距离"), "manhattan");
    heuristic_type_->addItem(tr("对角线距离"), "diagonal");
    heuristic_layout->addWidget(heuristic_type_);
    planner_layout->addLayout(heuristic_layout);

    // 添加规划参数设置
    QGridLayout* params_layout = new QGridLayout();
    
    planning_time_ = new QLineEdit("5.0");
    planning_time_->setValidator(new QDoubleValidator(0.1, 60.0, 1, this));
    params_layout->addWidget(new QLabel(tr("规划时间限制(秒):")), 0, 0);
    params_layout->addWidget(planning_time_, 0, 1);

    interpolation_distance_ = new QLineEdit("0.01");
    interpolation_distance_->setValidator(new QDoubleValidator(0.01, 1.0, 3, this));
    params_layout->addWidget(new QLabel(tr("路径插值距离(米):")), 1, 0);
    params_layout->addWidget(interpolation_distance_, 1, 1);

    planner_layout->addLayout(params_layout);

    // 添加高级选项
    QHBoxLayout* advanced_layout = new QHBoxLayout();
    allow_unknown_ = new QCheckBox(tr("允许未知区域"));
    allow_unknown_->setChecked(false);
    advanced_layout->addWidget(allow_unknown_);
    
    visualize_planning_ = new QCheckBox(tr("可视化规划过程"));
    visualize_planning_->setChecked(true);
    advanced_layout->addWidget(visualize_planning_);
    
    planner_layout->addLayout(advanced_layout);

    // 创建导航模式选择
    QHBoxLayout* mode_layout = new QHBoxLayout();
    mode_layout->addWidget(new QLabel(tr("导航模式:")));
    navigation_mode_ = new QComboBox();
    navigation_mode_->addItem(tr("默认导航"));
    navigation_mode_->addItem(tr("快速导航"));
    navigation_mode_->addItem(tr("精确导航"));
    mode_layout->addWidget(navigation_mode_);

    // 创建控制按钮
    QHBoxLayout* btn_layout = new QHBoxLayout();
    set_goal_btn_ = new QPushButton(tr("设置目标"));
    cancel_btn_ = new QPushButton(tr("取消导航"));
    btn_layout->addWidget(set_goal_btn_);
    btn_layout->addWidget(cancel_btn_);

    // 创建导航状态显示
    QGroupBox* status_group = new QGroupBox(tr("导航状态"));
    QVBoxLayout* status_layout = new QVBoxLayout(status_group);
    
    status_label_ = new QLabel(tr("就绪"));
    status_label_->setAlignment(Qt::AlignCenter);
    status_layout->addWidget(status_label_);
    
    progress_bar_ = new QProgressBar();
    progress_bar_->setRange(0, 100);
    progress_bar_->setValue(0);
    progress_bar_->setFormat(tr("进度: %p%"));
    status_layout->addWidget(progress_bar_);
    
    distance_label_ = new QLabel(tr("距离: -- 米"));
    distance_label_->setAlignment(Qt::AlignCenter);
    status_layout->addWidget(distance_label_);
    
    eta_label_ = new QLabel(tr("预计时间: -- 秒"));
    eta_label_->setAlignment(Qt::AlignCenter);
    status_layout->addWidget(eta_label_);

    // 组装布局
    main_layout->addWidget(goal_group);
    main_layout->addWidget(planner_group);
    main_layout->addLayout(mode_layout);
    main_layout->addLayout(btn_layout);
    main_layout->addWidget(status_group);
    main_layout->addStretch();

    // 连接信号
    connect(set_goal_btn_, &QPushButton::clicked, this, &NavigationPanel::onSetGoalClicked);
    connect(cancel_btn_, &QPushButton::clicked, this, &NavigationPanel::onCancelNavigationClicked);
    connect(navigation_mode_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &NavigationPanel::onNavigationModeChanged);
            
    // 连接RobotController的导航状态信号
    connect(robot_controller_.get(), &RobotController::navigationStateChanged,
            this, &NavigationPanel::updateNavigationState,
            Qt::QueuedConnection);
    connect(robot_controller_.get(), &RobotController::navigationProgressChanged,
            this, &NavigationPanel::updateNavigationProgress,
            Qt::QueuedConnection);
    connect(robot_controller_.get(), &RobotController::navigationFeedback,
            this, &NavigationPanel::updateNavigationStatus,
            Qt::QueuedConnection);
    connect(planner_type_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &NavigationPanel::onPlannerTypeChanged);
    connect(heuristic_type_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &NavigationPanel::onHeuristicTypeChanged);
    connect(planning_time_, &QLineEdit::textChanged,
            this, &NavigationPanel::onPlanningTimeChanged);
    connect(interpolation_distance_, &QLineEdit::textChanged,
            this, &NavigationPanel::onInterpolationDistanceChanged);
    connect(allow_unknown_, &QCheckBox::stateChanged,
            this, &NavigationPanel::onAllowUnknownChanged);
    connect(visualize_planning_, &QCheckBox::stateChanged,
            this, &NavigationPanel::onVisualizePlanningChanged);
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
    set_goal_btn_->setEnabled(has_map_ && !is_mapping_ && !is_navigating_);
    cancel_btn_->setEnabled(is_navigating_);
    navigation_mode_->setEnabled(!is_navigating_);
    
    // 根据导航状态更新进度条样式
    QString progress_style;
    switch (current_state_) {
        case NavigationState::SUCCEEDED:
            progress_style = "QProgressBar { text-align: center; } "
                           "QProgressBar::chunk { background-color: #2ecc71; }";
            break;
        case NavigationState::FAILED:
            progress_style = "QProgressBar { text-align: center; } "
                           "QProgressBar::chunk { background-color: #e74c3c; }";
            break;
        case NavigationState::CANCELED:
            progress_style = "QProgressBar { text-align: center; } "
                           "QProgressBar::chunk { background-color: #f1c40f; }";
            break;
        default:
            progress_style = "QProgressBar { text-align: center; } "
                           "QProgressBar::chunk { background-color: #3498db; }";
            break;
    }
    progress_bar_->setStyleSheet(progress_style);
}

void NavigationPanel::updateNavigationState(NavigationState state)
{
    current_state_ = state;
    is_navigating_ = (state != NavigationState::IDLE && 
                     state != NavigationState::SUCCEEDED &&
                     state != NavigationState::FAILED &&
                     state != NavigationState::CANCELED);
    
    // 更新按钮状态
    updateButtonStates();
    
    // 更新状态显示样式
    QString style;
    switch (state) {
        case NavigationState::SUCCEEDED:
            style = "QLabel { color: green; font-weight: bold; }";
            break;
        case NavigationState::FAILED:
            style = "QLabel { color: red; font-weight: bold; }";
            break;
        case NavigationState::CANCELED:
            style = "QLabel { color: orange; font-weight: bold; }";
            break;
        default:
            style = "QLabel { color: black; }";
            break;
    }
    status_label_->setStyleSheet(style);
    
    // 发送状态变化信号
    emit navigationStateChanged(is_navigating_);
}

void NavigationPanel::updateNavigationProgress(double progress)
{
    progress_bar_->setValue(static_cast<int>(progress * 100));
}

void NavigationPanel::updateNavigationStatus(const QString& status)
{
    status_label_->setText(status);
    
    // 从状态文本中提取距离信息（如果有）
    QRegExp rx("距离目标点还有 (\\d+\\.?\\d*) 米");
    if (rx.indexIn(status) != -1) {
        double distance = rx.cap(1).toDouble();
        distance_label_->setText(tr("距离: %.2f 米").arg(distance));
        
        // 估算到达时间（假设平均速度为0.2m/s）
        double eta = distance / 0.2;
        eta_label_->setText(tr("预计时间: %1 秒").arg(static_cast<int>(eta)));
    }
}

void NavigationPanel::updateMapState(bool has_map)
{
    has_map_ = has_map;
    updateButtonStates();
    emit mapUpdated();
}

void NavigationPanel::updatePoseEstimate(double x, double y, double theta)
{
    x_input_->setText(QString::number(x, 'f', 2));
    y_input_->setText(QString::number(y, 'f', 2));
    theta_input_->setText(QString::number(theta, 'f', 2));
}

void NavigationPanel::onSetGoalClicked()
{
    bool ok;
    double x = x_input_->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::warning(this, "错误", "X坐标格式不正确");
        return;
    }
    
    double y = y_input_->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::warning(this, "错误", "Y坐标格式不正确");
        return;
    }
    
    double theta = theta_input_->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::warning(this, "错误", "角度格式不正确");
        return;
    }

    if (robot_controller_->setNavigationGoal(x, y, theta)) {
        is_navigating_ = true;
        updateButtonStates();
        status_label_->setText("正在导航...");
        emit navigationGoalSet(x, y, theta);
    } else {
        QMessageBox::warning(this, "错误", "设置导航目标失败");
    }
}

void NavigationPanel::onCancelNavigationClicked()
{
    robot_controller_->cancelNavigation();
    is_navigating_ = false;
    updateButtonStates();
    status_label_->setText("导航已取消");
    emit navigationStateChanged(false);
}

void NavigationPanel::onStartMappingClicked()
{
    if (robot_controller_->startMapping()) {
        is_mapping_ = true;
        updateButtonStates();
        status_label_->setText("正在建图...");
    } else {
        QMessageBox::warning(this, "错误", "启动建图失败");
    }
}

void NavigationPanel::onStopMappingClicked()
{
    if (robot_controller_->stopMapping()) {
        is_mapping_ = false;
        updateButtonStates();
        status_label_->setText("建图已停止");
    } else {
        QMessageBox::warning(this, "错误", "停止建图失败");
    }
}

void NavigationPanel::onSaveMapClicked()
{
    QString filename = QFileDialog::getSaveFileName(this,
        "保存地图", "", "地图文件 (*.yaml);;所有文件 (*)");
    
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
    QString filename = QFileDialog::getOpenFileName(this,
        "加载地图", "", "地图文件 (*.yaml);;所有文件 (*)");
    
    if (!filename.isEmpty()) {
        if (robot_controller_->loadMap(filename.toStdString())) {
            has_map_ = true;
            updateButtonStates();
            QMessageBox::information(this, "成功", "地图加载成功");
            emit mapUpdated();
        } else {
            QMessageBox::warning(this, "错误", "加载地图失败");
        }
    }
}

void NavigationPanel::onInitialPoseClicked()
{
    bool ok;
    double x = QInputDialog::getDouble(this, "设置初始位姿",
        "X坐标 (米):", 0.0, -1000.0, 1000.0, 2, &ok);
    if (!ok) return;
    
    double y = QInputDialog::getDouble(this, "设置初始位姿",
        "Y坐标 (米):", 0.0, -1000.0, 1000.0, 2, &ok);
    if (!ok) return;
    
    double theta = QInputDialog::getDouble(this, "设置初始位姿",
        "角度 (弧度):", 0.0, -M_PI, M_PI, 2, &ok);
    if (!ok) return;

    if (robot_controller_->setInitialPose(x, y, theta)) {
        QMessageBox::information(this, "成功", "初始位姿设置成功");
    } else {
        QMessageBox::warning(this, "错误", "设置初始位姿失败");
    }
}

void NavigationPanel::onNavigationModeChanged(int index)
{
    robot_controller_->setNavigationMode(index);
}

void NavigationPanel::onCostmapUpdateClicked()
{
    if (robot_controller_->updateCostmap()) {
        QMessageBox::information(this, "成功", "代价地图更新成功");
    } else {
        QMessageBox::warning(this, "错误", "更新代价地图失败");
    }
}

void NavigationPanel::onPlannerTypeChanged(int index)
{
    QString planner = planner_type_->currentData().toString();
    robot_controller_->setParam("/move_base/base_global_planner", planner);
    
    // 更新启发式函数选择的可用性
    heuristic_type_->setEnabled(planner == "astar");
}

void NavigationPanel::onHeuristicTypeChanged(int index)
{
    QString heuristic = heuristic_type_->currentData().toString();
    robot_controller_->setParam("/move_base/global_planner/heuristic_function", heuristic);
}

void NavigationPanel::onPlanningTimeChanged(const QString& text)
{
    bool ok;
    double time = text.toDouble(&ok);
    if (ok) {
        robot_controller_->setParam("/move_base/planner_patience", time);
    }
}

void NavigationPanel::onInterpolationDistanceChanged(const QString& text)
{
    bool ok;
    double distance = text.toDouble(&ok);
    if (ok) {
        robot_controller_->setParam("/move_base/global_planner/interpolation_distance", distance);
    }
}

void NavigationPanel::onAllowUnknownChanged(int state)
{
    robot_controller_->setParam("/move_base/global_planner/allow_unknown", state == Qt::Checked);
}

void NavigationPanel::onVisualizePlanningChanged(int state)
{
    robot_controller_->setParam("/move_base/global_planner/visualize", state == Qt::Checked);
} 