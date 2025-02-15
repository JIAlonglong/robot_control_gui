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

#include "ui/mapping_panel.h"
#include "ui/rviz_view.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>
#include <QProgressBar>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QMessageBox>
#include <QFileDialog>

struct MappingPanel::Private
{
    std::shared_ptr<RobotController> robot_controller;
    
    // UI元素
    QComboBox* mapping_method_combo{nullptr};
    QPushButton* start_button{nullptr};
    QPushButton* stop_button{nullptr};
    QPushButton* save_button{nullptr};
    QLabel* status_label{nullptr};
    QProgressBar* progress_bar{nullptr};
    QLabel* method_description_label{nullptr};
    
    // 参数控件
    QDoubleSpinBox* resolution_spin{nullptr};
    QSpinBox* particles_spin{nullptr};
    QDoubleSpinBox* update_interval_spin{nullptr};
    QDoubleSpinBox* map_update_interval_spin{nullptr};
    
    bool is_mapping{false};
};

MappingPanel::MappingPanel(std::shared_ptr<RobotController> robot_controller, QWidget* parent)
    : QWidget(parent)
    , d(std::make_unique<Private>())
{
    d->robot_controller = std::move(robot_controller);
    setupUI();
    connectSignals();
}

MappingPanel::~MappingPanel()
{
    // 智能指针会自动清理 Private 结构体
}

void MappingPanel::setupUI()
{
    auto* main_layout = new QHBoxLayout(this);  // 水平布局
    main_layout->setContentsMargins(5, 5, 5, 5);
    main_layout->setSpacing(5);
    
    // 左侧控制面板
    auto* left_panel = new QWidget(this);
    auto* left_layout = new QVBoxLayout(left_panel);
    left_layout->setContentsMargins(0, 0, 0, 0);
    left_layout->setSpacing(10);
    
    // 添加各个控制组
    setupMappingGroup();        // 建图控制组
    setupParametersGroup();     // 参数设置组
    setupJoystickGroup();      // 摇杆控制组
    
    left_layout->addWidget(mapping_group_);
    left_layout->addWidget(parameters_group_);
    left_layout->addWidget(joystick_group_);
    left_layout->addStretch();
    
    // 右侧RViz显示
    setupRVizGroup();          // RViz显示组
    
    // 设置左右面板的比例（1:2）
    main_layout->addWidget(left_panel);
    main_layout->addWidget(rviz_group_);
}

void MappingPanel::setupMappingGroup()
{
    mapping_group_ = new QGroupBox(tr("建图控制"), this);
    auto* layout = new QVBoxLayout(mapping_group_);
    
    // 建图方法选择
    auto* method_layout = new QHBoxLayout;
    auto* method_label = new QLabel(tr("建图方法:"), this);
    d->mapping_method_combo = new QComboBox(this);
    d->mapping_method_combo->addItems({
        "Gmapping SLAM",
        "Cartographer SLAM",
        "Hector SLAM",
        "手动遥控建图"
    });
    method_layout->addWidget(method_label);
    method_layout->addWidget(d->mapping_method_combo);
    
    // 建图方法描述
    d->method_description_label = new QLabel(this);
    d->method_description_label->setWordWrap(true);
    d->method_description_label->setStyleSheet("color: gray;");
    
    // 控制按钮
    auto* button_layout = new QHBoxLayout;
    d->start_button = new QPushButton(tr("开始建图"), this);
    d->stop_button = new QPushButton(tr("停止建图"), this);
    d->save_button = new QPushButton(tr("保存地图"), this);
    d->stop_button->setEnabled(false);
    d->save_button->setEnabled(false);
    button_layout->addWidget(d->start_button);
    button_layout->addWidget(d->stop_button);
    button_layout->addWidget(d->save_button);
    
    // 状态显示
    d->status_label = new QLabel(tr("状态: 就绪"), this);
    d->progress_bar = new QProgressBar(this);
    d->progress_bar->setRange(0, 100);
    
    layout->addLayout(method_layout);
    layout->addWidget(d->method_description_label);
    layout->addLayout(button_layout);
    layout->addWidget(d->status_label);
    layout->addWidget(d->progress_bar);
    
    updateMethodDescription();
}

void MappingPanel::setupParametersGroup()
{
    parameters_group_ = new QGroupBox(tr("建图参数"), this);
    auto* layout = new QGridLayout(parameters_group_);
    
    // 地图分辨率
    auto* resolution_label = new QLabel(tr("地图分辨率(m):"), this);
    d->resolution_spin = new QDoubleSpinBox(this);
    d->resolution_spin->setRange(0.01, 0.1);
    d->resolution_spin->setSingleStep(0.01);
    d->resolution_spin->setValue(0.05);
    
    // 更新间隔
    auto* update_interval_label = new QLabel(tr("更新间隔(s):"), this);
    d->update_interval_spin = new QDoubleSpinBox(this);
    d->update_interval_spin->setRange(0.1, 5.0);
    d->update_interval_spin->setSingleStep(0.1);
    d->update_interval_spin->setValue(1.0);
    
    // 地图更新间隔
    auto* map_update_label = new QLabel(tr("地图更新间隔(s):"), this);
    d->map_update_interval_spin = new QDoubleSpinBox(this);
    d->map_update_interval_spin->setRange(0.1, 10.0);
    d->map_update_interval_spin->setSingleStep(0.1);
    d->map_update_interval_spin->setValue(5.0);
    
    layout->addWidget(resolution_label, 0, 0);
    layout->addWidget(d->resolution_spin, 0, 1);
    layout->addWidget(update_interval_label, 1, 0);
    layout->addWidget(d->update_interval_spin, 1, 1);
    layout->addWidget(map_update_label, 2, 0);
    layout->addWidget(d->map_update_interval_spin, 2, 1);
}

QString MappingPanel::getMappingMethodDescription(const QString& method) const
{
    if (method == "Gmapping SLAM") {
        return tr("基于粒子滤波的经典SLAM方法，适合室内环境，计算量较小");
    } else if (method == "Cartographer SLAM") {
        return tr("Google开发的高精度SLAM系统，支持2D/3D建图，计算量较大");
    } else if (method == "Hector SLAM") {
        return tr("无需里程计的SLAM方法，适合小型轻量级机器人");
    } else if (method == "手动遥控建图") {
        return tr("通过手动遥控机器人运动来构建地图，适合特殊场景");
    }
    return "";
}

void MappingPanel::onStartMapping()
{
    if (!d->robot_controller) return;
    
    QString method = d->mapping_method_combo->currentText();
    
    // 尝试启动建图
    try {
        d->robot_controller->startMapping(method);
        
        d->start_button->setEnabled(false);
        d->stop_button->setEnabled(true);
        d->save_button->setEnabled(false);
        d->mapping_method_combo->setEnabled(false);
        
        // 通知 RVizView 开始建图
        if (rviz_view_) {
            rviz_view_->showMappingView(true);
        }
    } catch (const std::exception& e) {
        QMessageBox::critical(this, tr("建图错误"),
            tr("启动建图失败: %1").arg(e.what()));
        return;
    }
}

void MappingPanel::onStopMapping()
{
    if (!d->robot_controller) return;
    
    d->robot_controller->stopMapping();
    
    d->start_button->setEnabled(true);
    d->stop_button->setEnabled(false);
    d->save_button->setEnabled(true);
    d->mapping_method_combo->setEnabled(true);
    
    // 通知 RVizView 停止建图
    if (rviz_view_) {
        rviz_view_->showMappingView(false);
    }
}

void MappingPanel::onSaveMap()
{
    QString filename = QFileDialog::getSaveFileName(this,
        tr("保存地图"), "",
        tr("地图文件 (*.pgm *.yaml)"));
    
    if (filename.isEmpty()) return;
    
    if (!filename.endsWith(".pgm")) {
        filename += ".pgm";
    }
    
    d->robot_controller->saveMap(filename);
}

void MappingPanel::connectSignals()
{
    connect(d->mapping_method_combo, &QComboBox::currentTextChanged,
            this, &MappingPanel::onMappingMethodChanged);
    connect(d->start_button, &QPushButton::clicked,
            this, &MappingPanel::onStartMapping);
    connect(d->stop_button, &QPushButton::clicked,
            this, &MappingPanel::onStopMapping);
    connect(d->save_button, &QPushButton::clicked,
            this, &MappingPanel::onSaveMap);
            
    if (d->robot_controller) {
        connect(d->robot_controller.get(), &RobotController::mappingProgressChanged,
                this, &MappingPanel::onMappingProgressChanged);
        connect(d->robot_controller.get(), &RobotController::mappingStatusChanged,
                this, &MappingPanel::onMappingStatusChanged);
    }
}

void MappingPanel::updateMethodDescription()
{
    if (!d->method_description_label || !d->mapping_method_combo) return;
    
    QString method = d->mapping_method_combo->currentText();
    d->method_description_label->setText(getMappingMethodDescription(method));
}

void MappingPanel::onMappingMethodChanged(const QString& method)
{
    updateMethodDescription();
    
    // 更新参数控件的值
    if (d && d->update_interval_spin) {  // 添加空指针检查
        if (method == "Gmapping SLAM") {
            d->update_interval_spin->setValue(1.0);
        } else if (method == "Cartographer SLAM") {
            d->update_interval_spin->setValue(0.5);
        } else if (method == "Hector SLAM") {
            d->update_interval_spin->setValue(0.1);
        }
    }
    
    updateMappingParameters();
}

void MappingPanel::onMappingProgressChanged(double progress)
{
    if (!d->progress_bar) return;
    d->progress_bar->setValue(static_cast<int>(progress));
}

void MappingPanel::onMappingStatusChanged(const QString& status)
{
    if (!d->status_label) return;
    d->status_label->setText(status);
}

void MappingPanel::updateMappingParameters()
{
    if (!d || !d->robot_controller) return;
    
    // 获取当前参数值（添加空指针检查）
    double resolution = d->resolution_spin ? d->resolution_spin->value() : 0.05;
    double update_interval = d->update_interval_spin ? d->update_interval_spin->value() : 1.0;
    double map_update_interval = d->map_update_interval_spin ? d->map_update_interval_spin->value() : 5.0;
    
    // 获取当前选择的建图方法
    QString method = d->mapping_method_combo ? d->mapping_method_combo->currentText() : QString();
    
    try {
        // 创建参数映射
        std::map<std::string, double> params = {
            {"resolution", resolution},
            {"update_interval", update_interval},
            {"map_update_interval", map_update_interval}
        };

        // 根据不同的建图方法添加特定参数
        if (method == "Gmapping SLAM") {
            // 可以添加 Gmapping 特有的参数
            params["particles"] = 30.0;  // 默认粒子数
        } else if (method == "Cartographer SLAM") {
            // 可以添加 Cartographer 特有的参数
        } else if (method == "Hector SLAM") {
            // 可以添加 Hector SLAM 特有的参数
        }

        // 更新参数
        d->robot_controller->updateMappingParameters(params);
        
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to update mapping parameters: %s", e.what());
    }
}

void MappingPanel::setupJoystickGroup()
{
    joystick_group_ = new QGroupBox(tr("遥控控制"), this);
    auto* layout = new QHBoxLayout(joystick_group_);
    
    // 创建线速度摇杆
    auto* linear_group = new QGroupBox(tr("线速度控制"), this);
    auto* linear_layout = new QVBoxLayout(linear_group);
    linear_joystick_ = new JoystickWidget(this);
    linear_layout->addWidget(linear_joystick_);
    
    // 创建角速度摇杆
    auto* angular_group = new QGroupBox(tr("角速度控制"), this);
    auto* angular_layout = new QVBoxLayout(angular_group);
    angular_joystick_ = new JoystickWidget(this);
    angular_layout->addWidget(angular_joystick_);
    
    layout->addWidget(linear_group);
    layout->addWidget(angular_group);
    
    // 连接摇杆信号 - 使用正确的信号名称
    connect(linear_joystick_, &JoystickWidget::moved,
            this, &MappingPanel::onLinearJoystickMoved);
    connect(angular_joystick_, &JoystickWidget::moved,
            this, &MappingPanel::onAngularJoystickMoved);
}

void MappingPanel::setupRVizGroup()
{
    rviz_group_ = new QGroupBox(tr("RViz 视图"), this);
    auto* layout = new QVBoxLayout(rviz_group_);
    layout->setContentsMargins(0, 0, 0, 0);
    
    if (rviz_view_) {
        layout->addWidget(rviz_view_);
        
        // 连接地图更新信号
        if (d->robot_controller) {
            connect(d->robot_controller.get(), &RobotController::mapUpdated,
                    rviz_view_, &RVizView::updateMappingDisplay,
                    Qt::QueuedConnection);
        }
    }
}

// 添加摇杆控制槽函数
void MappingPanel::onLinearJoystickMoved(double x, double y)
{
    if (!d->robot_controller) return;
    
    // 将y轴值映射到线速度（向上为正）
    double linear_vel = -y * d->robot_controller->getMaxLinearVelocity();
    d->robot_controller->setLinearVelocity(linear_vel);
}

void MappingPanel::onAngularJoystickMoved(double x, double y)
{
    if (!d->robot_controller) return;
    
    // 将x轴值映射到角速度（向右为负）
    double angular_vel = -x * d->robot_controller->getMaxAngularVelocity();
    d->robot_controller->setAngularVelocity(angular_vel);
}

void MappingPanel::setRVizView(RVizView* view)
{
    rviz_view_ = view;
} 