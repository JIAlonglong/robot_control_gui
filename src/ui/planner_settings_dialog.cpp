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

#include "ui/planner_settings_dialog.h"
#include <QGroupBox>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <QDialogButtonBox>

PlannerSettingsDialog::PlannerSettingsDialog(std::shared_ptr<RobotController> robot_controller, QWidget* parent)
    : QDialog(parent)
    , robot_controller_(robot_controller)
{
    setWindowTitle("路径规划器设置");
    setupUi();
    connectSignalsAndSlots();
}

void PlannerSettingsDialog::setupUi()
{
    auto* main_layout = new QVBoxLayout(this);
    
    // 全局规划器设置
    auto* global_planner_group = new QGroupBox("全局路径规划器", this);
    auto* global_planner_layout = new QVBoxLayout(global_planner_group);
    
    auto* global_planner_label = new QLabel("选择全局规划器:", this);
    global_planner_combo_ = new QComboBox(this);
    global_planner_description_ = new QLabel(this);
    global_planner_description_->setWordWrap(true);
    
    // 添加可用的全局规划器
    auto global_planners = robot_controller_->getAvailableGlobalPlanners();
    for (const auto& planner : global_planners) {
        global_planner_combo_->addItem(planner);
    }
    
    global_planner_layout->addWidget(global_planner_label);
    global_planner_layout->addWidget(global_planner_combo_);
    global_planner_layout->addWidget(global_planner_description_);
    
    // 局部规划器设置
    auto* local_planner_group = new QGroupBox("局部路径规划器", this);
    auto* local_planner_layout = new QVBoxLayout(local_planner_group);
    
    auto* local_planner_label = new QLabel("选择局部规划器:", this);
    local_planner_combo_ = new QComboBox(this);
    local_planner_description_ = new QLabel(this);
    local_planner_description_->setWordWrap(true);
    
    // 添加可用的局部规划器
    auto local_planners = robot_controller_->getAvailableLocalPlanners();
    for (const auto& planner : local_planners) {
        local_planner_combo_->addItem(planner);
    }
    
    local_planner_layout->addWidget(local_planner_label);
    local_planner_layout->addWidget(local_planner_combo_);
    local_planner_layout->addWidget(local_planner_description_);
    
    // 添加确定和取消按钮
    button_box_ = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
        Qt::Horizontal, this);
    
    main_layout->addWidget(global_planner_group);
    main_layout->addWidget(local_planner_group);
    main_layout->addWidget(button_box_);
    
    // 设置当前选中的规划器
    int global_index = global_planner_combo_->findText(robot_controller_->getCurrentGlobalPlanner());
    if (global_index >= 0) {
        global_planner_combo_->setCurrentIndex(global_index);
    }
    
    int local_index = local_planner_combo_->findText(robot_controller_->getCurrentLocalPlanner());
    if (local_index >= 0) {
        local_planner_combo_->setCurrentIndex(local_index);
    }
    
    updatePlannerDescriptions();
}

void PlannerSettingsDialog::connectSignalsAndSlots()
{
    connect(global_planner_combo_, QOverload<const QString&>::of(&QComboBox::currentTextChanged),
            this, &PlannerSettingsDialog::onGlobalPlannerChanged);
            
    connect(local_planner_combo_, QOverload<const QString&>::of(&QComboBox::currentTextChanged),
            this, &PlannerSettingsDialog::onLocalPlannerChanged);
            
    connect(button_box_, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(button_box_, &QDialogButtonBox::rejected, this, &QDialog::reject);
    
    connect(this, &QDialog::accepted, this, &PlannerSettingsDialog::onAccepted);
    connect(this, &QDialog::rejected, this, &PlannerSettingsDialog::onRejected);
}

void PlannerSettingsDialog::onGlobalPlannerChanged(const QString& planner_name)
{
    if (robot_controller_) {
        robot_controller_->setGlobalPlanner(planner_name);
    }
    updatePlannerDescriptions();
}

void PlannerSettingsDialog::onLocalPlannerChanged(const QString& planner_name)
{
    if (robot_controller_) {
        robot_controller_->setLocalPlanner(planner_name);
    }
    updatePlannerDescriptions();
}

void PlannerSettingsDialog::onAccepted()
{
    // 保存设置
    if (robot_controller_) {
        robot_controller_->setGlobalPlanner(global_planner_combo_->currentText());
        robot_controller_->setLocalPlanner(local_planner_combo_->currentText());
    }
}

void PlannerSettingsDialog::onRejected()
{
    // 恢复原来的设置
    if (robot_controller_) {
        robot_controller_->setGlobalPlanner(robot_controller_->getCurrentGlobalPlanner());
        robot_controller_->setLocalPlanner(robot_controller_->getCurrentLocalPlanner());
    }
}

void PlannerSettingsDialog::updatePlannerDescriptions()
{
    QString global_planner = global_planner_combo_->currentText();
    QString local_planner = local_planner_combo_->currentText();
    
    global_planner_description_->setText(getGlobalPlannerDescription(global_planner));
    local_planner_description_->setText(getLocalPlannerDescription(local_planner));
}

QString PlannerSettingsDialog::getGlobalPlannerDescription(const QString& planner_name) const
{
    if (planner_name == "navfn/NavfnROS") {
        return "默认全局规划器，使用 Dijkstra 算法计算最短路径，适合一般导航场景。";
    } else if (planner_name == "global_planner/GlobalPlanner") {
        return "改进的 A* 规划器，支持多种启发式方法，可以处理更复杂的环境。";
    } else if (planner_name == "carrot_planner/CarrotPlanner") {
        return "简单的直线规划器，适合开阔、无障碍的简单环境。";
    }
    return "";
}

QString PlannerSettingsDialog::getLocalPlannerDescription(const QString& planner_name) const
{
    if (planner_name == "base_local_planner/TrajectoryPlannerROS") {
        return "传统的 DWA 规划器，稳定可靠，适合一般场景。";
    } else if (planner_name == "dwa_local_planner/DWAPlannerROS") {
        return "改进的 DWA 规划器，生成更平滑的路径，对动态障碍物响应更好。";
    } else if (planner_name == "teb_local_planner/TebLocalPlannerROS") {
        return "时间弹性带规划器，可以处理各种运动约束，适合动态环境。";
    } else if (planner_name == "eband_local_planner/EBandPlannerROS") {
        return "弹性带规划器，生成平滑路径，计算效率高。";
    }
    return "";
} 