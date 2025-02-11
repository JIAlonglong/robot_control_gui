/**
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
 * 
 * @file goal_setting_dialog.cpp
 * @brief 未知组件
 * @author JIAlonglong
 */

#include "goal_setting_dialog.h"
#include <tf2/utils.h>
#include <QGridLayout>
#include <cmath>

GoalSettingDialog::GoalSettingDialog(const geometry_msgs::Pose& current_pose, QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle("设置导航目标点");
    setupUI();

    // 使用当前位姿初始化输入框
    x_spin_->setValue(current_pose.position.x);
    y_spin_->setValue(current_pose.position.y);
    yaw_spin_->setValue(tf2::getYaw(current_pose.orientation) * 180.0 / M_PI);
}

void GoalSettingDialog::setupUI()
{
    auto* layout = new QGridLayout(this);

    // X坐标
    layout->addWidget(new QLabel("X坐标 (米):", this), 0, 0);
    x_spin_ = new QDoubleSpinBox(this);
    x_spin_->setRange(-100.0, 100.0);
    x_spin_->setDecimals(3);
    layout->addWidget(x_spin_, 0, 1);

    // Y坐标
    layout->addWidget(new QLabel("Y坐标 (米):", this), 1, 0);
    y_spin_ = new QDoubleSpinBox(this);
    y_spin_->setRange(-100.0, 100.0);
    y_spin_->setDecimals(3);
    layout->addWidget(y_spin_, 1, 1);

    // 朝向角度
    layout->addWidget(new QLabel("朝向 (度):", this), 2, 0);
    yaw_spin_ = new QDoubleSpinBox(this);
    yaw_spin_->setRange(-180.0, 180.0);
    yaw_spin_->setDecimals(1);
    layout->addWidget(yaw_spin_, 2, 1);

    // 按钮
    auto* button_layout = new QHBoxLayout;
    auto* ok_button     = new QPushButton("确定", this);
    auto* cancel_button = new QPushButton("取消", this);
    button_layout->addWidget(ok_button);
    button_layout->addWidget(cancel_button);
    layout->addLayout(button_layout, 3, 0, 1, 2);

    connect(ok_button, &QPushButton::clicked, this, &QDialog::accept);
    connect(cancel_button, &QPushButton::clicked, this, &QDialog::reject);
}

void GoalSettingDialog::updateGoal()
{
    // 这个函数暂时不需要实现，因为我们在 getGoal() 中直接构造目标点
}

geometry_msgs::PoseStamped GoalSettingDialog::getGoal() const
{
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp    = ros::Time::now();

    goal.pose.position.x = x_spin_->value();
    goal.pose.position.y = y_spin_->value();
    goal.pose.position.z = 0.0;

    // 将角度转换为四元数
    double          yaw = yaw_spin_->value() * M_PI / 180.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    goal.pose.orientation.x = q.x();
    goal.pose.orientation.y = q.y();
    goal.pose.orientation.z = q.z();
    goal.pose.orientation.w = q.w();

    return goal;
}