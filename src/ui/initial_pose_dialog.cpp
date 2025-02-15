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

#include "ui/initial_pose_dialog.h"
#include <QVBoxLayout>
#include <QFormLayout>
#include <QDoubleSpinBox>
#include <QDialogButtonBox>
#include <QPushButton>
#include <QLabel>
#include <cmath>
#include <ros/ros.h>

InitialPoseDialog::InitialPoseDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(tr("设置初始位姿"));
    setupUI();
}

void InitialPoseDialog::setupUI()
{
    auto* layout = new QVBoxLayout(this);
    auto* form_layout = new QFormLayout;

    // 创建输入控件
    x_spinbox_ = new QDoubleSpinBox(this);
    y_spinbox_ = new QDoubleSpinBox(this);
    yaw_spinbox_ = new QDoubleSpinBox(this);

    // 设置范围和精度
    x_spinbox_->setRange(-100.0, 100.0);
    y_spinbox_->setRange(-100.0, 100.0);
    yaw_spinbox_->setRange(-180.0, 180.0);
    x_spinbox_->setDecimals(3);
    y_spinbox_->setDecimals(3);
    yaw_spinbox_->setDecimals(1);

    // 添加到表单布局
    form_layout->addRow(tr("X 坐标 (米):"), x_spinbox_);
    form_layout->addRow(tr("Y 坐标 (米):"), y_spinbox_);
    form_layout->addRow(tr("偏航角 (度):"), yaw_spinbox_);

    // 创建按钮
    button_box_ = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
        Qt::Horizontal, this);
    connect(button_box_, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(button_box_, &QDialogButtonBox::rejected, this, &QDialog::reject);

    layout->addLayout(form_layout);
    layout->addWidget(button_box_);
}

geometry_msgs::PoseWithCovarianceStamped InitialPoseDialog::getPose() const
{
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();

    // 设置位置
    pose.pose.pose.position.x = x_spinbox_->value();
    pose.pose.pose.position.y = y_spinbox_->value();
    pose.pose.pose.position.z = 0.0;

    // 从偏航角计算四元数
    double yaw = yaw_spinbox_->value() * M_PI / 180.0;
    pose.pose.pose.orientation.x = 0.0;
    pose.pose.pose.orientation.y = 0.0;
    pose.pose.pose.orientation.z = sin(yaw / 2.0);
    pose.pose.pose.orientation.w = cos(yaw / 2.0);

    // 设置协方差
    for (int i = 0; i < 36; ++i) {
        pose.pose.covariance[i] = 0.0;
    }
    pose.pose.covariance[0] = 0.25;   // x
    pose.pose.covariance[7] = 0.25;   // y
    pose.pose.covariance[35] = 0.068; // yaw

    return pose;
} 