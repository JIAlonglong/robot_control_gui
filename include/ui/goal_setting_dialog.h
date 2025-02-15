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

#ifndef GOAL_SETTING_DIALOG_H
#define GOAL_SETTING_DIALOG_H

#include <QDialog>
#include <QDoubleSpinBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

class GoalSettingDialog : public QDialog
{
    Q_OBJECT

public:
    explicit GoalSettingDialog(const geometry_msgs::Pose& current_pose, QWidget* parent = nullptr);
    ~GoalSettingDialog() = default;

    geometry_msgs::PoseStamped getGoal() const;

private:
    QDoubleSpinBox* x_spin_;
    QDoubleSpinBox* y_spin_;
    QDoubleSpinBox* yaw_spin_;
    geometry_msgs::PoseStamped goal_;

    void setupUI();
    void updateGoal();
};

#endif // GOAL_SETTING_DIALOG_H 