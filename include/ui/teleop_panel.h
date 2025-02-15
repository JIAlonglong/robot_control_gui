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

#ifndef TELEOP_PANEL_H
#define TELEOP_PANEL_H

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <memory>
#include "ros/robot_controller.h"

class QTimer;

class TeleopPanel : public QWidget {
    Q_OBJECT

public:
    explicit TeleopPanel(std::shared_ptr<RobotController> robot_controller,
                        QWidget* parent = nullptr);
    ~TeleopPanel();

private Q_SLOTS:
    void onForwardButtonPressed();
    void onBackwardButtonPressed();
    void onLeftButtonPressed();
    void onRightButtonPressed();
    void onStopButtonClicked();
    void onButtonReleased();
    void onUpdateTimeout();

private:
    void setupUi();

    std::shared_ptr<RobotController> robot_controller_;
    
    // UI组件
    QPushButton* forward_btn_;
    QPushButton* backward_btn_;
    QPushButton* left_btn_;
    QPushButton* right_btn_;
    QPushButton* stop_btn_;
    QLabel* linear_speed_label_;
    QLabel* angular_speed_label_;
    
    // 控制相关
    QTimer* update_timer_;
    double current_linear_velocity_;
    double current_angular_velocity_;
};

#endif // TELEOP_PANEL_H 