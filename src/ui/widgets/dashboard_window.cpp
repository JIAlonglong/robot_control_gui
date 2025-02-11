/**
 * Copyright (c) 2024 JIAlonglong
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
 * @file dashboard_window.cpp
 * @brief 仪表盘窗口类的实现,用于显示机器人的状态信息
 * @author JIAlonglong
 */

#include "dashboard_window.h"
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>

DashboardWindow::DashboardWindow(QWidget* parent)
    : QWidget(parent)
    , speed_label_(new QLabel(this))
    , battery_bar_(new QProgressBar(this))
    , status_label_(new QLabel(this))
{
    // 创建主布局
    QVBoxLayout* main_layout = new QVBoxLayout(this);

    // 创建状态组
    QGroupBox*   status_group  = new QGroupBox(tr("仪表盘"), this);
    QVBoxLayout* status_layout = new QVBoxLayout(status_group);

    // 创建速度显示
    speed_label_->setText(tr("速度: 0.0 m/s, 0.0 rad/s"));
    speed_label_->setStyleSheet("font-weight: bold;");

    // 创建电池状态
    QWidget*     battery_widget = new QWidget(this);
    QHBoxLayout* battery_layout = new QHBoxLayout(battery_widget);
    QLabel*      battery_label  = new QLabel(tr("电池电量:"), this);
    battery_bar_->setRange(0, 100);
    battery_bar_->setValue(100);
    battery_bar_->setStyleSheet(R"(
        QProgressBar {
            border: 1px solid #ccc;
            border-radius: 5px;
            text-align: center;
        }
        QProgressBar::chunk {
            background-color: #4CAF50;
            border-radius: 5px;
        }
    )");
    battery_layout->addWidget(battery_label);
    battery_layout->addWidget(battery_bar_);

    // 创建状态显示
    status_label_->setText(tr("状态: 就绪"));
    status_label_->setStyleSheet("font-weight: bold;");

    // 添加到状态布局
    status_layout->addWidget(speed_label_);
    status_layout->addWidget(battery_widget);
    status_layout->addWidget(status_label_);

    // 添加到主布局
    main_layout->addWidget(status_group);
    main_layout->addStretch();

    // 设置样式
    setStyleSheet(R"(
        QGroupBox {
            border: 1px solid #ccc;
            border-radius: 5px;
            margin-top: 1ex;
            padding: 10px;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 3px;
        }
        QLabel {
            color: #333;
        }
    )");
}

void DashboardWindow::updateSpeed(double linear_speed, double angular_speed)
{
    speed_label_->setText(tr("速度: %.2f m/s, %.2f rad/s").arg(linear_speed).arg(angular_speed));
}

void DashboardWindow::updateBatteryLevel(double level)
{
    battery_bar_->setValue(static_cast<int>(level));

    // 根据电量设置颜色
    QString color;
    if (level > 60)
        color = "#4CAF50";  // 绿色
    else if (level > 20)
        color = "#FFC107";  // 黄色
    else
        color = "#F44336";  // 红色

    battery_bar_->setStyleSheet(QString(R"(
        QProgressBar {
            border: 1px solid #ccc;
            border-radius: 5px;
            text-align: center;
        }
        QProgressBar::chunk {
            background-color: %1;
            border-radius: 5px;
        }
    )")
                                    .arg(color));
}

void DashboardWindow::updateStatus(const QString& status)
{
    status_label_->setText(tr("状态: %1").arg(status));
}