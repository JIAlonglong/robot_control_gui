/**
 * @file robot_status_panel.cpp
 * @brief 机器人状态显示面板实现
 */

#include "ui/robot_status_panel.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFrame>

RobotStatusPanel::RobotStatusPanel(QWidget* parent)
    : QWidget(parent)
    , battery_label_(new QLabel("Battery:", this))
    , battery_progress_(new QProgressBar(this))
    , velocity_label_(new QLabel("Velocity: 0.0 m/s, 0.0 rad/s", this))
    , mode_label_(new QLabel("Mode: Standby", this))
{
    setupUI();
}

void RobotStatusPanel::setupUI()
{
    QVBoxLayout* main_layout = new QVBoxLayout(this);
    main_layout->setSpacing(20);  // 增加组件间距
    main_layout->setContentsMargins(20, 20, 20, 20);  // 增加边距
    
    // Battery status
    QHBoxLayout* battery_layout = new QHBoxLayout();
    battery_layout->setSpacing(10);
    
    battery_label_->setText("Battery");  // 移除冒号，更简洁
    battery_label_->setStyleSheet(R"(
        QLabel {
            color: #2c3e50;
            font-size: 14pt;
            font-weight: 500;
        }
    )");
    
    battery_progress_->setRange(0, 100);
    battery_progress_->setTextVisible(true);
    battery_progress_->setValue(100);
    battery_progress_->setStyleSheet(R"(
        QProgressBar {
            text-align: center;
            min-height: 25px;
            max-height: 25px;
            border: none;
            border-radius: 12px;
            background-color: #f5f6fa;
            font-size: 12pt;
            font-weight: bold;
        }
        QProgressBar::chunk {
            border-radius: 12px;
            background-color: #2ecc71;
        }
    )");
    
    battery_layout->addWidget(battery_label_);
    battery_layout->addWidget(battery_progress_, 1);  // 进度条占据更多空间
    
    // Velocity and Mode section
    velocity_label_->setText("Velocity: 0.0 m/s, 0.0 rad/s");
    velocity_label_->setStyleSheet(R"(
        QLabel {
            color: #2c3e50;
            font-size: 14pt;
            font-weight: 500;
            padding: 10px;
            background-color: #f5f6fa;
            border-radius: 10px;
        }
    )");
    
    mode_label_->setText("Mode: Standby");
    mode_label_->setStyleSheet(R"(
        QLabel {
            color: #2c3e50;
            font-size: 14pt;
            font-weight: 500;
            padding: 10px;
            background-color: #f5f6fa;
            border-radius: 10px;
        }
    )");
    
    // Add to main layout
    main_layout->addLayout(battery_layout);
    main_layout->addWidget(velocity_label_);
    main_layout->addWidget(mode_label_);
    main_layout->addStretch();
    
    // Set panel style
    setStyleSheet(R"(
        RobotStatusPanel {
            background-color: white;
            border-radius: 15px;
            border: 1px solid #e0e0e0;
        }
    )");
}

void RobotStatusPanel::updateBatteryLevel(int percentage)
{
    battery_progress_->setValue(percentage);
    
    QString color;
    if (percentage < 20) {
        color = "#e74c3c";  // 红色
    } else if (percentage < 50) {
        color = "#f1c40f";  // 黄色
    } else {
        color = "#2ecc71";  // 绿色
    }
    
    battery_progress_->setStyleSheet(QString(R"(
        QProgressBar {
            text-align: center;
            min-height: 25px;
            max-height: 25px;
            border: none;
            border-radius: 12px;
            background-color: #f5f6fa;
            font-size: 12pt;
            font-weight: bold;
        }
        QProgressBar::chunk {
            border-radius: 12px;
            background-color: %1;
        }
    )").arg(color));
}

void RobotStatusPanel::updateVelocity(double linear_x, double angular_z)
{
    QString text = QString("Velocity: %1 m/s, %2 rad/s")
        .arg(QString::number(linear_x, 'f', 2))
        .arg(QString::number(angular_z, 'f', 2));
    velocity_label_->setText(text);
}

void RobotStatusPanel::updateWorkMode(const QString& mode)
{
    QString text = QString("Mode: %1").arg(mode);
    mode_label_->setText(text);
    
    // 根据模式设置不同的背景色
    QString bgColor = (mode == "Auto") ? "#e8f5e9" : "#fff3e0";
    QString textColor = (mode == "Auto") ? "#2e7d32" : "#e65100";
    
    mode_label_->setStyleSheet(QString(R"(
        QLabel {
            color: %1;
            font-size: 14pt;
            font-weight: 500;
            padding: 10px;
            background-color: %2;
            border-radius: 10px;
        }
    )").arg(textColor, bgColor));
} 