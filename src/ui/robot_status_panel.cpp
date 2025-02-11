/**
 * @file robot_status_panel.cpp
 * @brief 机器人状态显示面板实现
 */

#include "ui/robot_status_panel.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>

RobotStatusPanel::RobotStatusPanel(QWidget* parent)
    : QWidget(parent)
    , battery_bar_(new QProgressBar(this))
    , wifi_bar_(new QProgressBar(this))
    , status_label_(new QLabel(this))
    , battery_label_(new QLabel(this))
    , wifi_label_(new QLabel(this))
    , is_connected_(false)
{
    // 创建主布局
    QVBoxLayout* main_layout = new QVBoxLayout(this);
    
    // 创建状态组
    QGroupBox* status_group = new QGroupBox(tr("机器人状态"), this);
    QVBoxLayout* status_layout = new QVBoxLayout(status_group);
    
    // 创建状态标签
    status_label_ = new QLabel(tr("状态: 就绪"), this);
    status_label_->setStyleSheet("font-weight: bold;");
    
    // 创建电池状态
    QWidget* battery_widget = new QWidget(this);
    QHBoxLayout* battery_layout = new QHBoxLayout(battery_widget);
    battery_label_ = new QLabel(tr("电池电量:"), this);
    battery_bar_ = new QProgressBar(this);
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
    battery_layout->addWidget(battery_label_);
    battery_layout->addWidget(battery_bar_);
    
    // 创建WiFi状态
    QWidget* wifi_widget = new QWidget(this);
    QHBoxLayout* wifi_layout = new QHBoxLayout(wifi_widget);
    wifi_label_ = new QLabel(tr("WiFi信号:"), this);
    wifi_bar_ = new QProgressBar(this);
    wifi_bar_->setRange(0, 100);
    wifi_bar_->setValue(100);
    wifi_bar_->setStyleSheet(R"(
        QProgressBar {
            border: 1px solid #ccc;
            border-radius: 5px;
            text-align: center;
        }
        QProgressBar::chunk {
            background-color: #2196F3;
            border-radius: 5px;
        }
    )");
    wifi_layout->addWidget(wifi_label_);
    wifi_layout->addWidget(wifi_bar_);
    
    // 添加到状态布局
    status_layout->addWidget(status_label_);
    status_layout->addWidget(battery_widget);
    status_layout->addWidget(wifi_widget);
    
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

void RobotStatusPanel::updateBatteryLevel(double level)
{
    battery_bar_->setValue(static_cast<int>(level));
    
    // 根据电量设置颜色
    QString color;
    if (level > 60) color = "#4CAF50";  // 绿色
    else if (level > 20) color = "#FFC107";  // 黄色
    else color = "#F44336";  // 红色
    
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
    )").arg(color));
}

void RobotStatusPanel::updateWifiStrength(int strength)
{
    wifi_bar_->setValue(strength);
    
    // 根据信号强度设置颜色
    QString color;
    if (strength > 60) color = "#2196F3";  // 蓝色
    else if (strength > 20) color = "#FFC107";  // 黄色
    else color = "#F44336";  // 红色
    
    wifi_bar_->setStyleSheet(QString(R"(
        QProgressBar {
            border: 1px solid #ccc;
            border-radius: 5px;
            text-align: center;
        }
        QProgressBar::chunk {
            background-color: %1;
            border-radius: 5px;
        }
    )").arg(color));
}

void RobotStatusPanel::updateStatus(const QString& status)
{
    status_label_->setText(tr("状态: %1").arg(status));
}

double RobotStatusPanel::getBatteryLevel() const {
    return battery_bar_->value();
}

bool RobotStatusPanel::isConnected() const {
    return is_connected_;
}

void RobotStatusPanel::setConnected(bool connected) {
    is_connected_ = connected;
    updateStatus(connected ? "已连接" : "未连接");
} 