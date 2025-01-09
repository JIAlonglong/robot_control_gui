#include "ui/teleop_panel.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QMessageBox>

TeleopPanel::TeleopPanel(std::shared_ptr<RobotController> robot_controller, QWidget* parent)
    : QWidget(parent)
    , robot_controller_(robot_controller)
    , current_linear_velocity_(0.0)
    , current_angular_velocity_(0.0)
    , update_timer_(new QTimer(this))
{
    setupUi();
    
    // 设置定时器
    connect(update_timer_, &QTimer::timeout, this, &TeleopPanel::onUpdateTimeout);
    update_timer_->start(100);  // 10Hz
}

TeleopPanel::~TeleopPanel()
{
    if (robot_controller_) {
        robot_controller_->setLinearVelocity(0.0);
        robot_controller_->setAngularVelocity(0.0);
    }
}

void TeleopPanel::setupUi()
{
    QVBoxLayout* layout = new QVBoxLayout(this);
    
    // 创建按钮网格
    QGridLayout* button_grid = new QGridLayout;
    
    forward_btn_ = new QPushButton("↑", this);
    backward_btn_ = new QPushButton("↓", this);
    left_btn_ = new QPushButton("←", this);
    right_btn_ = new QPushButton("→", this);
    stop_btn_ = new QPushButton("■", this);
    
    // 设置按钮样式
    QString button_style = R"(
        QPushButton {
            font-size: 24px;
            min-width: 50px;
            min-height: 50px;
            background-color: #f0f0f0;
            border: 1px solid #cccccc;
            border-radius: 5px;
        }
        QPushButton:hover {
            background-color: #e0e0e0;
        }
        QPushButton:pressed {
            background-color: #d0d0d0;
        }
    )";
    
    forward_btn_->setStyleSheet(button_style);
    backward_btn_->setStyleSheet(button_style);
    left_btn_->setStyleSheet(button_style);
    right_btn_->setStyleSheet(button_style);
    stop_btn_->setStyleSheet(button_style + "QPushButton { color: red; }");
    
    // 添加按钮到网格
    button_grid->addWidget(forward_btn_, 0, 1);
    button_grid->addWidget(left_btn_, 1, 0);
    button_grid->addWidget(stop_btn_, 1, 1);
    button_grid->addWidget(right_btn_, 1, 2);
    button_grid->addWidget(backward_btn_, 2, 1);
    
    layout->addLayout(button_grid);
    
    // 添加速度显示
    QHBoxLayout* speed_layout = new QHBoxLayout;
    speed_layout->addWidget(new QLabel("线速度:"));
    linear_speed_label_ = new QLabel("0.0 m/s");
    speed_layout->addWidget(linear_speed_label_);
    speed_layout->addWidget(new QLabel("角速度:"));
    angular_speed_label_ = new QLabel("0.0 rad/s");
    speed_layout->addWidget(angular_speed_label_);
    
    layout->addLayout(speed_layout);
    
    // 连接信号
    connect(forward_btn_, &QPushButton::pressed, this, &TeleopPanel::onForwardButtonPressed);
    connect(backward_btn_, &QPushButton::pressed, this, &TeleopPanel::onBackwardButtonPressed);
    connect(left_btn_, &QPushButton::pressed, this, &TeleopPanel::onLeftButtonPressed);
    connect(right_btn_, &QPushButton::pressed, this, &TeleopPanel::onRightButtonPressed);
    connect(stop_btn_, &QPushButton::clicked, this, &TeleopPanel::onStopButtonClicked);
    
    connect(forward_btn_, &QPushButton::released, this, &TeleopPanel::onButtonReleased);
    connect(backward_btn_, &QPushButton::released, this, &TeleopPanel::onButtonReleased);
    connect(left_btn_, &QPushButton::released, this, &TeleopPanel::onButtonReleased);
    connect(right_btn_, &QPushButton::released, this, &TeleopPanel::onButtonReleased);
}

void TeleopPanel::onForwardButtonPressed()
{
    current_linear_velocity_ = robot_controller_->getMaxLinearVelocity();
    current_angular_velocity_ = 0.0;
}

void TeleopPanel::onBackwardButtonPressed()
{
    current_linear_velocity_ = -robot_controller_->getMaxLinearVelocity();
    current_angular_velocity_ = 0.0;
}

void TeleopPanel::onLeftButtonPressed()
{
    current_linear_velocity_ = 0.0;
    current_angular_velocity_ = robot_controller_->getMaxAngularVelocity();
}

void TeleopPanel::onRightButtonPressed()
{
    current_linear_velocity_ = 0.0;
    current_angular_velocity_ = -robot_controller_->getMaxAngularVelocity();
}

void TeleopPanel::onStopButtonClicked()
{
    current_linear_velocity_ = 0.0;
    current_angular_velocity_ = 0.0;
    robot_controller_->setLinearVelocity(0.0);
    robot_controller_->setAngularVelocity(0.0);
}

void TeleopPanel::onButtonReleased()
{
    current_linear_velocity_ = 0.0;
    current_angular_velocity_ = 0.0;
}

void TeleopPanel::onUpdateTimeout()
{
    robot_controller_->setLinearVelocity(current_linear_velocity_);
    robot_controller_->setAngularVelocity(current_angular_velocity_);
    
    // 更新速度显示
    linear_speed_label_->setText(QString("%1 m/s").arg(current_linear_velocity_, 0, 'f', 2));
    angular_speed_label_->setText(QString("%1 rad/s").arg(current_angular_velocity_, 0, 'f', 2));
} 