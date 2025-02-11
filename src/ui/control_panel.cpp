#include "ui/control_panel.h"
#include "ui/joystick_widget.h"
#include "ui/speed_dashboard.h"
#include "ros/robot_controller.h"
#include <QCamera>
#include <QCameraViewfinder>
#include <QCameraImageCapture>
#include <QSlider>
#include <QStyle>
#include <QStyleOption>
#include <QDir>
#include <QComboBox>
#include <QMessageBox>

ControlPanel::ControlPanel(const std::shared_ptr<RobotController>& robot_controller,
                         QWidget* parent)
    : QWidget(parent)
    , d_ptr(std::make_unique<ControlPanelPrivate>())
{
    d_ptr->robot_controller = robot_controller;
    
    // 初始化UI
    setupUi();
    setupCamera();
    setupJoystick();
    connectSignals();
    
    // 创建状态更新定时器
    d_ptr->status_timer = new QTimer(this);
    connect(d_ptr->status_timer, &QTimer::timeout, this, &ControlPanel::updateRobotStatus);
    d_ptr->status_timer->start(1000);  // 每秒更新一次状态
}

ControlPanel::~ControlPanel() = default;

void ControlPanel::setupUi()
{
    d_ptr->main_layout = new QVBoxLayout(this);
    d_ptr->main_layout->setContentsMargins(10, 10, 10, 10);
    d_ptr->main_layout->setSpacing(10);

    // 创建上下两个部分的布局
    auto* top_layout = new QHBoxLayout();
    auto* bottom_layout = new QHBoxLayout();
    
    // 设置样式
    setStyleSheet(
        "QGroupBox {"
        "    border: 1px solid #cccccc;"
        "    border-radius: 6px;"
        "    margin-top: 1em;"
        "    padding: 10px;"
        "    background-color: #ffffff;"
        "}"
        "QGroupBox::title {"
        "    subcontrol-origin: margin;"
        "    left: 10px;"
        "    padding: 0 3px 0 3px;"
        "    color: #333333;"
        "}"
        "QPushButton {"
        "    border: 1px solid #cccccc;"
        "    border-radius: 4px;"
        "    padding: 8px 16px;"
        "    background-color: #ffffff;"
        "    color: #333333;"
        "}"
        "QPushButton:hover {"
        "    background-color: #f5f5f5;"
        "}"
        "QPushButton:pressed {"
        "    background-color: #e5e5e5;"
        "}"
        "QLabel {"
        "    color: #333333;"
        "    padding: 2px;"
        "}"
    );

    // 创建状态组
    d_ptr->status_group = new QGroupBox(tr("机器人状态"), this);
    auto* status_layout = new QGridLayout(d_ptr->status_group);
    
    d_ptr->battery_label = new QLabel(tr("电池电量: --"), this);
    d_ptr->voltage_label = new QLabel(tr("电压: -- V"), this);
    d_ptr->current_label = new QLabel(tr("电流: -- A"), this);
    d_ptr->temperature_label = new QLabel(tr("温度: -- ℃"), this);
    d_ptr->motor_status_label = new QLabel(tr("电机状态: --"), this);
    d_ptr->connection_status_label = new QLabel(tr("连接状态: --"), this);
    
    status_layout->addWidget(d_ptr->battery_label, 0, 0);
    status_layout->addWidget(d_ptr->voltage_label, 0, 1);
    status_layout->addWidget(d_ptr->current_label, 1, 0);
    status_layout->addWidget(d_ptr->temperature_label, 1, 1);
    status_layout->addWidget(d_ptr->motor_status_label, 2, 0);
    status_layout->addWidget(d_ptr->connection_status_label, 2, 1);

    // 创建速度控制组
    d_ptr->speed_control_group = new QGroupBox(tr("速度控制"), this);
    auto* speed_layout = new QHBoxLayout(d_ptr->speed_control_group);
    speed_layout->setSpacing(20);  // 增加组件之间的间距
    
    // 添加到布局
    top_layout->addWidget(d_ptr->status_group);
    bottom_layout->addWidget(d_ptr->speed_control_group);
    
    d_ptr->main_layout->addLayout(top_layout);
    d_ptr->main_layout->addLayout(bottom_layout);
}

void ControlPanel::setupCamera()
{
    // 创建相机组
    d_ptr->camera_group = new QGroupBox(tr("相机视图"), this);
    auto* camera_layout = new QVBoxLayout(d_ptr->camera_group);
    
    // 创建相机取景器
    d_ptr->viewfinder = new QCameraViewfinder(d_ptr->camera_group);
    d_ptr->viewfinder->setMinimumSize(320, 240);
    camera_layout->addWidget(d_ptr->viewfinder);
    
    // 创建相机设备选择下拉框
    auto* device_layout = new QHBoxLayout();
    auto* device_label = new QLabel(tr("摄像头设备:"), d_ptr->camera_group);
    auto* device_combo = new QComboBox(d_ptr->camera_group);
    device_combo->setMinimumWidth(150);
    
    // 扫描可用的视频设备
    QDir dev_dir("/dev");
    QStringList filters;
    filters << "video*";  // 视频设备
    QStringList video_devices = dev_dir.entryList(filters, QDir::System);
    
    if (video_devices.isEmpty()) {
        device_combo->addItem(tr("未检测到摄像头"));
        device_combo->setEnabled(false);
    } else {
        for (const QString& device : video_devices) {
            device_combo->addItem("/dev/" + device);
        }
    }
    
    device_layout->addWidget(device_label);
    device_layout->addWidget(device_combo);
    device_layout->addStretch();
    
    // 创建相机控制按钮
    d_ptr->camera_toggle_button = new QPushButton(tr("开启相机"), d_ptr->camera_group);
    d_ptr->camera_toggle_button->setEnabled(!video_devices.isEmpty());
    
    // 添加到布局
    camera_layout->addLayout(device_layout);
    camera_layout->addWidget(d_ptr->camera_toggle_button);
    
    // 将相机组添加到主布局的顶部
    if (auto* top_layout = qobject_cast<QHBoxLayout*>(d_ptr->main_layout->itemAt(0)->layout())) {
        top_layout->insertWidget(0, d_ptr->camera_group, 2);  // 相机视图占2/3
    }
    
    // 连接设备选择信号
    connect(device_combo, QOverload<const QString&>::of(&QComboBox::currentTextChanged),
            this, [this](const QString& device) {
                if (d_ptr->camera) {
                    d_ptr->camera->stop();
                    delete d_ptr->camera;
                }
                
                if (device.startsWith("/dev/")) {
                    d_ptr->camera = new QCamera(device.toUtf8(), this);
                    d_ptr->camera->setViewfinder(d_ptr->viewfinder);
                    
                    // 连接错误信号
                    connect(d_ptr->camera, QOverload<QCamera::Error>::of(&QCamera::error),
                            this, [this](QCamera::Error error) {
                                QString error_msg;
                                switch (error) {
                                    case QCamera::NoError:
                                        return;
                                    case QCamera::CameraError:
                                        error_msg = tr("相机错误：无法访问相机");
                                        break;
                                    case QCamera::InvalidRequestError:
                                        error_msg = tr("相机错误：无效的请求");
                                        break;
                                    case QCamera::ServiceMissingError:
                                        error_msg = tr("相机错误：相机服务不可用");
                                        break;
                                    case QCamera::NotSupportedFeatureError:
                                        error_msg = tr("相机错误：不支持的功能");
                                        break;
                                    default:
                                        error_msg = tr("相机错误：未知错误");
                                        break;
                                }
                                QMessageBox::warning(this, tr("相机错误"), error_msg);
                                d_ptr->camera_toggle_button->setText(tr("开启相机"));
                                d_ptr->camera_toggle_button->setEnabled(true);
                            });
                }
            });
}

void ControlPanel::setupJoystick()
{
    if (!d_ptr->speed_control_group) return;

    auto* layout = qobject_cast<QHBoxLayout*>(d_ptr->speed_control_group->layout());
    if (!layout) return;

    // 创建左侧布局（摇杆和紧急停止按钮）
    auto* left_layout = new QVBoxLayout();
    left_layout->setSpacing(15);  // 增加垂直间距
    
    // 创建摇杆
    d_ptr->joystick = std::make_shared<JoystickWidget>(d_ptr->speed_control_group);
    d_ptr->joystick->setMinimumSize(200, 200);
    d_ptr->joystick->setMaximumSize(300, 300);
    left_layout->addWidget(d_ptr->joystick.get(), 0, Qt::AlignCenter);  // 居中对齐
    
    // 创建紧急停止按钮
    d_ptr->emergency_stop_button = new QPushButton(tr("紧急停止"), d_ptr->speed_control_group);
    d_ptr->emergency_stop_button->setMinimumWidth(150);  // 增加按钮宽度
    d_ptr->emergency_stop_button->setStyleSheet(
        "QPushButton {"
        "    background-color: #dc3545;"
        "    color: white;"
        "    border: none;"
        "    padding: 15px 30px;"  // 增加按钮内边距
        "    border-radius: 6px;"   // 增加圆角
        "    font-weight: bold;"    // 加粗字体
        "    font-size: 14px;"      // 增加字体大小
        "}"
        "QPushButton:hover {"
        "    background-color: #c82333;"
        "}"
        "QPushButton:pressed {"
        "    background-color: #bd2130;"
        "}"
    );
    left_layout->addWidget(d_ptr->emergency_stop_button, 0, Qt::AlignCenter);  // 居中对齐
    
    // 创建右侧布局（速度仪表盘和速度限制）
    auto* right_layout = new QVBoxLayout();
    right_layout->setSpacing(15);  // 增加垂直间距
    
    // 创建速度仪表盘
    d_ptr->speed_dashboard = std::make_shared<SpeedDashboard>(d_ptr->speed_control_group);
    d_ptr->speed_dashboard->setMinimumSize(250, 250);  // 增加仪表盘大小
    d_ptr->speed_dashboard->setMaximumSize(350, 350);
    right_layout->addWidget(d_ptr->speed_dashboard.get(), 0, Qt::AlignCenter);
    
    // 创建速度限制滑块组
    auto* speed_limit_group = new QGroupBox(tr("速度限制"), d_ptr->speed_control_group);
    speed_limit_group->setStyleSheet(
        "QGroupBox {"
        "    background-color: #f8f9fa;"
        "    border: 1px solid #dee2e6;"
        "    border-radius: 6px;"
        "    padding: 20px;"  // 增加内边距
        "    margin-top: 1em;"
        "}"
        "QGroupBox::title {"
        "    color: #495057;"
        "    font-weight: bold;"
        "}"
    );
    auto* speed_limit_layout = new QVBoxLayout(speed_limit_group);
    speed_limit_layout->setSpacing(15);  // 增加垂直间距
    
    // 线速度限制
    auto* linear_slider = new QSlider(Qt::Horizontal, speed_limit_group);
    linear_slider->setObjectName("linear_speed_slider");
    linear_slider->setRange(0, 100);
    linear_slider->setValue(50);
    linear_slider->setStyleSheet(
        "QSlider::groove:horizontal {"
        "    border: 1px solid #ddd;"
        "    background: white;"
        "    height: 10px;"
        "    border-radius: 5px;"
        "}"
        "QSlider::sub-page:horizontal {"
        "    background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,"
        "        stop: 0 #3b82f6, stop: 1 #60a5fa);"
        "    border: 1px solid #3b82f6;"
        "    height: 10px;"
        "    border-radius: 5px;"
        "}"
        "QSlider::add-page:horizontal {"
        "    background: #f1f5f9;"
        "    border: 1px solid #ddd;"
        "    height: 10px;"
        "    border-radius: 5px;"
        "}"
        "QSlider::handle:horizontal {"
        "    background: white;"
        "    border: 2px solid #3b82f6;"
        "    width: 20px;"
        "    margin-top: -6px;"
        "    margin-bottom: -6px;"
        "    border-radius: 10px;"
        "}"
        "QSlider::handle:horizontal:hover {"
        "    background: #f8fafc;"
        "}"
    );
    
    speed_limit_layout->addWidget(linear_slider);
    
    // 角速度限制
    auto* angular_slider = new QSlider(Qt::Horizontal, speed_limit_group);
    angular_slider->setObjectName("angular_speed_slider");
    angular_slider->setRange(0, 100);
    angular_slider->setValue(50);
    angular_slider->setStyleSheet(linear_slider->styleSheet());
    
    speed_limit_layout->addWidget(angular_slider);
    
    right_layout->addWidget(speed_limit_group);
    
    // 添加到主布局
    layout->addLayout(left_layout, 1);    // 权重1
    layout->addLayout(right_layout, 1);    // 权重1
    layout->setSpacing(30);  // 增加水平间距

    // 连接信号
    connect(linear_slider, &QSlider::valueChanged, this, [this](int value) {
        d_ptr->max_linear_speed = value / 100.0;
    });

    connect(angular_slider, &QSlider::valueChanged, this, [this](int value) {
        d_ptr->max_angular_speed = value / 100.0 * 2.0;  // 最大2.0 rad/s
    });
}

void ControlPanel::connectSignals()
{
    if (d_ptr->joystick) {
        connect(d_ptr->joystick.get(), &JoystickWidget::linearJoystickMoved,
                this, [this](double x, double y) {
                    if (!d_ptr->robot_controller) {
                        qWarning() << "机器人控制器未初始化";
                        return;
                    }
                    double linear_vel = -y * d_ptr->max_linear_speed;
                    d_ptr->robot_controller->setLinearVelocity(linear_vel);
                    if (d_ptr->speed_dashboard) {
                        d_ptr->speed_dashboard->setLinearSpeed(linear_vel);
                    }
                });

        connect(d_ptr->joystick.get(), &JoystickWidget::angularJoystickMoved,
                this, [this](double x, double y) {
                    if (!d_ptr->robot_controller) {
                        qWarning() << "机器人控制器未初始化";
                        return;
                    }
                    double angular_vel = -x * d_ptr->max_angular_speed;
                    d_ptr->robot_controller->setAngularVelocity(angular_vel);
                    if (d_ptr->speed_dashboard) {
                        d_ptr->speed_dashboard->setAngularSpeed(angular_vel);
                    }
                });
    }

    if (d_ptr->emergency_stop_button) {
        connect(d_ptr->emergency_stop_button, &QPushButton::clicked,
                this, &ControlPanel::onEmergencyStop);
    }

    if (d_ptr->camera_toggle_button) {
        connect(d_ptr->camera_toggle_button, &QPushButton::clicked,
                this, &ControlPanel::toggleCamera);
    }

    // 连接机器人控制器的信号
    if (d_ptr->robot_controller) {
        // 电池状态更新
        connect(d_ptr->robot_controller.get(), &RobotController::batteryStateChanged,
                this, &ControlPanel::updateBatteryStatus);
        
        // 诊断信息更新
        connect(d_ptr->robot_controller.get(), &RobotController::diagnosticsUpdated,
                this, &ControlPanel::updateDiagnostics);
    }
}

void ControlPanel::onEmergencyStop()
{
    if (!d_ptr->robot_controller) return;

    // 停止机器人
    d_ptr->robot_controller->stop();

    // 更新速度显示
    if (d_ptr->speed_dashboard) {
        d_ptr->speed_dashboard->setLinearSpeed(0.0);
        d_ptr->speed_dashboard->setAngularSpeed(0.0);
    }

    // 重置摇杆位置
    if (d_ptr->joystick) {
        d_ptr->joystick->reset();
    }
}

void ControlPanel::updateBatteryStatus(const sensor_msgs::BatteryState& status)
{
    d_ptr->battery_label->setText(tr("电池电量: %1%").arg(status.percentage));
    d_ptr->voltage_label->setText(tr("电压: %1 V").arg(status.voltage, 0, 'f', 1));
    d_ptr->current_label->setText(tr("电流: %1 A").arg(status.current, 0, 'f', 2));
    d_ptr->temperature_label->setText(tr("温度: %1 ℃").arg(status.temperature, 0, 'f', 1));
}

void ControlPanel::updateDiagnostics(const diagnostic_msgs::DiagnosticArray& diagnostics)
{
    for (const auto& status : diagnostics.status) {
        if (status.name == "motors") {
            d_ptr->motor_status_label->setText(tr("电机状态: %1").arg(QString::fromStdString(status.message)));
        }
    }
}

void ControlPanel::updateCameraImage(const sensor_msgs::Image& image)
{
    // TODO: 将ROS图像消息转换为Qt图像并显示
}

void ControlPanel::updateSpeedDisplay(double linear, double angular)
{
    if (d_ptr->speed_dashboard) {
        d_ptr->speed_dashboard->setLinearSpeed(linear);
        d_ptr->speed_dashboard->setAngularSpeed(angular);
    }
}

void ControlPanel::toggleCamera()
{
    if (!d_ptr->camera) {
        QMessageBox::warning(this, tr("错误"), tr("请先选择摄像头设备"));
        return;
    }

    d_ptr->camera_toggle_button->setEnabled(false);  // 禁用按钮，防止重复点击

    if (d_ptr->camera->state() == QCamera::ActiveState) {
        d_ptr->camera->stop();
        d_ptr->camera_toggle_button->setText(tr("开启相机"));
    } else {
        d_ptr->camera->start();
        if (d_ptr->camera->error() == QCamera::NoError) {
            d_ptr->camera_toggle_button->setText(tr("关闭相机"));
        }
    }

    d_ptr->camera_toggle_button->setEnabled(true);  // 重新启用按钮
}

void ControlPanel::updateRobotStatus()
{
    if (!d_ptr->robot_controller) {
        qWarning() << "机器人控制器未初始化";
        setStatusWarning(d_ptr->connection_status_label, tr("未连接"), false);
        setStatusWarning(d_ptr->battery_label, tr("电池电量: --"), false);
        setStatusWarning(d_ptr->voltage_label, tr("电压: -- V"), false);
        setStatusWarning(d_ptr->current_label, tr("电流: -- A"), false);
        setStatusWarning(d_ptr->temperature_label, tr("温度: -- ℃"), false);
        setStatusWarning(d_ptr->motor_status_label, tr("电机状态: 未知"), false);
        return;
    }

    // 更新连接状态
    bool is_connected = d_ptr->robot_controller->isInitialized();
    setStatusWarning(d_ptr->connection_status_label, 
                    tr("连接状态: %1").arg(is_connected ? tr("已连接") : tr("未连接")), 
                    is_connected);

    // 如果未连接，显示警告状态
    if (!is_connected) {
        setStatusWarning(d_ptr->battery_label, tr("电池电量: 未连接"), false);
        setStatusWarning(d_ptr->voltage_label, tr("电压: 未连接"), false);
        setStatusWarning(d_ptr->current_label, tr("电流: 未连接"), false);
        setStatusWarning(d_ptr->temperature_label, tr("温度: 未连接"), false);
        setStatusWarning(d_ptr->motor_status_label, tr("电机状态: 未连接"), false);
        qWarning() << "机器人未连接，无法获取状态信息";
    }
}

// 添加一个辅助方法来设置警告状态
void ControlPanel::setStatusWarning(QLabel* label, const QString& text, bool is_normal)
{
    if (!label) return;
    
    label->setText(text);
    
    QString style = QString(
        "QLabel {"
        "    background-color: %1;"
        "    color: white;"
        "    border-radius: 4px;"
        "    padding: 5px 10px;"
        "    font-size: 12px;"
        "}"
    ).arg(is_normal ? "#28a745" : "#dc3545");  // 正常为绿色，警告为红色
    
    label->setStyleSheet(style);
} 