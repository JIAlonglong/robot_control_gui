#ifndef TELEOP_PANEL_H
#define TELEOP_PANEL_H

#include <QLabel>
#include <QPushButton>
#include <QWidget>
#include <memory>
#include "robot_controller.h"

class QTimer;

class TeleopPanel : public QWidget
{
    Q_OBJECT

public:
    explicit TeleopPanel(std::shared_ptr<RobotController> robot_controller,
                         QWidget*                         parent = nullptr);
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
    QLabel*      linear_speed_label_;
    QLabel*      angular_speed_label_;

    // 控制相关
    QTimer* update_timer_;
    double  current_linear_velocity_;
    double  current_angular_velocity_;
};

#endif  // TELEOP_PANEL_H