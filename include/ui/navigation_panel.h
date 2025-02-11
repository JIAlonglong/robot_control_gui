#pragma once

#ifndef ROBOT_CONTROL_GUI_NAVIGATION_PANEL_H
#define ROBOT_CONTROL_GUI_NAVIGATION_PANEL_H

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QProgressBar>
#include <QComboBox>
#include <QMap>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QTimer>
#include <memory>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <QMutex>

#include "ros/robot_controller.h"
#include "ros/navigation_controller.h"
#include "ui/joystick_widget.h"
#include "ui/rviz_view.h"

class RobotController;
class NavigationPanel;
class JoystickWidget;
class RVizView;

class NavigationPanelPrivate {
public:
    explicit NavigationPanelPrivate(NavigationPanel* q)
        : q_ptr(q)
        , is_navigating_(false)
        , is_localizing_(false)
        , has_goal_(false)
        , keyboard_linear_speed_(0.5)
        , keyboard_angular_speed_(0.5)
        , joystick_linear_scale_(0.5)
        , joystick_angular_scale_(0.5)
    {
        key_pressed_[Qt::Key_W] = false;
        key_pressed_[Qt::Key_S] = false;
        key_pressed_[Qt::Key_A] = false;
        key_pressed_[Qt::Key_D] = false;
    }

    NavigationPanel* q_ptr;
    std::shared_ptr<RobotController> robot_controller;
    std::shared_ptr<RVizView> rviz_view;

    QVBoxLayout* main_layout_{nullptr};
    QGroupBox* control_group_{nullptr};
    QGroupBox* status_group_{nullptr};
    QGroupBox* planner_group_{nullptr};

    QPushButton* set_initial_pose_button{nullptr};
    QPushButton* auto_localization_button{nullptr};
    QPushButton* set_goal_button{nullptr};
    QPushButton* cancel_goal_button{nullptr};
    QPushButton* start_navigation_button{nullptr};
    QPushButton* stop_navigation_button{nullptr};
    QPushButton* pause_navigation_button{nullptr};
    QPushButton* emergency_stop_button{nullptr};
    QPushButton* planner_settings_button{nullptr};

    QLabel* navigation_status_label{nullptr};
    QLabel* distance_label{nullptr};
    QLabel* estimated_time_label{nullptr};
    QLabel* linear_velocity_label{nullptr};
    QLabel* angular_velocity_label{nullptr};
    QLabel* goal_status_label_{nullptr};

    QProgressBar* navigation_progress_bar{nullptr};

    QComboBox* planner_combo_{nullptr};

    QTimer* localization_timer{nullptr};

    bool is_navigating_;
    bool is_localizing_;
    bool has_goal_;
    QMap<int, bool> key_pressed_;
    double keyboard_linear_speed_;
    double keyboard_angular_speed_;
    double joystick_linear_scale_;
    double joystick_angular_scale_;
    geometry_msgs::PoseStamped current_goal_;
    
    JoystickWidget* joystick_{nullptr};

    QMutex mutex_;

    bool auto_localization_enabled_{false};
    QPushButton* auto_localization_button_{nullptr};
};

class NavigationPanel : public QWidget {
    Q_OBJECT

public:
    explicit NavigationPanel(const std::shared_ptr<RobotController>& controller, QWidget* parent = nullptr);
    ~NavigationPanel() override;

    void setRobotController(const std::shared_ptr<RobotController>& controller);
    void setNavigationController(const std::shared_ptr<NavigationController>& controller);
    void setRVizView(const std::shared_ptr<RVizView>& view);

public Q_SLOTS:
    void onNavigationStarted();
    void onNavigationFinished();
    void onNavigationCancelled();
    void onNavigationFailed(const QString& error);
    void onNavigationProgress(double progress);
    void onNavigationTargetChanged(const QString& target);
    void onNavigationStateChanged(const QString& state);
    void onNavigationStatusChanged(const QString& status);
    void onAutoLocalizationToggled(bool enabled);

private Q_SLOTS:
    void onStartNavigationButtonClicked();
    void onStopNavigationButtonClicked();
    void onPauseNavigationButtonClicked();
    void onResumeNavigationButtonClicked();
    void onSetTargetButtonClicked();
    void onClearTargetButtonClicked();
    void onAutoLocalizationButtonClicked();
    void onMethodChanged(int index);
    void onUpdateRateChanged(int value);
    void onMaxRangeChanged(double value);
    void onMinRangeChanged(double value);
    void onMaxVelocityChanged(double value);
    void onMinVelocityChanged(double value);
    void onAccelerationChanged(double value);
    void onDecelerationChanged(double value);

Q_SIGNALS:
    void navigationStarted();
    void navigationFinished();
    void navigationCancelled();
    void navigationFailed(const QString& error);
    void navigationProgress(double progress);
    void navigationTargetChanged(const QString& target);
    void navigationStateChanged(const QString& state);
    void navigationStatusChanged(const QString& status);
    void autoLocalizationStarted();
    void autoLocalizationFinished();

private:
    void setupUi();
    void connectSignals();
    void updateButtons();
    void updateButtonStates();
    void updateNavigationProgress(double progress);
    void updateNavigationStatus(const QString& status);

    struct Private;
    std::unique_ptr<Private> d_;
};

#endif // ROBOT_CONTROL_GUI_NAVIGATION_PANEL_H 