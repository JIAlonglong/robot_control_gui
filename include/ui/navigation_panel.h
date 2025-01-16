#pragma once

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QProgressBar>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QComboBox>
#include <QTimer>
#include <QDial>
#include <QTabWidget>
#include <memory>

class RobotController;
class RVizView;

class NavigationPanel : public QWidget {
    Q_OBJECT

public:
    explicit NavigationPanel(const std::shared_ptr<RobotController>& robot_controller,
                           QWidget* parent = nullptr);
    ~NavigationPanel() override;

    void setRVizView(const std::shared_ptr<RVizView>& rviz_view);

protected:
    void keyPressEvent(QKeyEvent* event) override;
    void keyReleaseEvent(QKeyEvent* event) override;

private slots:
    void onSetInitialPose();
    void onAutoLocalization();
    void onSetGoal();
    void onCancelGoal();
    void onStartMapping();
    void onStopMapping();
    void onSaveMap();
    void onLoadMap();
    void onNavigationModeChanged(int index);
    void updateLocalizationState(bool is_localized);
    void updateLocalizationStatus(const QString& status);
    void updateLocalizationProgress(double progress);
    void updateButtonStates();
    void onLinearVelocityChanged(int value);
    void onAngularVelocityChanged(int value);
    void onJoystickMoved();
    void onEmergencyStop();
    void onStartNavigation();
    void onPauseNavigation();
    void onStopNavigation();
    void onLocalizationStateChanged(bool is_localized);
    void onLocalizationProgressChanged(double progress);
    void onNavigationStateChanged(bool is_navigating);
    void onNavigationProgressChanged(double progress);
    void onDistanceToGoalChanged(double distance);
    void onEstimatedTimeToGoalChanged(double time);

Q_SIGNALS:
    void localizationStateChanged(bool is_localized);
    void localizationStatusChanged(const QString& status);
    void localizationProgressChanged(double progress);
    void navigationModeChanged(int mode);
    void navigationProgressChanged(double progress);
    void navigationDistanceChanged(double distance);
    void navigationTimeChanged(double time);

private:
    void setupUi();
    void setupJoystick();
    void setupKeyboardControl();
    void startAutoLocalization();
    void updateVelocityDisplay(double linear, double angular);
    void connectSignalsAndSlots();
    QPushButton* createStyledButton(const QString& text, const QString& icon_path = QString());

    class NavigationPanelPrivate {
    public:
        QVBoxLayout* main_layout{nullptr};
        QLabel* localization_status{nullptr};
        QProgressBar* localization_progress{nullptr};
        QPushButton* set_pose_button{nullptr};
        QPushButton* auto_localization_button{nullptr};
        QPushButton* set_goal_button{nullptr};
        QPushButton* cancel_goal_button{nullptr};
        QPushButton* start_mapping_button{nullptr};
        QPushButton* stop_mapping_button{nullptr};
        QPushButton* save_map_button{nullptr};
        QPushButton* load_map_button{nullptr};
        QComboBox* navigation_mode{nullptr};
        QLabel* navigation_status{nullptr};
        
        // 速度控制组件
        QDial* linear_velocity_dial{nullptr};
        QDial* angular_velocity_dial{nullptr};
        QLabel* linear_velocity_label{nullptr};
        QLabel* angular_velocity_label{nullptr};
        QPushButton* emergency_stop_button{nullptr};
        
        // RViz相关
        QWidget* rviz_placeholder{nullptr};
        std::shared_ptr<RVizView> rviz_view;
        
        // 控制器
        std::shared_ptr<RobotController> robot_controller;
        bool is_localizing{false};
        QTimer* localization_timer{nullptr};
        
        // 键盘控制状态
        bool key_up_pressed{false};
        bool key_down_pressed{false};
        bool key_left_pressed{false};
        bool key_right_pressed{false};
        double current_linear_velocity{0.0};
        double current_angular_velocity{0.0};
        
        // 导航控制
        QLabel* navigation_status_label{nullptr};
        QProgressBar* navigation_progress{nullptr};
        QLabel* distance_label{nullptr};
        QLabel* time_label{nullptr};
        QPushButton* start_nav_button{nullptr};
        QPushButton* pause_nav_button{nullptr};
        QPushButton* stop_nav_button{nullptr};
    };

    std::unique_ptr<NavigationPanelPrivate> d_ptr;
}; 