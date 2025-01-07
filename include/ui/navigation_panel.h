#ifndef NAVIGATION_PANEL_H
#define NAVIGATION_PANEL_H

#include <QWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QComboBox>
#include <QProgressBar>
#include <QCheckBox>
#include <memory>
#include "ros/robot_controller.h"

class NavigationPanel : public QWidget {
    Q_OBJECT

public:
    explicit NavigationPanel(std::shared_ptr<RobotController> robot_controller,
                           QWidget* parent = nullptr);

public slots:
    void updateNavigationState(NavigationState state);
    void updateNavigationProgress(double progress);
    void updateNavigationStatus(const QString& status);
    void updatePoseEstimate(double x, double y, double theta);

signals:
    void navigationGoalSet(double x, double y, double theta);
    void navigationStateChanged(bool is_navigating);
    void mapUpdated();

private slots:
    void onSetGoalClicked();
    void onCancelNavigationClicked();
    void onStartMappingClicked();
    void onStopMappingClicked();
    void onSaveMapClicked();
    void onLoadMapClicked();
    void onInitialPoseClicked();
    void onNavigationModeChanged(int index);
    void onCostmapUpdateClicked();
    void onPlannerTypeChanged(int index);
    void onHeuristicTypeChanged(int index);
    void onPlanningTimeChanged(const QString& text);
    void onInterpolationDistanceChanged(const QString& text);
    void onAllowUnknownChanged(int state);
    void onVisualizePlanningChanged(int state);

private:
    void createUI();
    void setupStyle();
    void updateButtonStates();
    void updateMapState(bool has_map);

    std::shared_ptr<RobotController> robot_controller_;
    
    // UI组件
    QLineEdit* x_input_;
    QLineEdit* y_input_;
    QLineEdit* theta_input_;
    QComboBox* navigation_mode_;
    QPushButton* set_goal_btn_;
    QPushButton* cancel_btn_;
    QLabel* status_label_;
    QProgressBar* progress_bar_;
    QLabel* distance_label_;
    QLabel* eta_label_;
    
    // 状态变量
    bool is_mapping_;
    bool has_map_;
    bool is_navigating_;
    NavigationState current_state_;

    // 路径规划相关控件
    QComboBox* planner_type_;
    QComboBox* heuristic_type_;
    QLineEdit* planning_time_;
    QLineEdit* interpolation_distance_;
    QCheckBox* allow_unknown_;
    QCheckBox* visualize_planning_;
};

#endif // NAVIGATION_PANEL_H 