#ifndef NAVIGATION_PANEL_H
#define NAVIGATION_PANEL_H

#include <QWidget>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QComboBox>
#include <QCheckBox>
#include <memory>

class RobotController;
class QTabWidget;

class NavigationPanel : public QWidget {
    Q_OBJECT

public:
    explicit NavigationPanel(std::shared_ptr<RobotController> robot_controller, QWidget* parent = nullptr);

signals:
    void navigationStateChanged(bool is_navigating);
    void mapUpdated();
    void navigationGoalSet(double x, double y, double theta);

public slots:
    void updateNavigationState();
    void updateMapState(bool has_map);
    void updatePoseEstimate(double x, double y, double theta);

private slots:
    void onSetGoalClicked();
    void onCancelNavigationClicked();
    void onSaveMapClicked();
    void onLoadMapClicked();
    void onStartMappingClicked();
    void onStopMappingClicked();
    void onInitialPoseClicked();
    void onNavigationModeChanged(int index);
    void onCostmapUpdateClicked();

private:
    void createUI();
    void setupStyle();
    void createNavigationTab();
    void createMappingTab();
    void createSettingsTab();
    void updateButtonStates();

    std::shared_ptr<RobotController> robot_controller_;

    // 主要UI组件
    QTabWidget* tab_widget_;

    // 导航标签页组件
    QLineEdit* x_input_;
    QLineEdit* y_input_;
    QLineEdit* theta_input_;
    QPushButton* set_goal_btn_;
    QPushButton* cancel_btn_;
    QLabel* status_label_;
    QComboBox* navigation_mode_;

    // 建图标签页组件
    QPushButton* start_mapping_btn_;
    QPushButton* stop_mapping_btn_;
    QPushButton* save_map_btn_;
    QPushButton* load_map_btn_;
    QLabel* mapping_status_;

    // 设置标签页组件
    QLineEdit* max_vel_linear_;
    QLineEdit* max_vel_angular_;
    QLineEdit* min_obstacle_dist_;
    QCheckBox* enable_recovery_;
    QPushButton* update_costmap_btn_;
    QPushButton* set_initial_pose_btn_;

    // 状态变量
    bool is_mapping_;
    bool has_map_;
    bool is_navigating_;
};

#endif // NAVIGATION_PANEL_H 