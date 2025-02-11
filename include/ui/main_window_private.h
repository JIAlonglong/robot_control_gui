#ifndef MAIN_WINDOW_PRIVATE_H
#define MAIN_WINDOW_PRIVATE_H

#include "ui/main_window.h"
#include <QAction>
#include <QLabel>
#include <QVBoxLayout>
#include <QDockWidget>
#include <QStackedWidget>
#include <QToolBar>
#include <QStatusBar>
#include <QButtonGroup>
#include <memory>

class RobotController;
class MappingController;
class NavigationPanel;
class ControlPanel;
class MappingPanel;
class SettingsPanel;
class TeleopPanel;
class RVizView;
class ActionConfigurator;
class ActionBlockFactory;
class ActionSequenceManager;
class ActionListWidget;
class SettingsWidget;

class MainWindowPrivate {
public:
    explicit MainWindowPrivate(MainWindow* window = nullptr) : q_(window) {}

    MainWindow* q_{nullptr};
    std::shared_ptr<RobotController> robot_controller;
    std::shared_ptr<MappingController> mapping_controller;
    
    // 工具栏动作
    QAction* navigation_action{nullptr};
    QAction* control_action{nullptr};
    QAction* mapping_action{nullptr};
    QAction* settings_action{nullptr};
    QAction* action_config_action{nullptr};
    
    // 界面组件
    ActionListWidget* action_list_widget_{nullptr};
    RVizView* rviz_view_{nullptr};
    SettingsWidget* settings_widget_{nullptr};
    
    // 面板组件
    NavigationPanel* navigation_panel{nullptr};
    ControlPanel* control_panel{nullptr};
    MappingPanel* mapping_panel{nullptr};
    SettingsPanel* settings_panel{nullptr};
    TeleopPanel* teleop_panel{nullptr};
    ActionConfigurator* action_configurator{nullptr};
    std::shared_ptr<ActionBlockFactory> action_factory;
    ActionSequenceManager* sequence_manager{nullptr};

    // 布局组件
    QVBoxLayout* main_layout{nullptr};
    QStackedWidget* stacked_widget{nullptr};
    QToolBar* tool_bar{nullptr};
    QStatusBar* status_bar{nullptr};
    QButtonGroup* button_group{nullptr};

    // Dock窗口
    QDockWidget* control_dock_{nullptr};
    QDockWidget* teleop_dock_{nullptr};
    QDockWidget* navigation_dock_{nullptr};
    QDockWidget* mapping_dock_{nullptr};
    QDockWidget* settings_dock_{nullptr};
    QDockWidget* action_config_dock_{nullptr};
    
    // 状态栏组件
    QLabel* status_label_{nullptr};
    QLabel* battery_label_{nullptr};
    QLabel* connection_label_{nullptr};
    QLabel* robot_status_label{nullptr};
    QLabel* battery_status_label{nullptr};
};

#endif // MAIN_WINDOW_PRIVATE_H 