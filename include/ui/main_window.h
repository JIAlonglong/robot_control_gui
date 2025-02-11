/**
 * @file main_window.h
 * @brief 机器人控制系统的主窗口类
 * 
 * 该类是整个应用程序的主界面,集成了:
 * - 机器人控制面板
 * - 导航面板
 * - 建图面板
 * - 动作编辑器
 * - 设置面板
 * - RViz可视化界面
 */

#pragma once

#ifndef ROBOT_CONTROL_GUI_MAIN_WINDOW_H
#define ROBOT_CONTROL_GUI_MAIN_WINDOW_H

#include <QMainWindow>
#include <QKeyEvent>
#include <QLabel>
#include <QStackedWidget>
#include <QToolBar>
#include <QStatusBar>
#include <QButtonGroup>
#include <memory>
#include <QDockWidget>
#include <QMenuBar>
#include <QVBoxLayout>
#include <QToolButton>
#include <QHBoxLayout>
#include <QSplitter>
#include <QMessageBox>
#include <QIcon>
#include <QEvent>

#include "ui/control_panel.h"
#include "ui/navigation_panel.h"
#include "ui/mapping_panel.h"
#include "ui/settings_panel.h"
#include "ui/rviz_view.h"
#include "ui/action_configurator.h"
#include "ros/robot_controller.h"
#include "ros/action_block_factory.h"

class RobotController;
class MappingController;
class NavigationPanel;
class ControlPanel;
class MappingPanel;
class SettingsPanel;
class RVizView;
class ActionConfigurator;
class ActionBlockFactory;

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param parent 父窗口部件
     */
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

protected:
    /**
     * @brief 按键按下事件处理
     * @param event 键盘事件
     */
    void keyPressEvent(QKeyEvent* event) override;

    /**
     * @brief 按键释放事件处理
     * @param event 键盘事件
     */
    void keyReleaseEvent(QKeyEvent* event) override;

    /**
     * @brief 窗口大小改变事件处理
     * @param event 大小改变事件
     */
    void resizeEvent(QResizeEvent* event) override;

    /**
     * @brief 窗口关闭事件处理
     * @param event 关闭事件
     */
    void closeEvent(QCloseEvent* event) override;

    /**
     * @brief 通用事件处理
     * @param event 事件对象
     * @return 是否处理了事件
     */
    bool event(QEvent* event) override;

    bool initialize();

private Q_SLOTS:
    /**
     * @brief 显示控制面板
     */
    void onShowControlPanel();

    /**
     * @brief 显示导航面板
     */
    void onShowNavigationPanel();

    /**
     * @brief 显示建图面板
     */
    void onShowMappingPanel();

    /**
     * @brief 显示遥操作面板
     */
    void onShowSettingsPanel();

    /**
     * @brief 显示动作配置器
     */
    void onShowActionConfigurator();

    /**
     * @brief 连接状态改变处理
     * @param connected 是否已连接
     */
    void onConnectionStateChanged(bool connected);

    /**
     * @brief 机器人状态改变处理
     * @param state 新状态
     */
    void onRobotStateChanged(const QString& state);

    void onAbout();
    void onRobotConnected();
    void onRobotDisconnected();
    void onBatteryStateChanged(double percentage, const QString& status);
    void handleKeyEvent(QKeyEvent* event, bool pressed);
    void onDockVisibilityChanged(bool visible);
    void onBatteryStatusChanged(const QString& status);
    void onBatteryLevelChanged(int level);
    void onBatteryVoltageChanged(double voltage);
    void onBatteryCurrentChanged(double current);
    void onBatteryTemperatureChanged(double temperature);
    void onNavigationStateChanged(const QString& state);
    void onNavigationStatusChanged(const QString& status);
    void onNavigationProgressChanged(double progress);
    void onLocalizationStateChanged(const QString& state);
    void onLocalizationStatusChanged(const QString& status);
    void onLocalizationProgressChanged(double progress);
    void onMappingStateChanged(const QString& state);
    void onRVizInitializationSucceeded();
    void onRVizInitializationFailed(const QString& error);
    void onError(const QString& error);
    void onSequenceStarted();
    void onSequenceStopped();
    void onSequenceCompleted();

private:
    void setupUi();
    void createMenus();
    void createStatusBar();
    void createToolBar();
    void createDockWidgets();
    void connectSignals();
    void readSettings();
    void writeSettings();
    void updateToolbarActions(QAction* active_action);
    void addToolbarButton(QToolBar* toolbar, const QString& text, const QIcon& icon, int id);
    void setupActionBlockFactories();
    void updateDockWidgets();
    void saveDockState();
    void restoreDockState();
    QDockWidget* createDockWidget(const QString& title, QWidget* widget, QDockWidget::DockWidgetFeatures features);

    struct Private {
        QToolBar* tool_bar_{nullptr};
        QStackedWidget* stacked_widget_{nullptr};
        QLabel* connection_label_{nullptr};
        QLabel* status_label_{nullptr};
        QLabel* battery_label_{nullptr};
        QLabel* robot_status_label_{nullptr};
        QStatusBar* status_bar_{nullptr};
        QButtonGroup* button_group_{nullptr};
        QVBoxLayout* main_layout_{nullptr};

        QAction* navigation_action_{nullptr};
        QAction* mapping_action_{nullptr};
        QAction* teleoperation_action_{nullptr};
        QAction* action_sequence_action_{nullptr};
        QAction* settings_action_{nullptr};
        QAction* action_config_action_{nullptr};

        NavigationPanel* navigation_panel_{nullptr};
        MappingPanel* mapping_panel_{nullptr};
        ControlPanel* control_panel_{nullptr};
        SettingsPanel* settings_panel_{nullptr};
        RVizView* rviz_view_{nullptr};

        QDockWidget* navigation_dock_{nullptr};
        QDockWidget* control_dock_{nullptr};
        QDockWidget* mapping_dock_{nullptr};
        QDockWidget* settings_dock_{nullptr};
        QDockWidget* action_config_dock_{nullptr};

        std::shared_ptr<RobotController> robot_controller_;
        std::shared_ptr<ActionBlockFactory> action_factory_;
    };

    std::unique_ptr<Private> d_;
};

#endif // ROBOT_CONTROL_GUI_MAIN_WINDOW_H 