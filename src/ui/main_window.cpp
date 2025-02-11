/**
 * @file main_window.cpp
 * @brief 主窗口类的实现
 */
#include "ui/main_window.h"
#include "ui/main_window_private.h"
#include "ui/action_block_factory.h"
#include "ui/action_sequence_manager.h"
#include "ui/action_configurator.h"
#include "ui/rviz_view.h"
#include "ui/action_list_widget.h"
#include "ui/settings_widget.h"
#include "ui/navigation_panel.h"
#include "ui/control_panel.h"
#include "ui/mapping_panel.h"
#include "ui/settings_panel.h"
#include "ui/movement_action_block.h"
#include "ui/delay_action_block.h"
#include "ui/navigation_action_block.h"
#include "ui/loop_action_block.h"
#include "ui/condition_action_block.h"
#include "ui/path_action_block.h"
#include "ros/robot_controller.h"
#include "ros/mapping_controller.h"
#include <QMenuBar>
#include <QStatusBar>
#include <QDockWidget>
#include <QMessageBox>
#include <QFileDialog>
#include <QSettings>
#include <QCloseEvent>
#include <QApplication>
#include <QDesktopWidget>
#include <QScreen>
#include <QStackedWidget>
#include <QToolBar>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSplitter>
#include <QAction>
#include <QButtonGroup>
#include <QToolButton>
#include <ros/ros.h>
#include <QSurfaceFormat>
#include <QTimer>
#include <QIcon>
#include <QDebug>
#include <QSizePolicy>
#include <QOpenGLContext>
#include <QKeySequence>

struct MainWindow::Private {
    QToolBar* tool_bar_{nullptr};
    QButtonGroup* button_group_{nullptr};
    QStatusBar* status_bar_{nullptr};
    QLabel* connection_label_{nullptr};
    QLabel* battery_label_{nullptr};
    QLabel* robot_status_label_{nullptr};
    QLabel* navigation_status_label_{nullptr};
    QLabel* localization_status_label_{nullptr};
    QLabel* mapping_status_label_{nullptr};
    QLabel* motor_status_label_{nullptr};
    QStackedWidget* stacked_widget_{nullptr};
    QDockWidget* navigation_dock_{nullptr};
    QDockWidget* control_dock_{nullptr};
    QDockWidget* mapping_dock_{nullptr};
    QDockWidget* settings_dock_{nullptr};
    QDockWidget* action_config_dock_{nullptr};
    QAction* navigation_action_{nullptr};
    QAction* control_action_{nullptr};
    QAction* mapping_action_{nullptr};
    QAction* settings_action_{nullptr};
    QAction* action_config_action_{nullptr};
    RVizView* rviz_view_{nullptr};
    ControlPanel* control_panel_{nullptr};
    NavigationPanel* navigation_panel_{nullptr};
    MappingPanel* mapping_panel_{nullptr};
    SettingsPanel* settings_panel_{nullptr};
    ActionConfigurator* action_configurator_{nullptr};
    std::shared_ptr<RobotController> robot_controller_;
    std::shared_ptr<ActionBlockFactory> action_factory_;
};

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , d_(std::make_unique<Private>())
{
    // 设置窗口属性
    setWindowTitle(tr("机器人控制系统"));
    setWindowIcon(QIcon(":/icons/robot.png"));
    setMinimumSize(800, 600);

    // 检查OpenGL支持
    QOpenGLContext context;
    if (!context.create()) {
        // 尝试使用最基本的OpenGL配置
        QSurfaceFormat fallback_format;
        fallback_format.setVersion(2, 1);
        fallback_format.setProfile(QSurfaceFormat::NoProfile);
        fallback_format.setDepthBufferSize(24);
        fallback_format.setStencilBufferSize(8);
        fallback_format.setSamples(0);
        QSurfaceFormat::setDefaultFormat(fallback_format);
        
        if (!context.create()) {
            QString error_msg = tr("OpenGL初始化失败。\n");
            error_msg += tr("当前OpenGL状态：\n");
            error_msg += tr("版本：%1.%2\n").arg(context.format().majorVersion()).arg(context.format().minorVersion());
            error_msg += tr("配置：%1\n").arg(context.format().profile() == QSurfaceFormat::NoProfile ? "No Profile" : "Unknown");
            ROS_ERROR_STREAM("OpenGL初始化失败: " << error_msg.toStdString());
            throw std::runtime_error("OpenGL不可用");
        }
    }

    // 创建中心部件
    auto* central_widget = new QWidget(this);
    setCentralWidget(central_widget);

    // 创建主布局
    d_->main_layout_ = new QVBoxLayout(central_widget);
    d_->main_layout_->setContentsMargins(0, 0, 0, 0);
    d_->main_layout_->setSpacing(0);

    // 创建分割器
    auto* splitter = new QSplitter(Qt::Horizontal, this);
    d_->main_layout_->addWidget(splitter);

    // 创建左侧面板
    auto* left_panel = new QWidget(this);
    auto* left_layout = new QVBoxLayout(left_panel);
    left_layout->setContentsMargins(0, 0, 0, 0);
    left_layout->setSpacing(0);

    // 创建堆叠部件
    d_->stacked_widget_ = new QStackedWidget(this);
    d_->stacked_widget_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);

    // 创建RViz视图
    d_->rviz_view_ = new RVizView(this);
    d_->rviz_view_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    // 添加部件到分割器
    splitter->addWidget(left_panel);
    splitter->addWidget(d_->rviz_view_);

    // 设置分割器比例
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 2);

    // 创建面板
    if (d_->robot_controller_) {
        // 创建控制面板
        d_->control_panel_ = new ControlPanel(d_->robot_controller_, this);
        d_->stacked_widget_->addWidget(d_->control_panel_);

        // 创建导航面板
        d_->navigation_panel_ = new NavigationPanel(d_->robot_controller_, this);
        d_->stacked_widget_->addWidget(d_->navigation_panel_);

        // 创建建图面板
        d_->mapping_panel_ = new MappingPanel(d_->robot_controller_, this);
        d_->stacked_widget_->addWidget(d_->mapping_panel_);

        // 创建动作配置器
        d_->action_configurator_ = new ActionConfigurator(d_->action_factory_, this);
        d_->stacked_widget_->addWidget(d_->action_configurator_);

        // 创建设置面板
        d_->settings_panel_ = new SettingsPanel(d_->robot_controller_, this);
        d_->stacked_widget_->addWidget(d_->settings_panel_);
    }

    // 将堆叠部件添加到左侧面板
    d_->stacked_widget_->setParent(left_panel);
    left_layout->addWidget(d_->stacked_widget_);

    // 设置初始页面
    d_->stacked_widget_->setCurrentIndex(0);

    // 创建状态栏
    createStatusBar();

    // 连接信号和槽
    connectSignals();

    // 恢复上次的窗口状态
    QSettings settings("RobotControlGUI", "MainWindow");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());

    // 初始化主窗口
    if (!initialize()) {
        QMessageBox::critical(this, tr("错误"), tr("主窗口初始化失败"));
        throw std::runtime_error("主窗口初始化失败");
    }
}

bool MainWindow::initialize()
{
    try {
        ROS_INFO("开始初始化主窗口...");

        // 创建机器人控制器
        d_->robot_controller = std::make_shared<RobotController>();
        d_->robot_controller->initialize();
        ROS_INFO("机器人控制器初始化完成");

        // 创建动作配置器
        d_->action_factory = std::make_shared<ActionBlockFactory>(d_->robot_controller, this);
        d_->action_configurator = new ActionConfigurator(d_->action_factory, this);
        ROS_INFO("动作配置器初始化完成");

        // 设置UI布局
        setupUi();
        ROS_INFO("UI布局设置完成");

        // 初始化RViz视图
        if (!d_->rviz_view_) {
            ROS_ERROR("RViz视图创建失败");
            QMessageBox::critical(this, tr("错误"), tr("RViz视图创建失败"));
            return false;
        }

        try {
            d_->rviz_view_->setRobotController(d_->robot_controller);
            d_->rviz_view_->initialize();
            ROS_INFO("RViz视图初始化完成");
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("RViz视图初始化异常: " << e.what());
            QMessageBox::critical(this, tr("错误"), 
                tr("RViz视图初始化异常：%1").arg(e.what()));
            return false;
        }

        // 更新状态栏
        d_->status_label_->setText(tr("就绪"));
        ROS_INFO("主窗口初始化完成");

        // 显示窗口
        show();
        ROS_INFO("主窗口已显示");

        return true;
    } catch (const std::exception& e) {
        QMessageBox::critical(this, tr("错误"), 
            tr("初始化过程中发生错误：%1").arg(e.what()));
        ROS_ERROR("初始化失败：%s", e.what());
        return false;
    }
}

void MainWindow::setupUi()
{
    // 创建菜单栏和工具栏
    createMenus();
    createToolBar();

    // 创建水平分割器
    auto* splitter = new QSplitter(Qt::Horizontal);
    d_->main_layout_->addWidget(splitter);

    // 创建左侧面板
    auto* left_panel = new QWidget(splitter);
    auto* left_layout = new QVBoxLayout(left_panel);
    left_layout->setContentsMargins(0, 0, 0, 0);
    left_layout->setSpacing(0);

    // 添加各个面板到堆叠部件
    if (d_->robot_controller) {
        // 控制面板
        d_->control_panel_ = new ControlPanel(d_->robot_controller, this);
        d_->stacked_widget_->addWidget(d_->control_panel_);

        // 导航面板
        d_->navigation_panel_ = new NavigationPanel(d_->robot_controller, this);
        d_->stacked_widget_->addWidget(d_->navigation_panel_);

        // 建图面板
        d_->mapping_panel_ = new MappingPanel(d_->robot_controller, this);
        d_->stacked_widget_->addWidget(d_->mapping_panel_);

        // 动作配置面板
        d_->action_configurator = new ActionConfigurator(d_->action_factory, this);
        d_->stacked_widget_->addWidget(d_->action_configurator);

        // 设置面板
        d_->settings_panel_ = new SettingsPanel(d_->robot_controller, this);
        d_->stacked_widget_->addWidget(d_->settings_panel_);
    }

    // 将堆叠部件移动到左侧面板
    d_->stacked_widget_->setParent(left_panel);
    left_layout->addWidget(d_->stacked_widget_);

    // 创建RViz视图
    d_->rviz_view_ = new RVizView(splitter);
    if (!d_->rviz_view_) {
        throw std::runtime_error("无法创建RViz视图");
    }
    
    // 设置RViz视图的大小策略
    d_->rviz_view_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    d_->rviz_view_->setMinimumWidth(600);

    // 添加到分割器
    splitter->addWidget(left_panel);
    splitter->addWidget(d_->rviz_view_);

    // 设置分割器比例
    splitter->setStretchFactor(0, 1);  // 左侧面板
    splitter->setStretchFactor(1, 2);  // RViz视图

    // 设置默认显示的面板
    d_->stacked_widget_->setCurrentIndex(0);

    // 创建停靠窗口
    createDockWidgets();
}

void MainWindow::createMenus()
{
    // 创建文件菜单
    auto* file_menu = menuBar()->addMenu(tr("文件(&F)"));
    
    auto* save_map_action = file_menu->addAction(tr("保存地图(&S)"), this, [this]() {
        if (d_->mapping_panel_) {
            d_->mapping_panel_->onSaveMap();
        }
    });
    save_map_action->setShortcut(QKeySequence::Save);
    
    auto* load_map_action = file_menu->addAction(tr("加载地图(&O)"), this, [this]() {
        if (d_->mapping_panel_) {
            d_->mapping_panel_->onLoadMap();
        }
    });
    load_map_action->setShortcut(QKeySequence::Open);
    
    file_menu->addSeparator();
    
    auto* quit_action = file_menu->addAction(tr("退出(&Q)"), this, &QWidget::close);
    quit_action->setShortcut(QKeySequence::Quit);
    
    // 创建编辑菜单
    auto* edit_menu = menuBar()->addMenu(tr("编辑(&E)"));
    
    auto* undo_action = edit_menu->addAction(tr("撤销(&U)"), this, [this]() {
        if (d_->action_configurator_) {
            d_->action_configurator_->undo();
        }
    });
    undo_action->setShortcut(QKeySequence::Undo);
    
    auto* redo_action = edit_menu->addAction(tr("重做(&R)"), this, [this]() {
        if (d_->action_configurator_) {
            d_->action_configurator_->redo();
        }
    });
    redo_action->setShortcut(QKeySequence::Redo);
    
    // 创建视图菜单
    auto* view_menu = menuBar()->addMenu(tr("视图(&V)"));
    
    auto* control_panel_action = view_menu->addAction(tr("控制面板(&C)"), this, &MainWindow::onShowControlPanel);
    control_panel_action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_1));
    
    auto* navigation_panel_action = view_menu->addAction(tr("导航面板(&N)"), this, &MainWindow::onShowNavigationPanel);
    navigation_panel_action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_2));
    
    auto* mapping_panel_action = view_menu->addAction(tr("建图面板(&M)"), this, &MainWindow::onShowMappingPanel);
    mapping_panel_action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_3));
    
    auto* settings_panel_action = view_menu->addAction(tr("设置面板(&S)"), this, &MainWindow::onShowSettingsPanel);
    settings_panel_action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_4));
    
    auto* action_config_action = view_menu->addAction(tr("动作配置器(&A)"), this, &MainWindow::onShowActionConfigurator);
    action_config_action->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_5));
    
    view_menu->addSeparator();
    
    auto* refresh_action = view_menu->addAction(tr("刷新(&R)"), this, [this]() {
        if (d_->rviz_view_) {
            d_->rviz_view_->initialize();
        }
    });
    refresh_action->setShortcut(QKeySequence::Refresh);
    
    // 创建帮助菜单
    auto* help_menu = menuBar()->addMenu(tr("帮助(&H)"));
    
    auto* about_action = help_menu->addAction(tr("关于(&A)"), this, &MainWindow::onAbout);
    about_action->setShortcut(QKeySequence::HelpContents);
}

void MainWindow::createToolBar()
{
    d_->tool_bar_ = addToolBar(tr("主工具栏"));
    d_->tool_bar_->setMovable(false);
    
    // 添加连接/断开按钮
    auto* connect_action = d_->tool_bar_->addAction(QIcon(":/icons/connect.png"), tr("连接"));
    connect_action->setCheckable(true);
    connect(connect_action, &QAction::triggered, this, [this, connect_action](bool checked) {
        if (checked) {
            if (d_->robot_controller_) {
                d_->robot_controller_->connect();
                connect_action->setIcon(QIcon(":/icons/disconnect.png"));
                connect_action->setText(tr("断开"));
            }
        } else {
            if (d_->robot_controller_) {
                d_->robot_controller_->disconnect();
                connect_action->setIcon(QIcon(":/icons/connect.png"));
                connect_action->setText(tr("连接"));
            }
        }
    });
    
    // 添加紧急停止按钮
    auto* stop_action = d_->tool_bar_->addAction(QIcon(":/icons/stop.png"), tr("紧急停止"));
    stop_action->setShortcut(Qt::Key_Space);
    connect(stop_action, &QAction::triggered, this, [this]() {
        if (d_->robot_controller_) {
            d_->robot_controller_->emergencyStop();
            QMessageBox::warning(this, tr("紧急停止"), tr("机器人已紧急停止!"));
        }
    });
    
    // 添加清除代价地图按钮
    auto* clear_costmap_action = d_->tool_bar_->addAction(QIcon(":/icons/clear.png"), tr("清除代价地图"));
    connect(clear_costmap_action, &QAction::triggered, this, [this]() {
        if (d_->robot_controller_) {
            d_->robot_controller_->clearCostmaps();
            statusBar()->showMessage(tr("代价地图已清除"), 3000);
        }
    });
    
    d_->tool_bar_->addSeparator();
    
    // 添加面板切换按钮组
    d_->button_group_ = new QButtonGroup(this);
    d_->button_group_->setExclusive(true);
    
    addToolbarButton(d_->tool_bar_, tr("控制面板"), QIcon(":/icons/control.png"), 0);
    addToolbarButton(d_->tool_bar_, tr("导航"), QIcon(":/icons/navigation.png"), 1);
    addToolbarButton(d_->tool_bar_, tr("建图"), QIcon(":/icons/mapping.png"), 2);
    addToolbarButton(d_->tool_bar_, tr("动作编辑"), QIcon(":/icons/action.png"), 3);
    addToolbarButton(d_->tool_bar_, tr("设置"), QIcon(":/icons/settings.png"), 4);
    
    // 连接按钮组信号
    connect(d_->button_group_, QOverload<int>::of(&QButtonGroup::buttonClicked),
            this, [this](int id) {
        // 切换当前面板
        d_->stacked_widget_->setCurrentIndex(id);
        
        // 更新RViz显示
        if (d_->rviz_view_) {
            switch (id) {
                case 0: // 控制面板
                    d_->rviz_view_->showRobotModel(true);
                    d_->rviz_view_->showLaserScan(true);
                    d_->rviz_view_->showMap(false);
                    d_->rviz_view_->showPath(false);
                    break;
                case 1: // 导航面板
                    d_->rviz_view_->showRobotModel(true);
                    d_->rviz_view_->showMap(true);
                    d_->rviz_view_->showPath(true);
                    d_->rviz_view_->showLaserScan(false);
                    break;
                case 2: // 建图面板
                    d_->rviz_view_->showRobotModel(true);
                    d_->rviz_view_->showLaserScan(true);
                    d_->rviz_view_->showMap(true);
                    d_->rviz_view_->showPath(false);
                    break;
            }
        }
    });
    
    // 默认选中第一个按钮
    if (auto* button = d_->button_group_->button(0)) {
        button->click();
    }
}

void MainWindow::createStatusBar()
{
    d_->status_bar_ = statusBar();
    
    // 创建状态标签
    d_->connection_label_ = new QLabel(tr("未连接"), this);
    d_->connection_label_->setMinimumWidth(100);
    
    d_->battery_label_ = new QLabel(tr("电池: --"), this);
    d_->battery_label_->setMinimumWidth(150);
    
    d_->robot_status_label_ = new QLabel(tr("状态: --"), this);
    d_->robot_status_label_->setMinimumWidth(150);
    
    // 创建新的状态标签
    d_->navigation_status_label_ = new QLabel(tr("导航: --"), this);
    d_->navigation_status_label_->setMinimumWidth(150);
    
    d_->localization_status_label_ = new QLabel(tr("定位: --"), this);
    d_->localization_status_label_->setMinimumWidth(150);
    
    d_->mapping_status_label_ = new QLabel(tr("建图: --"), this);
    d_->mapping_status_label_->setMinimumWidth(150);
    
    d_->motor_status_label_ = new QLabel(tr("电机: --"), this);
    d_->motor_status_label_->setMinimumWidth(100);
    
    // 添加到状态栏
    d_->status_bar_->addPermanentWidget(d_->connection_label_);
    d_->status_bar_->addPermanentWidget(d_->battery_label_);
    d_->status_bar_->addPermanentWidget(d_->robot_status_label_);
    d_->status_bar_->addPermanentWidget(d_->navigation_status_label_);
    d_->status_bar_->addPermanentWidget(d_->localization_status_label_);
    d_->status_bar_->addPermanentWidget(d_->mapping_status_label_);
    d_->status_bar_->addPermanentWidget(d_->motor_status_label_);
    
    // 设置样式
    QString normalStyle = "QLabel { color: black; }";
    QString errorStyle = "QLabel { color: red; }";
    QString warningStyle = "QLabel { color: orange; }";
    QString successStyle = "QLabel { color: green; }";
    
    d_->connection_label_->setStyleSheet(normalStyle);
    d_->battery_label_->setStyleSheet(normalStyle);
    d_->robot_status_label_->setStyleSheet(normalStyle);
    d_->navigation_status_label_->setStyleSheet(normalStyle);
    d_->localization_status_label_->setStyleSheet(normalStyle);
    d_->mapping_status_label_->setStyleSheet(normalStyle);
    d_->motor_status_label_->setStyleSheet(normalStyle);
}

void MainWindow::createDockWidgets()
{
    // 创建各个面板
    d_->control_panel_ = new ControlPanel(d_->robot_controller_, this);
    d_->navigation_panel_ = new NavigationPanel(d_->robot_controller_, this);
    d_->mapping_panel_ = new MappingPanel(d_->robot_controller_, this);
    d_->settings_panel_ = new SettingsPanel(d_->robot_controller_, this);

    // 设置面板的大小策略
    d_->control_panel_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    d_->navigation_panel_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    d_->mapping_panel_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    d_->settings_panel_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    d_->action_configurator_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

    // 设置面板的最小大小
    QSize min_size(250, 300);
    d_->control_panel_->setMinimumSize(min_size);
    d_->navigation_panel_->setMinimumSize(min_size);
    d_->mapping_panel_->setMinimumSize(min_size);
    d_->settings_panel_->setMinimumSize(min_size);
    d_->action_configurator_->setMinimumSize(min_size);

    // 添加面板到堆叠部件
    d_->stacked_widget_->addWidget(d_->control_panel_);
    d_->stacked_widget_->addWidget(d_->navigation_panel_);
    d_->stacked_widget_->addWidget(d_->mapping_panel_);
    d_->stacked_widget_->addWidget(d_->action_configurator_);
    d_->stacked_widget_->addWidget(d_->settings_panel_);

    // 设置dock窗口的特性
    auto features = QDockWidget::DockWidgetClosable |
                   QDockWidget::DockWidgetMovable |
                   QDockWidget::DockWidgetFloatable;

    // 创建并设置各个dock窗口
    d_->navigation_dock_ = createDockWidget(tr("导航"), d_->navigation_panel_, features);
    d_->control_dock_ = createDockWidget(tr("控制"), d_->control_panel_, features);
    d_->mapping_dock_ = createDockWidget(tr("建图"), d_->mapping_panel_, features);
    d_->settings_dock_ = createDockWidget(tr("设置"), d_->settings_panel_, features);
    d_->action_config_dock_ = createDockWidget(tr("动作配置"), d_->action_configurator_, features);

    // 设置dock窗口的初始位置
    addDockWidget(Qt::LeftDockWidgetArea, d_->navigation_dock_);
    addDockWidget(Qt::LeftDockWidgetArea, d_->control_dock_);
    addDockWidget(Qt::LeftDockWidgetArea, d_->mapping_dock_);
    addDockWidget(Qt::LeftDockWidgetArea, d_->settings_dock_);
    addDockWidget(Qt::LeftDockWidgetArea, d_->action_config_dock_);

    // 初始隐藏所有dock窗口
    d_->navigation_dock_->hide();
    d_->control_dock_->hide();
    d_->mapping_dock_->hide();
    d_->settings_dock_->hide();
    d_->action_config_dock_->hide();
}

QDockWidget* MainWindow::createDockWidget(const QString& title, QWidget* widget, QDockWidget::DockWidgetFeatures features)
{
    auto* dock = new QDockWidget(title, this);
    dock->setWidget(widget);
    dock->setFeatures(features);
    dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    return dock;
}

void MainWindow::connectSignals()
{
    // 连接机器人控制器的信号
    if (d_->robot_controller_) {
        connect(d_->robot_controller_.get(), &RobotController::batteryStatusChanged,
                this, &MainWindow::onBatteryStatusChanged);

        connect(d_->robot_controller_.get(), &RobotController::batteryLevelChanged,
                this, &MainWindow::onBatteryLevelChanged);

        connect(d_->robot_controller_.get(), &RobotController::batteryVoltageChanged,
                this, &MainWindow::onBatteryVoltageChanged);

        connect(d_->robot_controller_.get(), &RobotController::batteryCurrentChanged,
                this, &MainWindow::onBatteryCurrentChanged);

        connect(d_->robot_controller_.get(), &RobotController::batteryTemperatureChanged,
                this, &MainWindow::onBatteryTemperatureChanged);

        connect(d_->robot_controller_.get(), &RobotController::navigationStateChanged,
                this, &MainWindow::onNavigationStateChanged);

        connect(d_->robot_controller_.get(), &RobotController::navigationStatusChanged,
                this, &MainWindow::onNavigationStatusChanged);

        connect(d_->robot_controller_.get(), &RobotController::navigationProgressChanged,
                this, &MainWindow::onNavigationProgressChanged);

        connect(d_->robot_controller_.get(), &RobotController::localizationStateChanged,
                this, &MainWindow::onLocalizationStateChanged);

        connect(d_->robot_controller_.get(), &RobotController::localizationStatusChanged,
                this, &MainWindow::onLocalizationStatusChanged);

        connect(d_->robot_controller_.get(), &RobotController::localizationProgressChanged,
                this, &MainWindow::onLocalizationProgressChanged);

        connect(d_->robot_controller_.get(), &RobotController::mappingStateChanged,
                this, &MainWindow::onMappingStateChanged);

        connect(d_->robot_controller_.get(), &RobotController::connectionStateChanged,
                this, &MainWindow::onConnectionStateChanged);

        connect(d_->robot_controller_.get(), &RobotController::error,
                this, &MainWindow::onError);
    }

    // 连接 RViz 视图的信号
    if (d_->rviz_view_) {
        connect(d_->rviz_view_, &RVizView::initializationSucceeded,
                this, &MainWindow::onRVizInitializationSucceeded);

        connect(d_->rviz_view_, &RVizView::initializationFailed,
                this, &MainWindow::onRVizInitializationFailed);
    }

    // 连接 dock 窗口的可见性变化信号
    if (d_->navigation_dock_) {
        connect(d_->navigation_dock_, &QDockWidget::visibilityChanged,
                this, &MainWindow::onDockVisibilityChanged);
    }

    if (d_->control_dock_) {
        connect(d_->control_dock_, &QDockWidget::visibilityChanged,
                this, &MainWindow::onDockVisibilityChanged);
    }

    if (d_->mapping_dock_) {
        connect(d_->mapping_dock_, &QDockWidget::visibilityChanged,
                this, &MainWindow::onDockVisibilityChanged);
    }

    if (d_->settings_dock_) {
        connect(d_->settings_dock_, &QDockWidget::visibilityChanged,
                this, &MainWindow::onDockVisibilityChanged);
    }

    if (d_->action_config_dock_) {
        connect(d_->action_config_dock_, &QDockWidget::visibilityChanged,
                this, &MainWindow::onDockVisibilityChanged);
    }
}

void MainWindow::onConnectionStateChanged(bool connected)
{
    if (connected) {
        d_->connection_label_->setText(tr("已连接"));
        onRobotConnected();
    } else {
        d_->connection_label_->setText(tr("未连接"));
        onRobotDisconnected();
    }
}

void MainWindow::onBatteryStateChanged(double percentage, const QString& status)
{
    if (d_->battery_label_) {
        d_->battery_label_->setText(tr("电池: %1% (%2)").arg(percentage, 0, 'f', 1).arg(status));
        
        if (percentage > 30) {
            d_->battery_label_->setStyleSheet("QLabel { color: green; }");
        } else if (percentage > 15) {
            d_->battery_label_->setStyleSheet("QLabel { color: orange; }");
        } else {
            d_->battery_label_->setStyleSheet("QLabel { color: red; }");
        }
    }
}

void MainWindow::onRobotStateChanged(const QString& state)
{
    d_->robot_status_label_->setText(tr("状态: %1").arg(state));
    statusBar()->showMessage(state);
}

void MainWindow::onRVizInitializationSucceeded()
{
    statusBar()->showMessage(tr("RViz 初始化成功"), 3000);
}

void MainWindow::onRVizInitializationFailed(const QString& error)
{
    QMessageBox::critical(this, tr("错误"), tr("RViz 初始化失败: %1").arg(error));
}

void MainWindow::onRobotConnected()
{
    statusBar()->showMessage(tr("已连接到机器人"), 3000);
}

void MainWindow::onRobotDisconnected()
{
    statusBar()->showMessage(tr("与机器人的连接已断开"), 3000);
}

void MainWindow::handleKeyEvent(QKeyEvent* event, bool pressed)
{
    // 如果控制面板可见,则转发键盘事件
    if (d_->control_panel_ && d_->control_panel_->isVisible()) {
        d_->control_panel_->handleKeyEvent(event, pressed);
    }
}

void MainWindow::keyPressEvent(QKeyEvent* event)
{
    // 处理键盘事件
    handleKeyEvent(event, true);
    
    // 处理快捷键
    switch (event->key()) {
        case Qt::Key_F1:  // 显示帮助
            onAbout();
            break;
            
        case Qt::Key_F5:  // 刷新RViz显示
            if (d_->rviz_view_) {
                d_->rviz_view_->initialize();
            }
            break;
            
        case Qt::Key_Space:  // 紧急停止
            if (d_->robot_controller_) {
                d_->robot_controller_->emergencyStop();
            }
            break;
            
        case Qt::Key_1:  // 切换到控制面板
            if (event->modifiers() & Qt::ControlModifier) {
                onShowControlPanel();
            }
            break;
            
        case Qt::Key_2:  // 切换到导航面板 
            if (event->modifiers() & Qt::ControlModifier) {
                onShowNavigationPanel();
            }
            break;
            
        case Qt::Key_3:  // 切换到建图面板
            if (event->modifiers() & Qt::ControlModifier) {
                onShowMappingPanel();
            }
            break;
            
        case Qt::Key_4:  // 切换到设置面板
            if (event->modifiers() & Qt::ControlModifier) {
                onShowSettingsPanel();
            }
            break;
            
        case Qt::Key_5:  // 切换到动作配置器
            if (event->modifiers() & Qt::ControlModifier) {
                onShowActionConfigurator();
            }
            break;
            
        case Qt::Key_S:  // 保存
            if (event->modifiers() & Qt::ControlModifier) {
                if (d_->mapping_panel_ && d_->mapping_panel_->isVisible()) {
                    d_->mapping_panel_->onSaveMap();
                }
            }
            break;
            
        case Qt::Key_O:  // 打开
            if (event->modifiers() & Qt::ControlModifier) {
                if (d_->mapping_panel_ && d_->mapping_panel_->isVisible()) {
                    d_->mapping_panel_->onLoadMap();
                }
            }
            break;
            
        case Qt::Key_Z:  // 撤销
            if (event->modifiers() & Qt::ControlModifier) {
                if (d_->action_configurator_ && d_->action_configurator_->isVisible()) {
                    d_->action_configurator_->undo();
                }
            }
            break;
            
        case Qt::Key_Y:  // 重做
            if (event->modifiers() & Qt::ControlModifier) {
                if (d_->action_configurator_ && d_->action_configurator_->isVisible()) {
                    d_->action_configurator_->redo();
                }
            }
            break;
    }
    
    QMainWindow::keyPressEvent(event);
}

void MainWindow::keyReleaseEvent(QKeyEvent* event)
{
    handleKeyEvent(event, false);
    QMainWindow::keyReleaseEvent(event);
}

void MainWindow::onAbout()
{
    QMessageBox::about(this, tr("关于"),
                      tr("机器人控制系统\n\n"
                         "版本: 3.0.0\n"
                         "作者: JIAlonglong\n"
                         "联系方式: jialonglongliu@gmail.com\n\n"
                         "Copyright © 2025 GDUT"));
}

void MainWindow::readSettings()
{
    QSettings settings("RobotControlGUI", "MainWindow");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
}

void MainWindow::writeSettings()
{
    QSettings settings("RobotControlGUI", "MainWindow");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    writeSettings();
    QMainWindow::closeEvent(event);
}

void MainWindow::onShowNavigationPanel()
{
    updateToolbarActions(d_->navigation_action_);
    d_->navigation_dock_->show();
    d_->navigation_dock_->raise();
}

void MainWindow::onShowControlPanel()
{
    updateToolbarActions(d_->control_action_);
    d_->control_dock_->show();
    d_->control_dock_->raise();
}

void MainWindow::onShowMappingPanel()
{
    updateToolbarActions(d_->mapping_action_);
    d_->mapping_dock_->show();
    d_->mapping_dock_->raise();
}

void MainWindow::onShowTeleopPanel()
{
    if (d_->teleop_dock_) {
        d_->teleop_dock_->show();
        d_->teleop_dock_->raise();
    }
}

void MainWindow::onShowSettingsPanel()
{
    updateToolbarActions(d_->settings_action_);
    d_->settings_dock_->show();
    d_->settings_dock_->raise();
}

void MainWindow::onShowActionConfigurator()
{
    updateToolbarActions(d_->action_config_action_);
    d_->action_config_dock_->show();
    d_->action_config_dock_->raise();
}

void MainWindow::onSequenceStarted()
{
    statusBar()->showMessage(tr("动作序列开始执行"));
}

void MainWindow::onSequenceStopped()
{
    statusBar()->showMessage(tr("动作序列已停止"));
}

void MainWindow::onSequenceCompleted()
{
    statusBar()->showMessage(tr("动作序列执行完成"));
}

void MainWindow::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);
    
    // 确保所有dock窗口的大小合适
    updateDockWidgets();
    
    // 保存窗口状态
    writeSettings();
}

bool MainWindow::event(QEvent* event)
{
    // 处理窗口状态变化事件
    if (event->type() == QEvent::WindowStateChange) {
        updateDockWidgets();
    }
    return QMainWindow::event(event);
}

void MainWindow::updateDockWidgets()
{
    // 更新所有dock窗口的大小
    if (d_->control_dock_) {
        d_->control_dock_->adjustSize();
    }
    if (d_->navigation_dock_) {
        d_->navigation_dock_->adjustSize();
    }
    if (d_->mapping_dock_) {
        d_->mapping_dock_->adjustSize();
    }
    if (d_->settings_dock_) {
        d_->settings_dock_->adjustSize();
    }
    if (d_->action_config_dock_) {
        d_->action_config_dock_->adjustSize();
    }
}

void MainWindow::onDockVisibilityChanged(bool visible)
{
    auto* dock = qobject_cast<QDockWidget*>(sender());
    if (dock) {
        if (dock == d_->control_dock_) {
            d_->control_action_->setChecked(visible);
        } else if (dock == d_->navigation_dock_) {
            d_->navigation_action_->setChecked(visible);
        } else if (dock == d_->mapping_dock_) {
            d_->mapping_action_->setChecked(visible);
        } else if (dock == d_->settings_dock_) {
            d_->settings_action_->setChecked(visible);
        } else if (dock == d_->action_config_dock_) {
            d_->action_config_action_->setChecked(visible);
        }
    }
}

void MainWindow::saveDockState()
{
    QSettings settings;
    settings.setValue("MainWindow/geometry", saveGeometry());
    settings.setValue("MainWindow/windowState", saveState());
}

void MainWindow::restoreDockState()
{
    QSettings settings;
    restoreGeometry(settings.value("MainWindow/geometry").toByteArray());
    restoreState(settings.value("MainWindow/windowState").toByteArray());
}

void MainWindow::updateToolbarActions(QAction* active_action)
{
    if (d_->navigation_action_) d_->navigation_action_->setChecked(d_->navigation_action_ == active_action);
    if (d_->control_action_) d_->control_action_->setChecked(d_->control_action_ == active_action);
    if (d_->mapping_action_) d_->mapping_action_->setChecked(d_->mapping_action_ == active_action);
    if (d_->settings_action_) d_->settings_action_->setChecked(d_->settings_action_ == active_action);
    if (d_->action_config_action_) d_->action_config_action_->setChecked(d_->action_config_action_ == active_action);
}

void MainWindow::setupActionBlockFactories()
{
    // 创建机器人控制器
    d_->robot_controller_ = std::make_shared<RobotController>();
    d_->robot_controller_->initialize();

    // 创建动作工厂
    d_->action_factory_ = std::make_shared<ActionBlockFactory>(d_->robot_controller_, this);
}

void MainWindow::addToolbarButton(QToolBar* toolbar, const QString& text, const QIcon& icon, int id)
{
    auto* button = new QToolButton(this);
    button->setText(text);
    button->setIcon(icon);
    button->setCheckable(true);
    button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    d_->button_group_->addButton(button, id);
    toolbar->addWidget(button);
}

void MainWindow::onBatteryStatusChanged(const QString& status)
{
    d_->battery_label_->setText(tr("电池: %1").arg(status));
}

void MainWindow::onBatteryLevelChanged(int level)
{
    d_->battery_label_->setText(tr("电池: %1%").arg(level));
}

void MainWindow::onBatteryVoltageChanged(double voltage)
{
    d_->battery_label_->setText(tr("电池: %.1fV").arg(voltage));
}

void MainWindow::onBatteryCurrentChanged(double current)
{
    d_->battery_label_->setText(tr("电池: %.1fA").arg(current));
}

void MainWindow::onBatteryTemperatureChanged(double temperature)
{
    d_->battery_label_->setText(tr("电池: %.1f°C").arg(temperature));
}

void MainWindow::onNavigationStateChanged(const QString& state)
{
    if (d_->navigation_status_label_) {
        d_->navigation_status_label_->setText(tr("导航: %1").arg(state));
        
        // 根据状态设置样式
        if (state == "运行中") {
            d_->navigation_status_label_->setStyleSheet("QLabel { color: green; }");
        } else if (state == "暂停") {
            d_->navigation_status_label_->setStyleSheet("QLabel { color: orange; }");
        } else if (state == "错误") {
            d_->navigation_status_label_->setStyleSheet("QLabel { color: red; }");
        } else {
            d_->navigation_status_label_->setStyleSheet("QLabel { color: black; }");
        }
    }
}

void MainWindow::onNavigationStatusChanged(const QString& status)
{
    statusBar()->showMessage(status);
}

void MainWindow::onNavigationProgressChanged(double progress)
{
    statusBar()->showMessage(tr("导航进度: %1%").arg(progress * 100.0, 0, 'f', 1));
}

void MainWindow::onLocalizationStateChanged(const QString& state)
{
    if (d_->localization_status_label_) {
        d_->localization_status_label_->setText(tr("定位: %1").arg(state));
        
        if (state == "已定位") {
            d_->localization_status_label_->setStyleSheet("QLabel { color: green; }");
        } else if (state == "定位中") {
            d_->localization_status_label_->setStyleSheet("QLabel { color: orange; }");
        } else if (state == "未定位") {
            d_->localization_status_label_->setStyleSheet("QLabel { color: red; }");
        } else {
            d_->localization_status_label_->setStyleSheet("QLabel { color: black; }");
        }
    }
}

void MainWindow::onLocalizationStatusChanged(const QString& status)
{
    statusBar()->showMessage(status);
}

void MainWindow::onLocalizationProgressChanged(double progress)
{
    statusBar()->showMessage(tr("定位进度: %1%").arg(progress * 100.0, 0, 'f', 1));
}

void MainWindow::onMappingStateChanged(const QString& state)
{
    if (d_->mapping_status_label_) {
        d_->mapping_status_label_->setText(tr("建图: %1").arg(state));
        
        if (state == "运行中") {
            d_->mapping_status_label_->setStyleSheet("QLabel { color: green; }");
        } else if (state == "暂停") {
            d_->mapping_status_label_->setStyleSheet("QLabel { color: orange; }");
        } else if (state == "停止") {
            d_->mapping_status_label_->setStyleSheet("QLabel { color: black; }");
        } else {
            d_->mapping_status_label_->setStyleSheet("QLabel { color: red; }");
        }
    }
}

void MainWindow::onError(const QString& error)
{
    QMessageBox::critical(this, tr("错误"), error);
}

MainWindow::~MainWindow()
{
    // 保存设置
    writeSettings();

    // 清理资源
    if (d_->robot_controller_) {
        d_->robot_controller_->cleanup();
    }

    if (d_->rviz_view_) {
        d_->rviz_view_->cleanup();
    }
} 