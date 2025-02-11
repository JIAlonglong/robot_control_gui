/**
 * @file action_configurator.h
 * @brief 动作配置器界面类,用于可视化编辑和配置机器人动作序列
 * 
 * ActionConfigurator类提供了一个图形化界面,用于:
 * - 创建和编辑机器人动作序列
 * - 可视化动作流程图
 * - 配置动作参数
 * - 执行和监控动作序列
 */

#pragma once

#ifndef ROBOT_CONTROL_GUI_ACTION_CONFIGURATOR_H
#define ROBOT_CONTROL_GUI_ACTION_CONFIGURATOR_H

#include <QWidget>
#include <QPoint>
#include <QString>
#include <QVariant>
#include <QMap>
#include <memory>
#include <QIcon>
#include <QVariantMap>
#include <QJsonObject>
#include <QComboBox>
#include <QPushButton>
#include <QUndoStack>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QToolBar>
#include <QAction>
#include <QDockWidget>
#include <QLabel>
#include <QProgressBar>
#include <QStatusBar>
#include <QMenu>
#include <QMenuBar>
#include <QList>
#include <QSettings>

class ActionBlock;
class ActionBlockFactory;
class ActionSequenceManager;
class ActionBlockWidget;
class PropertyEditor;
class ConnectionWidget;
class WaypointWidget;

/**
 * @class ActionConfigurator
 * @brief 动作配置器类,用于创建和管理动作序列
 */
class ActionConfigurator : public QWidget {
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param factory 动作块工厂实例
     * @param parent 父窗口部件
     */
    explicit ActionConfigurator(const std::shared_ptr<ActionBlockFactory>& factory, QWidget* parent = nullptr);
    
    /**
     * @brief 析构函数
     */
    ~ActionConfigurator() override;

    /**
     * @brief 设置动作块工厂
     * @param factory 动作块工厂实例
     */
    void setActionBlockFactory(ActionBlockFactory* factory);

    /**
     * @brief 设置动作序列管理器
     * @param manager 动作序列管理器实例
     */
    void setActionSequenceManager(ActionSequenceManager* manager);

    /**
     * @brief 从文件加载配置
     * @param filename 配置文件路径
     */
    void loadConfig(const QString& filename);

    /**
     * @brief 保存配置到文件
     * @param filename 配置文件路径
     */
    void saveConfig(const QString& filename);

    /**
     * @brief 清除当前配置
     */
    void clear();

    /**
     * @brief 显示帮助信息
     */
    void showHelp();

    void setAction(ActionBlock* action);
    ActionBlock* currentAction() const;

    void setupUi();
    void connectSignals();

public Q_SLOTS:
    void onStartSequence();
    void onStopSequence();
    void onPauseSequence();
    void onResumeSequence();
    void onAddAction(const QString& type);
    void onRemoveAction();
    void onMoveActionUp();
    void onMoveActionDown();
    void onActionSelected(ActionBlockWidget* widget);
    void onConnectionStarted(ActionBlockWidget* widget, const QPointF& pos);
    void onConnectionFinished(ActionBlockWidget* widget, const QPointF& pos);
    void onBlockMoved(ActionBlockWidget* block, const QPointF& oldPos, const QPointF& newPos);
    void onPropertyChanged(const QString& name, const QVariant& value);
    void onSequenceCompleted();
    void undo();
    void redo();
    void addAction(const QString& type);
    void removeAction();
    void clearActions();
    void connectActions();
    void setCurrentAction(ActionBlock* action);
    void onWaypointMoved(int index, double x, double y, double theta);
    void onLoadConfig();
    void onSaveConfig();
    void onAddActionClicked();
    void onHelpRequested();
    void onActionChanged();
    void onValidationError(const QString& error);

Q_SIGNALS:
    void sequenceStarted();
    void sequenceStopped();
    void sequencePaused();
    void sequenceResumed();
    void sequenceCompleted();
    void undoAvailable(bool available);
    void redoAvailable(bool available);
    void actionAdded(ActionBlock* action);
    void actionRemoved(ActionBlock* action);
    void actionsConnected(ActionBlock* from, ActionBlock* to);
    void actionSelected(ActionBlock* action);
    void currentActionChanged(ActionBlock* action);
    void actionConfigured(ActionBlock* action);
    void configurationError(const QString& error);
    void actionStarted(int index);
    void actionCompleted(int index);
    void actionFailed(int index, const QString& error);

protected:
    void createToolBar();
    void createActionPalette();
    void createPropertyEditor();
    void createUndoView();
    void updateConnections();
    void createHelpDialog();

private:
    QJsonObject saveToJson() const;
    void loadFromJson(const QJsonObject& obj);

    std::shared_ptr<ActionBlockFactory> factory_;
    ActionSequenceManager* manager_{nullptr};
    ActionBlock* current_action_{nullptr};
    
    QPushButton* add_button_{nullptr};
    QComboBox* action_combo_{nullptr};
    QPushButton* start_button_{nullptr};
    QPushButton* pause_button_{nullptr};
    QPushButton* resume_button_{nullptr};
    QPushButton* stop_button_{nullptr};
    QPushButton* load_button_{nullptr};
    QPushButton* save_button_{nullptr};
    QPushButton* help_button_{nullptr};
    
    QUndoStack* undo_stack_{nullptr};
    PropertyEditor* property_editor_{nullptr};

    QVBoxLayout* main_layout_{nullptr};
    QPushButton* add_action_button_{nullptr};
    QPushButton* remove_action_button_{nullptr};
    QPushButton* start_sequence_button_{nullptr};
    QPushButton* stop_sequence_button_{nullptr};
    QComboBox* action_type_combo_{nullptr};
};

#endif // ROBOT_CONTROL_GUI_ACTION_CONFIGURATOR_H 