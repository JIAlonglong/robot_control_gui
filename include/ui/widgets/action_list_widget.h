#pragma once

#ifndef ROBOT_CONTROL_GUI_ACTION_LIST_WIDGET_H
#define ROBOT_CONTROL_GUI_ACTION_LIST_WIDGET_H

#include <QWidget>
#include <memory>
#include "robot_controller.h"

class ActionBlock;

class ActionListWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ActionListWidget(QWidget* parent = nullptr);
    ~ActionListWidget() override;

    void setRobotController(const std::shared_ptr<RobotController>& controller);
    bool validateActions() const;
    void executeActions();
    void stopActions();

    void                addAction(ActionBlock* action);
    void                removeAction(int index);
    void                moveAction(int from, int to);
    void                clear();
    int                 count() const;
    ActionBlock*        actionAt(int index) const;
    QList<ActionBlock*> actions() const;

public Q_SLOTS:
    void onActionAdded(ActionBlock* action);
    void onActionRemoved(ActionBlock* action);
    void onActionMoved(ActionBlock* action, int from, int to);
    void onActionSelected(ActionBlock* action);
    void onActionStarted(ActionBlock* action);
    void onActionCompleted(ActionBlock* action);
    void onActionFailed(ActionBlock* action, const QString& error);

Q_SIGNALS:
    void actionAdded(int index);
    void actionRemoved(int index);
    void actionMoved(int from, int to);
    void actionSelected(int index);
    void actionStarted(int index);
    void actionCompleted(int index);
    void actionFailed(int index, const QString& error);
    void executionStarted();
    void executionStopped();
    void executionCompleted(bool success);
    void error(const QString& message);

protected:
    void dragEnterEvent(QDragEnterEvent* event) override;
    void dragMoveEvent(QDragMoveEvent* event) override;
    void dropEvent(QDropEvent* event) override;

private Q_SLOTS:
    void onAddNavigationAction();
    void onAddLoopAction();
    void onAddConditionAction();
    void onRemoveAction();
    void onMoveActionUp();
    void onMoveActionDown();
    void onActionSelectionChanged();
    void onActionExecutionCompleted(bool success);

private:
    void setupUi();
    void createToolBar();
    void updateButtons();
    void connectSignals();
    void updateActions();
    int  findDropIndex(const QPoint& pos) const;

    struct Private;
    std::unique_ptr<Private> d_;
};

#endif  // ROBOT_CONTROL_GUI_ACTION_LIST_WIDGET_H