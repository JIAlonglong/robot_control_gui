#pragma once

#ifndef ROBOT_CONTROL_GUI_ACTION_SEQUENCE_MANAGER_H
#define ROBOT_CONTROL_GUI_ACTION_SEQUENCE_MANAGER_H

#include <QList>
#include <QObject>
#include <QTimer>
#include <QVariantMap>
#include <memory>

#include "action_block.h"

class ActionSequenceManager : public QObject
{
    Q_OBJECT

public:
    explicit ActionSequenceManager(QObject* parent = nullptr);
    ~ActionSequenceManager() override;

    void addAction(ActionBlock* action);
    void removeAction(ActionBlock* action);
    void removeAction(int index);
    void clearActions();
    void moveAction(int from, int to);

    bool                             isRunning() const;
    bool                             isPaused() const;
    int                              currentIndex() const;
    const std::vector<ActionBlock*>& actions() const;

    void saveToFile(const QString& filename);
    void loadFromFile(const QString& filename);

public Q_SLOTS:
    void start();
    void stop();
    void pause();
    void resume();
    void reset();

Q_SIGNALS:
    void started();
    void stopped();
    void paused();
    void resumed();
    void completed();
    void actionAdded(ActionBlock* action);
    void actionRemoved(ActionBlock* action);
    void actionMoved(int from, int to);
    void currentActionChanged(int index);
    void progressChanged(int progress);
    void error(const QString& message);

private Q_SLOTS:
    void onActionCompleted();
    void onActionError(const QString& message);
    void onTimeout();

private:
    void executeNextAction();
    void cleanup();

    struct Private;
    std::unique_ptr<Private> d_;
};

#endif  // ROBOT_CONTROL_GUI_ACTION_SEQUENCE_MANAGER_H